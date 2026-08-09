#!/usr/bin/env python3

import argparse
import math
import re
import socket
import sys
import time
from dataclasses import dataclass


DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 8080
MAX_DRAWING_WIDTH = 120.0
MAX_DRAWING_HEIGHT = 80.0
WORD_PATTERN = re.compile(
    r"([A-Za-z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
)


class GCodeServerError(RuntimeError):
    pass


class GCodeProcessingError(ValueError):
    pass


@dataclass
class GCodeMove:
    line_index: int
    command: int
    x: float
    y: float


@dataclass
class GCodeTransformResult:
    lines: list[str]
    move_count: int
    original_width: float
    original_height: float
    scale: float


def remove_comments(line):
    result = []
    in_parentheses = False

    for character in line:
        if character == ";" and not in_parentheses:
            break
        if character == "(" and not in_parentheses:
            in_parentheses = True
            continue
        if character == ")" and in_parentheses:
            in_parentheses = False
            continue
        if not in_parentheses:
            result.append(character)

    return "".join(result)


def format_coordinate(value):
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")


def process_gcode(lines, max_width=MAX_DRAWING_WIDTH, max_height=MAX_DRAWING_HEIGHT):
    source_lines = [line.rstrip("\r\n") for line in lines]
    moves = []
    current_x = 0.0
    current_y = 0.0
    is_absolute_mode = True
    is_millimeter_mode = True

    for line_index, line in enumerate(source_lines):
        words = [
            (match.group(1).upper(), float(match.group(2)))
            for match in WORD_PATTERN.finditer(remove_comments(line))
        ]
        has_move_command = False
        move_command = 0
        has_x = False
        has_y = False
        has_unsupported_command = False
        next_x = current_x
        next_y = current_y

        for word, value in words:
            if not math.isfinite(value):
                raise GCodeProcessingError(f"line {line_index + 1}: invalid numeric value")

            if word == "G":
                command = int(value)
                if abs(value - command) > 0.0001:
                    raise GCodeProcessingError(f"line {line_index + 1}: invalid G code")
                if command in (0, 1):
                    has_move_command = True
                    move_command = command
                elif command == 20:
                    is_millimeter_mode = False
                elif command == 21:
                    is_millimeter_mode = True
                elif command == 90:
                    is_absolute_mode = True
                elif command == 91:
                    is_absolute_mode = False
                else:
                    has_unsupported_command = True
            elif word == "X":
                next_x = value
                has_x = True
            elif word == "Y":
                next_y = value
                has_y = True

        if has_unsupported_command or not has_move_command or (not has_x and not has_y):
            continue

        if not is_absolute_mode or not is_millimeter_mode:
            raise GCodeProcessingError(
                f"line {line_index + 1}: drawing moves must use G21 millimeters and G90 absolute coordinates"
            )

        current_x = next_x
        current_y = next_y
        moves.append(GCodeMove(line_index, move_command, current_x, current_y))

    if not moves:
        raise GCodeProcessingError("no supported XY movement commands found")

    min_x = min(move.x for move in moves)
    max_x = max(move.x for move in moves)
    min_y = min(move.y for move in moves)
    max_y = max(move.y for move in moves)
    original_width = max_x - min_x
    original_height = max_y - min_y
    scale_candidates = []

    if original_width > 0:
        scale_candidates.append(max_width / original_width)
    if original_height > 0:
        scale_candidates.append(max_height / original_height)

    scale = min(scale_candidates) if scale_candidates else 1.0
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    processed_lines = source_lines.copy()

    for move in moves:
        transformed_x = (move.x - center_x) * scale
        transformed_y = (move.y - center_y) * scale
        processed_lines[move.line_index] = (
            f"G21 G90 G{move.command} "
            f"X{format_coordinate(transformed_x)} Y{format_coordinate(transformed_y)}"
        )

    return GCodeTransformResult(
        processed_lines,
        len(moves),
        original_width,
        original_height,
        scale,
    )


def configure_keepalive(connection):
    connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

    if hasattr(socket, "TCP_KEEPIDLE"):
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
    if hasattr(socket, "TCP_KEEPINTVL"):
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
    if hasattr(socket, "TCP_KEEPCNT"):
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
    if hasattr(socket, "SIO_KEEPALIVE_VALS"):
        connection.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 10000, 3000))


def open_connection(host, port, connect_timeout):
    connection = socket.create_connection((host, port), timeout=connect_timeout)
    configure_keepalive(connection)
    connection.settimeout(None)
    return connection, connection.makefile("rb")


def close_connection(connection, response_stream):
    if response_stream is not None:
        response_stream.close()
    if connection is not None:
        connection.close()


def send_lines(lines, host=DEFAULT_HOST, port=DEFAULT_PORT, connect_timeout=5.0, retry_delay=1.0):
    connection = None
    response_stream = None

    try:
        for line_number, line in enumerate(lines, start=1):
            command = line.rstrip("\r\n")

            while True:
                try:
                    if connection is None:
                        connection, response_stream = open_connection(host, port, connect_timeout)
                        print(f"Connected to {host}:{port}")

                    connection.sendall(command.encode("utf-8") + b"\n")
                    response_bytes = response_stream.readline()
                    if not response_bytes:
                        raise ConnectionError("server closed the connection")

                    response = response_bytes.decode("utf-8", errors="replace").strip()
                    print(f"{line_number}: {response}")

                    if response.startswith("ERR"):
                        raise GCodeServerError(f"line {line_number}: {response}")
                    if not response.startswith("OK"):
                        raise ConnectionError(f"unexpected response: {response}")

                    break
                except GCodeServerError:
                    raise
                except (ConnectionError, OSError) as error:
                    print(f"Connection lost ({error}); retrying line {line_number}", file=sys.stderr)
                    close_connection(connection, response_stream)
                    connection = None
                    response_stream = None
                    time.sleep(retry_delay)
    finally:
        close_connection(connection, response_stream)


def parse_args():
    parser = argparse.ArgumentParser(description="Stream a G-code file to the hanging plotter.")
    parser.add_argument("file", help="G-code file to stream")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"plotter address (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"server port (default: {DEFAULT_PORT})")
    parser.add_argument("--connect-timeout", type=float, default=5.0, help="connection timeout in seconds")
    parser.add_argument("--retry-delay", type=float, default=1.0, help="delay before reconnecting")
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        with open(args.file, "r", encoding="utf-8") as gcode_file:
            transformed = process_gcode(gcode_file)
            print(
                f"Prepared {transformed.move_count} moves from a "
                f"{transformed.original_width:g} x {transformed.original_height:g} mm drawing "
                f"at {transformed.scale:g}x scale"
            )
            send_lines(
                transformed.lines,
                host=args.host,
                port=args.port,
                connect_timeout=args.connect_timeout,
                retry_delay=args.retry_delay,
            )
    except (GCodeProcessingError, GCodeServerError, OSError) as error:
        print(error, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Cancelled", file=sys.stderr)
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
