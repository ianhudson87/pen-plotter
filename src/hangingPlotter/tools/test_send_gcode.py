import socket
import threading
import unittest

from send_gcode import GCodeProcessingError, process_gcode, send_lines


class RetryServer:
    def __init__(self):
        self.received = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.port = self.listener.getsockname()[1]
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        first_connection, _ = self.listener.accept()
        with first_connection, first_connection.makefile("rb") as stream:
            self.received.append(stream.readline().decode().strip())

        second_connection, _ = self.listener.accept()
        with second_connection, second_connection.makefile("rb") as stream:
            self.received.append(stream.readline().decode().strip())
            second_connection.sendall(b"OK\n")
            self.received.append(stream.readline().decode().strip())
            second_connection.sendall(b"OK SKIPPED\n")

        self.listener.close()


class SendGCodeTests(unittest.TestCase):
    def test_scales_landscape_drawing_to_width_and_centers_it(self):
        result = process_gcode(["G1 X0 Y0\n", "G1 X200 Y50\n"])

        self.assertEqual(result.lines, ["G21 G90 G1 X-60 Y-15", "G21 G90 G1 X60 Y15"])
        self.assertEqual(result.scale, 0.6)

    def test_scales_portrait_drawing_to_height_and_centers_it(self):
        result = process_gcode(["G0 X0 Y0\n", "G1 X50 Y200\n"])

        self.assertEqual(result.lines, ["G21 G90 G0 X-10 Y-40", "G21 G90 G1 X10 Y40"])
        self.assertEqual(result.scale, 0.4)

    def test_scales_up_and_preserves_aspect_ratio(self):
        result = process_gcode(["G1 X10 Y20\n", "G1 X30 Y30\n"])

        self.assertEqual(result.lines, ["G21 G90 G1 X-60 Y-30", "G21 G90 G1 X60 Y30"])
        self.assertEqual(result.scale, 6.0)

    def test_expands_omitted_axes_before_transforming(self):
        result = process_gcode(["G1 X10 Y20\n", "G1 X30\n", "G1 Y60\n"])

        self.assertEqual(
            result.lines,
            [
                "G21 G90 G1 X-20 Y-40",
                "G21 G90 G1 X20 Y-40",
                "G21 G90 G1 X20 Y40",
            ],
        )

    def test_rejects_moves_that_cannot_be_safely_transformed(self):
        with self.assertRaisesRegex(GCodeProcessingError, "G21 millimeters and G90"):
            process_gcode(["G91\n", "G1 X10 Y20\n"])

    def test_retries_unacknowledged_line_after_disconnect(self):
        server = RetryServer()
        server.start()

        send_lines(
            ["G1 X10 Y20\n", "G90\n"],
            host="127.0.0.1",
            port=server.port,
            connect_timeout=1.0,
            retry_delay=0.01,
        )
        server.thread.join(timeout=1.0)

        self.assertEqual(server.received, ["G1 X10 Y20", "G1 X10 Y20", "G90"])


if __name__ == "__main__":
    unittest.main()
