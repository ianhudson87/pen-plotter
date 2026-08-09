# Hanging Plotter

Arduino sketch and modules for an ESP32-based hanging plotter. The ESP32 creates a Wi-Fi access point and accepts G-code as a newline-delimited TCP stream, so the full drawing never needs to be stored in microcontroller memory.

The sketch owns the TCP server and its Wi-Fi lifecycle, then injects the server into `MovementPlanner`. The parser converts supported G-code into origin-relative coordinates, while the planner applies the configured plotter origin and delivers movement targets. The state machine retains its original calculate/move cycle: each call to `MovementPlanner::GetNextPos()` waits for the next executable G-code line and returns its XY target.

## Configuration

The main settings are near the top of `hangingPlotter.ino`:

- Access point name: `HangingPlotter`
- Access point password: `plotter123`
- TCP port: `8080`
- G-code origin: `gcodeOriginOffset`, in millimeters

Change the default password before using the plotter around other people.

## Supported G-code

- `G0` and `G1` absolute XY movement
- All coordinates, geometry dimensions, and string lengths use millimeters internally
- The movement planner adds the configured G-code origin to every coordinate
- X or Y may be omitted and retains its previous value
- `F` and other unsupported words are ignored
- Comments beginning with `;` and comments inside parentheses are ignored
- `G20` or `G91` disables movement until a later `G21` or `G90`

Unsupported commands return `OK SKIPPED`. `G0` and `G1` currently behave the same because there is no pen-lift actuator or separate rapid speed.

## Streaming Protocol

The server holds at most one line of G-code. Send one UTF-8 command followed by a newline, then wait for one response:

- `OK`: both motors completed the move
- `OK SKIPPED`: the line did not produce a supported movement
- `ERR <reason>`: the line was malformed, too long, or the plotter was reset

The host should not send another command before receiving a response. A connection may remain open for the entire drawing. If it drops, reconnect and resend the unacknowledged line; absolute targets make this safe even when the previous move finished before its acknowledgment was lost.

## Sending a File

1. Power on the ESP32 and connect the computer to the `HangingPlotter` Wi-Fi network.
2. Complete the existing button-driven lowering and left/right retraction sequence. The plotter then waits for commands.
3. Stream a file with the included Python client:

```powershell
python ..\tools\send_gcode.py path\to\drawing.gcode
```

The ESP32 access point normally uses `192.168.4.1`. Override connection settings when needed:

```powershell
python ..\tools\send_gcode.py drawing.gcode --host 192.168.4.1 --port 8080
```

Before connecting, the sender reads the drawing and uniformly scales it to fit within the plotter's 120 mm by 80 mm reachable area. It preserves the original aspect ratio and centers the drawing so that `(min + max) / 2` is zero on both axes. The transformed coordinates are sent as absolute millimeter moves, and the movement planner adds the configured origin to place that zero point at the physical center.

After preprocessing, the sender waits for each move to complete, enables TCP keepalive, and reconnects and retries the current unacknowledged line after a disconnect. Drawing moves must use `G21` millimeters and `G90` absolute coordinates so they can be transformed safely.

Because `GetNextPos()` waits inside the calculating state, the main sketch loop does not process button input while it is waiting for the next command. Start the sender when calibration is complete, and stop a waiting job from the computer side.

## Prerequisites

- Arduino CLI installed
- ESP32 core installed (`esp32:esp32`)

## Build

From this project directory, run:

```powershell
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

## Sender Tests

The desktop sender uses only the Python standard library. Its scaling, centering, and reconnect behavior can be checked without hardware:

```powershell
python ..\tools\test_send_gcode.py
```

## Upload

List serial ports:

```powershell
arduino-cli board list
```

Upload to your ESP32 (replace `COM3` as needed):

```powershell
arduino-cli upload --fqbn esp32:esp32:esp32 --port COM3 .
```

## Serial Monitor

```powershell
arduino-cli monitor --port COM3 --config baudrate=115200
```

