# Hanging Plotter

Arduino sketch and modules for an ESP32-based hanging plotter.

## Prerequisites

- Arduino CLI installed
- ESP32 core installed (`esp32:esp32`)

## Build

From this project directory, run:

```powershell
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 .
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

