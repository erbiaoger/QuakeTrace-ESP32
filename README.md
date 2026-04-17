# QuakeTrace ESP32

ESP32-based piezo trigger logger for vibration/seismic-like events.

It captures trigger events from a piezo sensor, stores recent events in RAM, optionally persists full history to SPIFFS flash, supports BLE status/data export, and provides a Python plotting script for threshold tuning.

## Features

- Auto calibration on boot to estimate baseline and noise-driven thresholds.
- Trigger + rearm hysteresis (`triggerDelta` / `rearmDelta`) to reduce chatter.
- Deadtime and peak-capture window per event.
- Ring buffer in RAM (`MAX_EVENTS`) for recent events.
- Optional flash CSV logging (`/events.csv`) for full history.
- Serial commands for dump/clear/status/tuning workflow.
- BLE notify interface for status, command handling, and flash CSV export.
- Optional I2C OLED runtime status display.
- Python CLI tool to parse logs and generate tuning plots.

## Hardware

- ESP32 dev board.
- Piezo sensor + protection/clamp circuit.
- Optional I2C OLED (SSD1306 128x64 / SH1106 128x64 / SSD1306 128x32).

Default pins/config are in `include/config.h`.

- `PIN_PIEZO = 34` (ADC1 input).
- `PIN_LED = 2`.
- OLED I2C address auto-scan: `0x3C` / `0x3D`.

## Project Layout

- `src/main.cpp`: firmware logic (triggering, logging, serial/BLE/OLED).
- `include/config.h`: runtime configuration constants.
- `scripts/plot_signal.py`: plotting CLI for stream logs.
- `platformio.ini`: PlatformIO env and dependencies.
- `run.sh`: serial monitor shortcut.
- `upload.sh`: clean + upload + monitor shortcut.

## Build and Flash

Prerequisite: [PlatformIO Core](https://docs.platformio.org/).

```bash
pio run
```

Upload (example port):

```bash
pio run -t upload --upload-port /dev/cu.usbserial-0001
```

Monitor serial:

```bash
pio device monitor -b 115200
```

Or use the provided scripts:

```bash
./run.sh
./upload.sh
```

## Serial Command Reference

Send commands in serial monitor at 115200 baud.

| Command | Description |
|---|---|
| `status` | Print current runtime status and config summary. |
| `dump` | Dump RAM events in readable format. |
| `dumpcsv` | Dump CSV (flash full history if enabled, otherwise RAM CSV). |
| `dumpramcsv` | Dump RAM events in CSV only. |
| `dumpflash` | Dump flash CSV (`/events.csv`). |
| `clear` | Clear RAM event buffer. |
| `clearflash` | Clear flash CSV and keep header. |
| `clearall` | Clear RAM and flash logs. |
| `recal` | Re-run auto calibration and re-arm trigger. |
| `stream` | Show stream status/help. |
| `stream on` / `stream off` | Enable/disable real-time stream output. |
| `stream ms <20-200>` | Set stream interval in ms and enable stream. |
| `stream hz <1-50>` | Set stream rate in Hz and enable stream. |
| `wifiretry` | Retry Wi-Fi connection. |
| `bletest` | Send BLE test notify message. |
| `drv0` / `drv1` / `drv2` | Switch OLED driver mode. |
| `oledtest` | Draw OLED test pattern. |
| `oledon` / `oledoff` | Toggle OLED power-save mode. |

## BLE Interface

Configured in `include/config.h`:

- Device name: `DAS-ESP32` (default).
- Service UUID: `12345678-1234-1234-1234-1234567890ab`.
- Characteristic UUID: `12345678-1234-1234-1234-1234567890ac`.

BLE write commands:

- `help`
- `status`
- `dumpflash` (alias: `dumpcsv`)
- `stopdump`
- `clearflash`

During flash dump over BLE, stream is framed by:

- `#BEGIN_FLASH_CSV`
- `#END_FLASH_CSV`

## Data Formats

Real-time signal stream CSV line:

```text
sig_ms,adc,delta,base,td,rd,armed
8925,1327,560,767,4095,1433,1
```

Event CSV line (`dumpcsv`/`dumpflash`):

```text
index,local_ms,local_us,trigger_adc,peak_adc
```

## Plotting Workflow

Use `scripts/plot_signal.py` to visualize stream logs for threshold tuning.

1. Record serial output to a file (`stream_raw.log` or CSV-like stream lines).
2. Run the plotting CLI with `uv`.

```bash
uv run --with matplotlib python scripts/plot_signal.py stream_raw.log -o signal_plot.png
```

Optional clipping and preview:

```bash
uv run --with matplotlib python scripts/plot_signal.py signal_stream.csv --t-start 2 --t-end 12 --show
```

CLI usage:

```bash
uv run python scripts/plot_signal.py --help
```

## Configuration Notes

Tune values in `include/config.h` as needed:

- Trigger behavior: `PIEZO_TRIGGER_THRESHOLD`, `PIEZO_REARM_THRESHOLD`, `TRIGGER_DEADTIME_MS`, `PEAK_WINDOW_US`.
- Auto-calibration: `ENABLE_AUTO_CALIBRATION`, `AUTO_*` constants.
- Logging: `ENABLE_FLASH_LOG`, `MAX_EVENTS`, `FLASH_LOG_PATH`.
- Wireless: `ENABLE_WIFI`, `WIFI_*`, `ENABLE_BLE`, `BLE_*`.
- Stream/debug: `ENABLE_DEBUG`, `DEBUG_HEARTBEAT_MS`, `SIGNAL_STREAM_*`.
- OLED: `ENABLE_OLED`, `OLED_DRIVER_MODE`, `OLED_CONTRAST`, `OLED_REFRESH_MS`.

## Troubleshooting

- No OLED output: verify wiring and try `drv0` / `drv1` / `drv2`, then `oledtest`.
- No stream samples parsed: enable `stream on` before logging, or pass a raw serial log containing stream lines.
- Wi-Fi not connecting: set `WIFI_SSID` / `WIFI_PASSWORD` and run `wifiretry`.

## License

No license file is currently included. Add one before public distribution if needed.
