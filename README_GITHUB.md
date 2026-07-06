# Flap Word Clock

Mechanical word clock built on ESP32. The clock displays time using rotating flap drums for hours and 5-minute blocks, plus two servo-driven indicators for `PAST / TO / blank` and the extra `1-4` minutes.

## Features

- ESP32-based controller with Wi-Fi time sync via NTP
- Two stepper-driven word drums:
  - hours: `ONE` to `TWELVE`
  - minutes: `blank`, `FIVE`, `TEN`, `QUARTER`, `TWENTY`, `TWENTY FIVE`, `HALF`, ...
- Two servo-driven indicators:
  - `PAST / TO / blank`
  - extra minutes `1-4`
- Built-in local calibration panel at `http://FlapWordClock.local`
- EEPROM persistence for:
  - stepper positions
  - servo fine calibration
  - summer/winter time selection
- Forward-only stepper movement logic, matched to the mechanical design

## Repository Files

- `flap_word_clock_github.ino` - GitHub-safe version of the Arduino sketch without private Wi-Fi credentials

## Hardware Overview

- `ESP32`
- `2x stepper motors`
- `2x servo motors`
- mechanical flap drums:
  - 12 hour flaps
  - 12 minute flaps

One flap change equals `512` half-steps. Each drum has `12` flaps.

## Time Display Logic

- `:00` shows only the hour word
- `:05` to `:30` uses `PAST`
- `:35` to `:55` uses `TO`
- from `:35` onward, the next hour is shown
- extra minutes `1-4` are shown with a separate servo indicator

Examples:

- `1:00` -> `ONE`
- `1:07` -> `FIVE PAST ONE` + `2 min`
- `2:15` -> `QUARTER PAST TWO`
- `4:43` -> `QUARTER TO FIVE` + `3 min`

## Calibration

The clock provides a browser-based calibration interface.

### Stepper calibration

For each drum:

- move using `+100`, `+10`, `+1`
- stop when the visible flap is centered
- choose the flap text from the list
- click `I can see this flap`

This also re-bases the internal step counter to a safe single-turn range.

### Servo calibration

Two servo calibration sections are available:

- `PAST / TO servo`
- `1-4 minute servo`

Use `-1` and `+1` for fine positioning, then click `Save position`.

### Time mode

The configuration page also lets the user choose:

- `Winter time`
- `Summer time`

This setting is stored in EEPROM and applied immediately.

## Setup

1. Open `flap_word_clock_github.ino`
2. Replace:
   - `YOUR_WIFI_SSID`
   - `YOUR_WIFI_PASSWORD`
3. Install the required Arduino libraries:
   - `ESP32Servo`
   - `AccelStepper`
4. Select an ESP32 board in Arduino IDE
5. Upload the sketch

## Notes

- The published sketch intentionally does not contain real Wi-Fi credentials.
- The project assumes forward-only motion for both steppers.
- Calibration and stored positions are used to keep the physical and logical state aligned across restarts.
