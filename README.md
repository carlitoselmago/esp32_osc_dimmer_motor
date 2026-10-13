# esp32_osc_dimmer_motor
dimmer and stepper motor controller via OSC via wifi

osc_controller.ino

## Dependencies

Board support (Arduino IDE → Boards Manager)
- https://wiki.elegoo.com/oshw-getting-started-&-kits/download-and-install
- Arduino IDE → *File → Preferences* → add to "Additional Board URLs":
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- `esp32` by Espressif Systems — install via Boards Manager, search "esp32"

Libraries (Arduino IDE → Library Manager)
- `TMCStepper` by Teemuatlut — controls the TMC2209 stepper driver over UART
- `OSC` by CNMAT — parses incoming OSC/UDP messages

Built-in (no install needed, bundled with the ESP32 core)
- `WiFi.h`
- `WiFiUdp.h`
- `HardwareSerial.h`


# Slider by moekoe

https://www.instructables.com/DIY-Motorized-Camera-Slider-From-Four-3D-Printed-P/


# PENDING: Krida dimmer testing (delete this section after testing)

Nothing below has been run on hardware yet; both sketches only compile (ESP32 core 3.3.12).

## Before powering the module
- [ ] Check the module's supply voltage and ZC output level. ESP32 pins are 3.3V only and not
      5V tolerant, so a 5V ZC output needs a divider / level shifter before D4.
- [ ] Confirm the mains terminals against the module's own markings (dimmer sits in the live line).

## 1. `krida_dimmer_test` (bench, one bulb)
- [ ] Zero-cross arrives: Serial prints `Measured half-cycle: ~10000 us` (50Hz) or ~8333 us (60Hz).
      If you get "No zero-cross detected": wiring, supply or logic level. If the period is ~double
      or half: wrong edge or noisy ZC. Set `MAINS_FREQ_HZ` if the mains is 60Hz.
- [ ] Gate polarity: gate HIGH = conducting (bulb lights at 100%, off at 0%). If inverted, the
      module's gate is active-low and the sketch needs flipping.
- [ ] Pot sweep is smooth with no flicker or flashing, in both directions.
- [ ] Find the lowest stable level with the Serial override (type 0-100) and set
      `DIM_MIN_PERCENT` (placeholder is 5).
- [ ] Check the top end holds cleanly at 100%, and set `DIM_MAX_PERCENT` if it glitches (placeholder is 100).
- [ ] Pot noise at the dimmest end doesn't flicker the bulb on/off (hysteresis gap, `DIM_OFF_EXIT_PERCENT`).
- [ ] Unplug ZC while running: output goes off and the warning prints once.
- [ ] Flip `USE_POT_CONTROL` to `false` and check the auto 0->100->0% sweep.

## 2. `osc_controller` (full firmware)
- [ ] Copy the tuned `DIM_MIN_PERCENT` / `DIM_MAX_PERCENT` / `MAINS_FREQ_HZ` values from the test sketch.
- [ ] Pot mode (`USE_ANALOG_INPUT = true`): dimmer pot on GPIO34 works (left = on, right = off) and
      the onboard LED follows the level.
- [ ] OSC mode (`USE_ANALOG_INPUT = false`): `/dimmer2` works with a float 0.0-1.0 and an int 0-255.
- [ ] No dimmer flicker with WiFi connected (WiFi interrupt latency can jitter the gate-off edge).
- [ ] No dimmer flicker while the slider is moving, and no stepper stutter with the dimmer lit,
      including the fastest pot jog speed.
- [ ] Bulb stays off during boot calibration.
- [ ] Without mains connected: "No zero-cross detected" prints once, then stays quiet.
- [ ] Serial override: a number 0-100 holds that level, `pot` / `auto` hands control back.
- [ ] Optional: `DEBUG_PRINT_DIMMER_POT` shows the gate on-time and measured half-cycle.
