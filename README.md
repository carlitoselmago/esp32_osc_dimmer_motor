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

