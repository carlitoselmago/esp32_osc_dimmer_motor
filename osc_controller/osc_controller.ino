// Camera slider + dimmer - final firmware
// - Connects to WiFi
// - Listens for OSC over UDP: /slider (0.0-1.0 target position), /velocidad (seconds to reach it),
//   /dimmer (0.0-1.0 or 0-255 brightness for the LED/dimmer output)
// - Slider moves always use cubic ease-in-out
// - On boot: auto-calibrates both limits via StallGuard, then centers. OSC is ignored until this finishes.
// - Once calibrated, slider position is trusted from step counting. StallGuard is only used during
//   calibration, not monitored during normal moves.
//
// Libraries needed (Library Manager):
//   TMCStepper (Teemuatlut)
//   OSC (CNMAT)
//
// ASSUMPTIONS MADE (adjust if wrong):
//   - OSC addresses are "/slider", "/velocidad", "/dimmer" (with leading slash)
//   - /slider argument is a float 0.0-1.0 (0 = one end, 1 = the other end)
//   - /velocidad argument is a float, the number of SECONDS the move should take
//     (it's stored and reused for the next /slider message; send it whenever you want to change speed)
//   - /dimmer argument is a float 0.0-1.0 or an int 0-255, either is accepted
//   - UDP listen port is 9000 for everything (unified from the two sketches, dimmer used to be 8000)
//   - rms_current(800) is a placeholder, set to your motor's actual rated current in mA

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <TMCStepper.h>
#include <HardwareSerial.h>

// ---------- Wiring ----------
const int EN_PIN = 27;
const int DIR_PIN = 25;
const int STEP_PIN = 26;
const int RX_PIN = 32;
const int TX_PIN = 33;
const int DIAG_PIN = 14;

#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f

HardwareSerial driverSerial(1);
TMC2209Stepper driver(&driverSerial, R_SENSE, DRIVER_ADDRESS);

// ---------- Dimmer output ----------
const int ledPin = 2;      // onboard LED, debug/visual feedback
const int dimmerPin = 15;  // D15 -> DEWIN PWM dimmer module
const int ledChannel = 0;
const int dimmerChannel = 1;
const int ledcFreq = 5000;
const int dimmerLedcFreq = 200;  // dentro del rango 1-500Hz del YYAC-3S
const int ledcResolution = 8;    // 0-255

const int DIM_MIN = 130;  // rough guess for where variation starts to matter — tune this
const int DIM_MAX = 250;  // pulled back from the literal ceiling (255) since some bulbs glitch off/on right at max
// Hysteresis around the off point: without a gap between the "turn off" and "turn back on"
// thresholds, ADC noise right at the boundary flickers the output between 0 and DIM_MIN every loop.
const float DIM_OFF_ENTER = 0.03f;  // turn off once norm drops to/below this
const float DIM_OFF_EXIT = 0.07f;   // only turn back on once norm rises above this
const int DIM_MAX_DUTY_STEP = 4;    // max duty change per update; keeps every transition a real pulse
                                     // sequence instead of a single static jump the module can miss

// ---------- WiFi / OSC ----------
const char *WIFI_SSID = "MANGO";
const char *WIFI_PASS = "remotamente";
const unsigned int OSC_PORT = 9000;
WiFiUDP udp;

// OSC addresses - edit these if you want to rename them, used below in loop()
char oscAddressSlider[32] = "/slider2";
char oscAddressVelocidad[32] = "/velocidad2";
char oscAddressDimmer[32] = "/dimmer2";

// ---------- Motion parameters ----------
const int CAL_STEP_DELAY_US = 800;  // slow, safe speed used only during calibration
const int STEP_PULSE_US = 3;        // step pulse width during normal moves
const int BACKOFF_STEPS = 200;      // steps to back off from each limit after detecting it
const int STALL_DEBOUNCE = 20;      // consecutive HIGH DIAG reads required to trust a stall (how many signals of stall to call stall, initial value 30)

// ---------- State ----------
long minSteps = 0;
long maxSteps = 0;
long currentSteps = 0;
bool calibrated = false;

bool moving = false;
long moveStartPos = 0;
long moveTargetPos = 0;
unsigned long moveStartTime = 0;
unsigned long moveDurationMs = 3000;

const float MIN_VELOCIDAD_SEC = 10.0f;   // fastest allowed transition
const float MAX_VELOCIDAD_SEC = 100.0f;  // slowest allowed transition
float lastVelocidadSec = 10.0;           // default move duration until /velocidad is received

// ---------- Input mode ----------
// false = WiFi/OSC control (default). true = local potentiometers, no WiFi/OSC at all.
// Flip this and reflash to switch modes.
const bool USE_ANALOG_INPUT = true;

// ---------- DEBUG ----------
// TEMPORARY: when true, skips homing/calibration entirely (no StallGuard sensing)
// and lets the analog pots drive the dimmer/slider right away, with no travel limits.
// Set back to false before real use on the actual slider hardware.
const bool DEBUG_SKIP_CALIBRATION = false;
// Placeholder travel range used only when calibration is skipped, since the speed
// math below needs a maxSteps to divide by (real calibration would set this from
// the actual measured range). Tune if slider feels too fast/slow in debug mode.
const long DEBUG_NOMINAL_MAX_STEPS = 4000;
// Uncomment to print slider pot readings (raw/norm/disp/forward/step interval) ~4x/sec.
#define DEBUG_PRINT_SLIDER_POT

// ---------- Analog input (potentiometer) wiring ----------
const int POT_SLIDER_PIN = 35;  // 35 wiper -> GPIO34 (ADC1, input-only)
const int POT_DIMMER_PIN = 34;  // 34 wiper -> GPIO35 (ADC1, input-only)
const float ADC_MAX = 4095.0f;
const float POT_DEADBAND = 0.15f;  // fraction around center (0.5) treated as "stopped"
const float POT_SPEED_CURVE = 1.4f;  // >1 = speed ramps up more sharply away from center
const float POT_MIN_STEPS_PER_SEC = 100.0f;   // slowest jog speed, just past the deadband
const float POT_MAX_STEPS_PER_SEC = 1300.0f;  // fastest jog speed, at full pot deflection

// ---------- Easing ----------
const float EASE_MIX = 0.5f;  // 1.0 = full cubic ease, 0.0 = pure linear (no easing)

float easeInOutCubic(float t) {
  float eased;
  if (t < 0.5f) eased = 4.0f * t * t * t;
  else {
    float f = -2.0f * t + 2.0f;
    eased = 1.0f - (f * f * f) / 2.0f;
  }
  return EASE_MIX * eased + (1.0f - EASE_MIX) * t;
}

// ---------- Low-level step helpers ----------
void stepOnce(bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_PULSE_US);
}

void stepOnceSlow(bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(CAL_STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(CAL_STEP_DELAY_US);
}

bool stalledDebounced() {
  static int streak = 0;
  bool diagHigh = digitalRead(DIAG_PIN) == HIGH;
  if (diagHigh) {
    streak++;
  } else {
    streak = 0;
  }

#ifdef DEBUG_PRINT_SLIDER_POT
  static unsigned long lastSgDebugMs = 0;
  if (millis() - lastSgDebugMs >= 100) {
    lastSgDebugMs = millis();
    Serial.print("SG_RESULT: ");
    Serial.print(driver.SG_RESULT());
    Serial.print("  DIAG: ");
    Serial.print(diagHigh);
    Serial.print("  streak: ");
    Serial.println(streak);
  }
#endif

  if (streak >= STALL_DEBOUNCE) {
    streak = 0;
    return true;
  }
  return false;
}

// ---------- Calibration ----------
void calibrate() {
  Serial.println("Calibrating: searching for limit A...");
  while (!stalledDebounced()) {
    stepOnceSlow(false);  // toward limit A
  }
  Serial.println("Limit A found, backing off...");
  for (int i = 0; i < BACKOFF_STEPS; i++) stepOnceSlow(true);
  currentSteps = 0;
  minSteps = 0;

  Serial.println("Searching for limit B...");
  long traveled = 0;
  while (!stalledDebounced()) {
    stepOnceSlow(true);  // toward limit B
    traveled++;
  }
  Serial.println("Limit B found, backing off...");
  for (int i = 0; i < BACKOFF_STEPS; i++) stepOnceSlow(false);
  maxSteps = traveled - BACKOFF_STEPS;
  currentSteps = maxSteps;

  calibrated = true;
  Serial.print("Calibration done. Range (steps): ");
  Serial.println(maxSteps);

  Serial.println("Moving to center...");
  long center = maxSteps / 2;
  bool forward = center > currentSteps;
  while (currentSteps != center) {
    stepOnceSlow(forward);
    currentSteps += forward ? 1 : -1;
  }
  Serial.println("Centered. Ready for OSC.");
}

// ---------- OSC callbacks ----------
void sliderCallback(OSCMessage &msg) {
  if (!calibrated) return;  // safety: ignore until calibration is done
  float norm = msg.getFloat(0);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;

  moveStartPos = currentSteps;
  moveTargetPos = (long)round(norm * maxSteps);
  moveStartTime = millis();
  moveDurationMs = (unsigned long)(lastVelocidadSec * 1000.0f);
  moveDurationMs = constrain(moveDurationMs, (unsigned long)(MIN_VELOCIDAD_SEC * 1000.0f), (unsigned long)(MAX_VELOCIDAD_SEC * 1000.0f));
  moving = true;

  Serial.print("New target: ");
  Serial.print(norm);
  Serial.print("  duration(ms): ");
  Serial.println(moveDurationMs);
}

void velocidadCallback(OSCMessage &msg) {
  float raw;
  if (msg.isFloat(0)) {
    raw = msg.getFloat(0);
  } else if (msg.isInt(0)) {
    raw = (float)msg.getInt(0);
  } else {
    Serial.println("Unsupported /velocidad2 argument type");
    return;
  }
  raw = constrain(raw, 0.0f, 1.0f);

  float v = MIN_VELOCIDAD_SEC + raw * (MAX_VELOCIDAD_SEC - MIN_VELOCIDAD_SEC);
  lastVelocidadSec = v;
  Serial.print("New duration set (s): ");
  Serial.println(v);
}
void dimmerCallback(OSCMessage &msg) {
  float value;
  if (msg.isFloat(0)) {
    value = msg.getFloat(0);
  } else if (msg.isInt(0)) {
    value = (float)msg.getInt(0);
  } else {
    Serial.println("Unsupported /dimmer argument type");
    return;
  }

  Serial.print("dimmer raw: ");
  Serial.print(value, 4);
  Serial.print("  isFloat: ");
  Serial.print(msg.isFloat(0));
  Serial.print("  isInt: ");
  Serial.println(msg.isInt(0));

  setDimmer(value);
}


void setDimmer(float value) {
  float norm;
  if (value <= 1.0f && value >= 0.0f) {
    norm = value;
  } else {
    norm = constrain(value, 0.0f, 255.0f) / 255.0f;
  }

  // Hysteresis: stay off until norm rises past DIM_OFF_EXIT, stay on until it falls
  // to/below DIM_OFF_ENTER. Prevents ADC noise at the boundary from toggling on/off rapidly.
  static bool dimmerOff = true;
  if (dimmerOff) {
    if (norm > DIM_OFF_EXIT) dimmerOff = false;
  } else {
    if (norm <= DIM_OFF_ENTER) dimmerOff = true;
  }

  int targetDuty;
  if (dimmerOff) {
    targetDuty = 0;
  } else {
    float upperNorm = (norm - DIM_OFF_ENTER) / (1.0f - DIM_OFF_ENTER);
    upperNorm = constrain(upperNorm, 0.0f, 1.0f);
    targetDuty = DIM_MIN + (int)(upperNorm * (DIM_MAX - DIM_MIN));
  }
  targetDuty = constrain(targetDuty, 0, DIM_MAX);

  // Slew-limit: always step toward the target rather than jumping straight to it, so the
  // module always sees a real pulse sequence rather than an instantaneous static change.
  static float currentDuty = 0.0f;
  if (targetDuty > currentDuty) {
    currentDuty = min((float)targetDuty, currentDuty + DIM_MAX_DUTY_STEP);
  } else if (targetDuty < currentDuty) {
    currentDuty = max((float)targetDuty, currentDuty - DIM_MAX_DUTY_STEP);
  }
  int duty = constrain((int)(currentDuty + 0.5f), 0, 255);

#ifdef DEBUG_PRINT_SLIDER_POT
  static unsigned long lastFinalDutyDebugMs = 0;
  if (millis() - lastFinalDutyDebugMs >= 250) {
    lastFinalDutyDebugMs = millis();
    Serial.print("  -> final duty: ");
    Serial.println(duty);
  }
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ledPin, duty);
  ledcWrite(dimmerPin, duty);
#else
  ledcWrite(ledChannel, duty);
  ledcWrite(dimmerChannel, duty);
#endif
}

// ---------- Dimmer debug: manual duty override over Serial ----------
// Type a number 0-255 + Enter in the Serial Monitor to hold that exact duty steady
// (bypassing the pot and all hysteresis/slew logic), so you can characterize exactly
// which duty values flicker vs. hold cleanly on the real hardware. Type "auto" or "pot"
// to hand control back to the potentiometer.
int dimmerDebugOverride = -1;  // -1 = no override, pot is in control

void checkDimmerDebugSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("auto") || line.equalsIgnoreCase("pot")) {
    dimmerDebugOverride = -1;
    Serial.println("Dimmer override OFF: pot back in control.");
    return;
  }

  int duty = line.toInt();
  duty = constrain(duty, 0, 255);
  dimmerDebugOverride = duty;
  Serial.print("Dimmer override duty: ");
  Serial.println(duty);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ledPin, duty);
  ledcWrite(dimmerPin, duty);
#else
  ledcWrite(ledChannel, duty);
  ledcWrite(dimmerChannel, duty);
#endif
}

// ---------- Analog (potentiometer) control ----------
// Dimmer pot: direct proportional, same as an OSC 0.0-1.0 value.
void handleAnalogDimmer() {
  int raw = analogRead(POT_DIMMER_PIN);
  float norm = 1.0f - (raw / ADC_MAX);  // inverted: left = on, right = off

#ifdef DEBUG_PRINT_SLIDER_POT
  static unsigned long lastDimmerDebugMs = 0;
  if (millis() - lastDimmerDebugMs >= 250) {
    lastDimmerDebugMs = millis();
    Serial.print("dimmer raw: ");
    Serial.print(raw);
    Serial.print("  norm: ");
    Serial.println(norm, 3);
  }
#endif

  setDimmer(norm);
}

// Slider pot: center = stopped. Turning away from center drives the slider toward
// that side, with distance from center setting speed, directly in steps/sec
// (POT_MIN_STEPS_PER_SEC..POT_MAX_STEPS_PER_SEC), independent of OSC's /velocidad range.
void handleAnalogSlider() {
  static unsigned long lastStepMicros = 0;

  int raw = analogRead(POT_SLIDER_PIN);
  float norm = raw / ADC_MAX;         // 0.0 - 1.0
  float disp = (norm - 0.5f) * 2.0f;  // -1.0 .. 0 (center) .. 1.0

  float mag = fabs(disp);

#ifdef DEBUG_PRINT_SLIDER_POT
  static unsigned long lastDebugPrintMs = 0;
  if (millis() - lastDebugPrintMs >= 250) {
    lastDebugPrintMs = millis();
    Serial.print("raw: ");
    Serial.print(raw);
    Serial.print("  norm: ");
    Serial.print(norm, 3);
    Serial.print("  disp: ");
    Serial.println(disp, 3);
  }
#endif

  if (mag < POT_DEADBAND) return;  // centered: stay still

  // remap magnitude from [deadband..1.0] to [0..1] so speed ramps smoothly from the deadband edge
  float speedFrac = (mag - POT_DEADBAND) / (1.0f - POT_DEADBAND);
  speedFrac = constrain(speedFrac, 0.0f, 1.0f);
  speedFrac = pow(speedFrac, POT_SPEED_CURVE);  // >1 sharpens the ramp for a more dramatic speed change

  // Further from center -> faster, directly in steps/sec (independent of maxSteps/calibration).
  float stepsPerSec = POT_MIN_STEPS_PER_SEC + speedFrac * (POT_MAX_STEPS_PER_SEC - POT_MIN_STEPS_PER_SEC);
  unsigned long stepIntervalMicros = (unsigned long)(1000000.0f / stepsPerSec);

  bool forward = disp < 0.0f;
  if (!DEBUG_SKIP_CALIBRATION) {
    if (forward && currentSteps >= maxSteps) return;
    if (!forward && currentSteps <= minSteps) return;
  }

  unsigned long now = micros();
  if (now - lastStepMicros >= stepIntervalMicros) {
    stepOnce(forward);
    currentSteps += forward ? 1 : -1;
    lastStepMicros = now;
  }
}

// ---------- Setup ----------
void setup() {



  Serial.begin(115200);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ledPin, ledcFreq, ledcResolution);
  ledcAttach(dimmerPin, dimmerLedcFreq, ledcResolution);
#else
  ledcSetup(ledChannel, ledcFreq, ledcResolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcSetup(dimmerChannel, dimmerLedcFreq, ledcResolution);
  ledcAttachPin(dimmerPin, dimmerChannel);
#endif

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);
  digitalWrite(EN_PIN, LOW);

  driverSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  driver.begin();
  Serial.print("Driver version: ");
  Serial.println(driver.version());
  driver.toff(4);
  driver.rms_current(1000);  //800 set to your motor's rated current (mA)
  driver.microsteps(8);
  driver.pwm_autoscale(true);
  driver.TCOOLTHRS(0xFFFFF);
  driver.SGTHRS(100);  // controlls sensitivity of the stallguard, the higher the more sensitive, initial value was 60

  // Calibrate BEFORE touching WiFi/OSC, so nothing can be received or processed during it
  if (DEBUG_SKIP_CALIBRATION) {
    Serial.println("DEBUG_SKIP_CALIBRATION: skipping homing/calibration.");
    maxSteps = DEBUG_NOMINAL_MAX_STEPS;
    minSteps = 0;
    currentSteps = maxSteps / 2;
    calibrated = true;
  } else {
    calibrate();
  }

  if (USE_ANALOG_INPUT) {
    Serial.println("Analog input mode: skipping WiFi/OSC setup, using potentiometers.");
  } else {
    // Now connect WiFi and start listening
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(300);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());

    udp.begin(OSC_PORT);
    Serial.print("Listening for OSC on port ");
    Serial.println(OSC_PORT);
  }
}

// ---------- Main loop ----------
void loop() {
  if (USE_ANALOG_INPUT) {
    handleAnalogSlider();
    checkDimmerDebugSerial();
    // Dimmer doesn't need every-loop precision; throttling its analogRead keeps it
    // from stealing loop cycles the slider needs to hit high step rates.
    static unsigned long lastDimmerMs = 0;
    unsigned long nowMs = millis();
    if (dimmerDebugOverride < 0 && nowMs - lastDimmerMs >= 20) {
      lastDimmerMs = nowMs;
      handleAnalogDimmer();
    }
    return;
  }

  // --- read incoming OSC ---
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    OSCMessage msg;
    while (packetSize--) {
      msg.fill(udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch(oscAddressSlider, sliderCallback);
      msg.dispatch(oscAddressVelocidad, velocidadCallback);
      msg.dispatch(oscAddressDimmer, dimmerCallback);
    }
  }

  // --- advance any ongoing move, cubic-eased ---
  if (moving) {
    unsigned long elapsed = millis() - moveStartTime;
    float t = (float)elapsed / (float)moveDurationMs;
    if (t > 1.0f) t = 1.0f;
    float eased = easeInOutCubic(t);
    long desired = moveStartPos + (long)round((moveTargetPos - moveStartPos) * eased);

    if (desired != currentSteps) {
      bool forward = desired > currentSteps;
      stepOnce(forward);
      currentSteps += forward ? 1 : -1;
    }

    if (t >= 1.0f && desired == currentSteps) {
      moving = false;
      Serial.println("Move complete.");
    }
  }
}
