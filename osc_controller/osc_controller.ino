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
// Krida trailing-edge MOSFET AC LED dimmer. The module reports every mains zero-cross; we
// raise the gate right after it and drop it again after a set on-time (longer = brighter).
// Same pins and DIM_* constants as krida_dimmer_test - validate the module/bulb there and
// copy the tuned values back here.
// Wiring: module ZC -> D4, module GATE -> D18. ESP32 pins are 3.3V only, so check the
// module's supply voltage / ZC output level before connecting.
const int ZC_PIN = 4;     // zero-cross detection input (falls at the crossing)
const int GATE_PIN = 18;  // MOSFET gate drive, HIGH = conducting

const int ledPin = 2;      // onboard LED, mirrors the dimmer level for visual feedback
const int ledChannel = 0;
const int ledcFreq = 5000;
const int ledcResolution = 8;  // 0-255

// Usable range clamp, in percent of the half-cycle the gate is held HIGH. Starting guesses,
// not measured values - tune per bulb. At/below DIM_MIN_PERCENT the output is forced fully
// off rather than interpolated.
const int DIM_MIN_PERCENT = 5;
const int DIM_MAX_PERCENT = 100;
// Hysteresis around the off point: without a gap between the "turn off" and "turn back on"
// thresholds, ADC noise right at the boundary flickers the output between off and a sliver
// of on-time every loop.
const int DIM_OFF_ENTER_PERCENT = DIM_MIN_PERCENT;     // go off once at/below this
const int DIM_OFF_EXIT_PERCENT = DIM_MIN_PERCENT + 4;  // only turn back on once above this

// Mains timing
const float MAINS_FREQ_HZ = 50.0f;  // set to 60.0f if your mains is 60Hz
const unsigned long HALF_CYCLE_US = (unsigned long)(1000000.0f / MAINS_FREQ_HZ / 2.0f);
// The detector's edge doesn't land exactly on the true crossing, so wait this long after it
// before raising the gate (the manufacturer's demo does the same).
const unsigned long ZC_OFFSET_US = 100;
const unsigned long ZC_DEBOUNCE_US = 5000;  // reject edges closer together than this (noise, not real crossings)
// No zero-cross for 3 half-cycles -> signal lost: force the gate off, so a full-power level
// can't leave the output latched on without a mains reference.
const unsigned long ZC_LOST_MS = (HALF_CYCLE_US * 3) / 1000 + 1;

// Touched by the ISRs below
volatile unsigned long zeroCrossMicros = 0;
volatile bool zeroCrossFlag = false;
volatile unsigned long gateOnTimeUs = 0;  // how long the gate stays HIGH each half-cycle; 0 = off
hw_timer_t *gateTimer = nullptr;

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
bool freeMode = false;  // true = no travel limits enforced (debug skip, or calibration timed out)

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
// Placeholder travel range used when calibration is skipped OR aborts (see
// CALIBRATION_TIMEOUT_STEPS below), since the speed math needs a maxSteps to divide
// by even with no real measured range. Tune if slider feels too fast/slow in that mode.
const long DEBUG_NOMINAL_MAX_STEPS = 4000;
// Safety limit for each homing search direction: if a stall isn't detected within this
// many steps, something is wrong (DIAG/UART not working, mechanical issue, etc) — rather
// than grinding into the end stop forever, calibration aborts and falls back to free mode
// (same as DEBUG_SKIP_CALIBRATION: no travel limits enforced). Set comfortably above the
// real full-travel step count.
const long CALIBRATION_TIMEOUT_STEPS = 20000;
// Uncomment to print slider pot readings (raw/norm/disp/forward/step interval) ~4x/sec,
// and live SG_RESULT/DIAG/streak values during calibration.
#define DEBUG_PRINT_SLIDER_POT
// Uncomment to print dimmer pot readings, final gate on-time and the measured mains
// half-cycle. Separate from the flag above so you can debug stallguard/calibration
// without the dimmer spamming the console.
#define DEBUG_PRINT_DIMMER_POT

// ---------- Analog input (potentiometer) wiring ----------
const int POT_SLIDER_PIN = 35;  // 35 wiper -> GPIO34 (ADC1, input-only)
const int POT_DIMMER_PIN = 34;  // 34 wiper -> GPIO35 (ADC1, input-only)
const float ADC_MAX = 4095.0f;
const float POT_DEADBAND = 0.15f;  // fraction around center (0.5) treated as "stopped"
const float POT_SPEED_CURVE = 1.4f;  // >1 = speed ramps up more sharply away from center
const float POT_MIN_STEPS_PER_SEC = 100.0f;   // slowest jog speed, just past the deadband
const float POT_MAX_STEPS_PER_SEC = 1300.0f;  // fastest jog speed, at full pot deflection
// Disconnected-pot detection (see handleAnalogSlider): EMA of |raw - prevRaw| above this
// is treated as a floating/disconnected input. Tune up if a connected pot false-triggers
// this during fast turns, or down if a disconnected pot isn't being caught reliably.
const float POT_JITTER_THRESHOLD = 150.0f;
const float POT_JITTER_EMA_ALPHA = 0.2f;

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
// Falls back to free mode (no travel limits, same as DEBUG_SKIP_CALIBRATION) if a stall
// isn't found within CALIBRATION_TIMEOUT_STEPS — protects against grinding forever into
// an end stop if DIAG/UART stall detection isn't working for whatever reason.
void enterFreeModeFallback(const char *reason) {
  Serial.print("Calibration aborted: ");
  Serial.println(reason);
  Serial.println("Falling back to free mode (no travel limits).");
  freeMode = true;
  maxSteps = DEBUG_NOMINAL_MAX_STEPS;
  minSteps = 0;
  currentSteps = maxSteps / 2;
  calibrated = true;
}

void calibrate() {
  Serial.println("Calibrating: searching for limit A...");
  long searchedA = 0;
  while (!stalledDebounced()) {
    stepOnceSlow(false);  // toward limit A
    searchedA++;
    if (searchedA >= CALIBRATION_TIMEOUT_STEPS) {
      enterFreeModeFallback("no stall found searching for limit A (check DIAG wiring/UART)");
      return;
    }
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
    if (traveled >= CALIBRATION_TIMEOUT_STEPS) {
      enterFreeModeFallback("no stall found searching for limit B (check DIAG wiring/UART)");
      return;
    }
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

// ---------- Dimmer (Krida trailing-edge MOSFET) ----------
// Schedule the gate to drop `us` microseconds from now. One-shot, re-armed every half-cycle.
void IRAM_ATTR armGateOffTimer(unsigned long us) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerRestart(gateTimer);
  timerAlarm(gateTimer, us, false, 0);
#else
  timerWrite(gateTimer, 0);
  timerAlarmWrite(gateTimer, us, false);
  timerAlarmEnable(gateTimer);
#endif
}

void IRAM_ATTR onGateOff() {
  digitalWrite(GATE_PIN, LOW);
}

void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
  if (now - zeroCrossMicros < ZC_DEBOUNCE_US) return;
  zeroCrossMicros = now;
  zeroCrossFlag = true;

  unsigned long onUs = gateOnTimeUs;
  if (onUs == 0) {
    digitalWrite(GATE_PIN, LOW);
    return;
  }
  delayMicroseconds(ZC_OFFSET_US);
  digitalWrite(GATE_PIN, HIGH);
  // Full power: leave the gate HIGH, no timer. Dropping it at the very end of the half-cycle
  // would only race the next zero-cross for a few microseconds of nothing.
  if (onUs >= HALF_CYCLE_US) return;
  armGateOffTimer(onUs);
}

// Raw, unclamped mapping - ignores DIM_MIN_PERCENT/DIM_MAX_PERCENT. 100% = the whole
// half-cycle, which is the manufacturer's (level * FREQ) / 255 with level = 255.
unsigned long computeRawOnTimeUs(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  return (unsigned long)((percent / 100.0f) * HALF_CYCLE_US);
}

// Hysteresis: stay off until percent rises past DIM_OFF_EXIT_PERCENT, stay on until it
// falls to/below DIM_OFF_ENTER_PERCENT. Prevents ADC noise at the boundary from toggling
// on/off rapidly. While on, percent is held inside DIM_MIN_PERCENT..DIM_MAX_PERCENT.
unsigned long computeOnTimeUs(float percent) {
  static bool dimmerOff = true;
  if (dimmerOff) {
    if (percent > DIM_OFF_EXIT_PERCENT) dimmerOff = false;
  } else {
    if (percent <= DIM_OFF_ENTER_PERCENT) dimmerOff = true;
  }
  if (dimmerOff) return 0;

  return computeRawOnTimeUs(constrain(percent, (float)DIM_MIN_PERCENT, (float)DIM_MAX_PERCENT));
}

// Single place that changes the dimmer output: hands the on-time to the ISR (takes effect
// from the next zero-cross) and mirrors the level on the onboard LED.
void applyDimmerOnTime(unsigned long onUs) {
  gateOnTimeUs = onUs;
  int ledDuty = constrain((int)((onUs * 255UL) / HALF_CYCLE_US), 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ledPin, ledDuty);
#else
  ledcWrite(ledChannel, ledDuty);
#endif
}

// Call every loop(). Keeps the gate off while the zero-cross signal is missing (the ISR
// can't drop a gate it was never called to drop) and reports it once per change, so a bench
// run without mains doesn't spam the console.
void serviceZeroCross() {
  // 32-bit reads are atomic; a flag lost to the ISR race only skews the diagnostic below.
  bool crossed = zeroCrossFlag;
  zeroCrossFlag = false;

  static bool seenCross = false;
  static unsigned long lastCrossMs = 0;
  if (crossed) {
    seenCross = true;
    lastCrossMs = millis();
  }

  static bool lostReported = false;
  bool lost = !seenCross || (millis() - lastCrossMs > ZC_LOST_MS);
  if (lost) {
    digitalWrite(GATE_PIN, LOW);
    if (!lostReported) {
      lostReported = true;
      Serial.println("No zero-cross detected - dimmer gate held off (check ZC wiring / mains / module supply).");
    }
  } else if (lostReported) {
    lostReported = false;
    Serial.println("Zero-cross detected - dimmer active.");
  }

#ifdef DEBUG_PRINT_DIMMER_POT
  // Measured half-cycle every couple of seconds, to confirm ZC wiring and MAINS_FREQ_HZ.
  static unsigned long lastCrossForPeriod = 0;
  static unsigned long lastPeriodPrintMs = 0;
  if (crossed) {
    unsigned long zc = zeroCrossMicros;
    if (lastCrossForPeriod != 0 && millis() - lastPeriodPrintMs >= 2000) {
      lastPeriodPrintMs = millis();
      Serial.print("Measured half-cycle: ");
      Serial.print(zc - lastCrossForPeriod);
      Serial.print(" us (expected ");
      Serial.print(HALF_CYCLE_US);
      Serial.println(" us)");
    }
    lastCrossForPeriod = zc;
  }
#endif
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

  // Raw ADC/OSC values keep full float resolution all the way to the on-time; no stepping
  // to whole percents and no slew limit (a held-high gate has no pulse train to protect).
  unsigned long onUs = computeOnTimeUs(norm * 100.0f);
  applyDimmerOnTime(onUs);

#ifdef DEBUG_PRINT_DIMMER_POT
  static unsigned long lastFinalDutyDebugMs = 0;
  if (millis() - lastFinalDutyDebugMs >= 250) {
    lastFinalDutyDebugMs = millis();
    Serial.print("  -> gate on-time (us): ");
    Serial.println(onUs);
  }
#endif
}

// ---------- Dimmer debug: manual level override over Serial ----------
// Type a number 0-100 + Enter in the Serial Monitor to hold that exact level steady
// (bypassing the pot and the DIM_MIN_PERCENT/DIM_MAX_PERCENT clamp + hysteresis, same as
// krida_dimmer_test), so you can characterize exactly which levels flicker vs. hold cleanly
// on the real hardware. Type "auto" or "pot" to hand control back to the potentiometer.
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

  int percent = constrain(line.toInt(), 0, 100);
  dimmerDebugOverride = percent;
  Serial.print("Dimmer override ON - holding RAW (unclamped) level: ");
  Serial.print(percent);
  Serial.println("%");

  applyDimmerOnTime(computeRawOnTimeUs((float)percent));
}

// ---------- Analog (potentiometer) control ----------
// Dimmer pot: direct proportional, same as an OSC 0.0-1.0 value.
void handleAnalogDimmer() {
  int raw = analogRead(POT_DIMMER_PIN);
  float norm = 1.0f - (raw / ADC_MAX);  // inverted: left = on, right = off

#ifdef DEBUG_PRINT_DIMMER_POT
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

  // Disconnected-pot safety: GPIO35 (ADC1, input-only) has no internal pull resistor, so an
  // unplugged pot floats and picks up noise -- readings jitter wildly between samples, unlike
  // a real connected pot which barely moves sample-to-sample even while being turned. Track
  // that jitter and refuse to move if it looks disconnected, rather than trusting noise as a
  // real "pushed hard to one side" command.
  static int prevRaw = -1;
  static float jitterEma = 0.0f;
  if (prevRaw >= 0) {
    float delta = fabs((float)(raw - prevRaw));
    jitterEma += POT_JITTER_EMA_ALPHA * (delta - jitterEma);
  }
  prevRaw = raw;
  bool potDisconnected = jitterEma > POT_JITTER_THRESHOLD;

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

  if (potDisconnected) {
    static unsigned long lastDisconnectWarnMs = 0;
    if (millis() - lastDisconnectWarnMs >= 1000) {
      lastDisconnectWarnMs = millis();
      Serial.println("Slider pot appears disconnected (noisy ADC reading) - ignoring input.");
    }
    return;
  }

  if (mag < POT_DEADBAND) return;  // centered: stay still

  // remap magnitude from [deadband..1.0] to [0..1] so speed ramps smoothly from the deadband edge
  float speedFrac = (mag - POT_DEADBAND) / (1.0f - POT_DEADBAND);
  speedFrac = constrain(speedFrac, 0.0f, 1.0f);
  speedFrac = pow(speedFrac, POT_SPEED_CURVE);  // >1 sharpens the ramp for a more dramatic speed change

  // Further from center -> faster, directly in steps/sec (independent of maxSteps/calibration).
  float stepsPerSec = POT_MIN_STEPS_PER_SEC + speedFrac * (POT_MAX_STEPS_PER_SEC - POT_MIN_STEPS_PER_SEC);
  unsigned long stepIntervalMicros = (unsigned long)(1000000.0f / stepsPerSec);

  bool forward = disp < 0.0f;
  if (!freeMode) {
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
#else
  ledcSetup(ledChannel, ledcFreq, ledcResolution);
  ledcAttachPin(ledPin, ledChannel);
#endif

  // Dimmer starts off (gateOnTimeUs = 0): the ISR keeps the gate LOW until a level is set
  pinMode(ZC_PIN, INPUT);
  pinMode(GATE_PIN, OUTPUT);
  digitalWrite(GATE_PIN, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  gateTimer = timerBegin(1000000);                // 1 MHz tick -> 1 count = 1 us
  timerAttachInterrupt(gateTimer, &onGateOff);
#else
  gateTimer = timerBegin(0, 80, true);            // 80 MHz / 80 = 1 MHz, counting up
  timerAttachInterrupt(gateTimer, &onGateOff, true);
#endif
  attachInterrupt(digitalPinToInterrupt(ZC_PIN), onZeroCross, FALLING);

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
  driver.SGTHRS(60);  // controlls sensitivity of the stallguard, the higher the more sensitive, initial value was 60

  // Calibrate BEFORE touching WiFi/OSC, so nothing can be received or processed during it
  if (DEBUG_SKIP_CALIBRATION) {
    Serial.println("DEBUG_SKIP_CALIBRATION: skipping homing/calibration.");
    freeMode = true;
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
  serviceZeroCross();

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
