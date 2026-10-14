// Camera slider - TEMP version with NO dimmer code
// Copy of osc_controller with everything dimmer-related removed (ZC/gate pins, ISRs, timer,
// dimmer pot, /dimmer2 OSC address, onboard LED mirror, Serial dimmer override), to rule the
// dimmer out while debugging power/ground problems. Slider behavior is unchanged.
//
// - Listens for OSC over UDP: /slider2 (0.0-1.0 target position), /velocidad2 (0-1 -> move duration)
// - Slider moves always use cubic ease-in-out
// - On boot: auto-calibrates both limits via StallGuard, then centers. OSC is ignored until this finishes.
// - Once calibrated, slider position is trusted from step counting. StallGuard is only used during
//   calibration, not monitored during normal moves.
//
// Libraries needed (Library Manager):
//   TMCStepper (Teemuatlut)
//   OSC (CNMAT)

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

// ---------- WiFi / OSC ----------
const char *WIFI_SSID = "MANGO";
const char *WIFI_PASS = "remotamente";
const unsigned int OSC_PORT = 9000;
WiFiUDP udp;

// OSC addresses - edit these if you want to rename them, used below in loop()
char oscAddressSlider[32] = "/slider2";
char oscAddressVelocidad[32] = "/velocidad2";

// ---------- Motion parameters ----------
const int CAL_STEP_DELAY_US = 800;  // slow, safe speed used only during calibration
const int STEP_PULSE_US = 3;        // step pulse width during normal moves
const int BACKOFF_STEPS = 200;      // steps to back off from each limit after detecting it
const int STALL_DEBOUNCE = 20;      // consecutive HIGH DIAG reads required to trust a stall

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
// false = WiFi/OSC control. true = local slider potentiometer, no WiFi/OSC at all.
const bool USE_ANALOG_INPUT = true;

// ---------- DEBUG ----------
// When true, skips homing/calibration entirely (no StallGuard sensing), no travel limits.
const bool DEBUG_SKIP_CALIBRATION = false;
const long DEBUG_NOMINAL_MAX_STEPS = 4000;
// If a stall isn't detected within this many steps, calibration aborts and falls back to free mode.
const long CALIBRATION_TIMEOUT_STEPS = 20000;
// Uncomment to print slider pot readings ~4x/sec and live SG_RESULT/DIAG/streak during calibration.
//#define DEBUG_PRINT_SLIDER_POT

// ---------- Analog input (potentiometer) wiring ----------
const int POT_SLIDER_PIN = 35;  // ADC1, input-only
const float ADC_MAX = 4095.0f;
const float POT_DEADBAND = 0.15f;  // fraction around center (0.5) treated as "stopped"
const float POT_SPEED_CURVE = 1.4f;  // >1 = speed ramps up more sharply away from center
const float POT_MIN_STEPS_PER_SEC = 100.0f;   // slowest jog speed, just past the deadband
const float POT_MAX_STEPS_PER_SEC = 1300.0f;  // fastest jog speed, at full pot deflection
// Disconnected-pot detection: EMA of |raw - prevRaw| above this is treated as a floating input.
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

// ---------- Analog (potentiometer) control ----------
// Slider pot: center = stopped. Turning away from center drives the slider toward
// that side, with distance from center setting speed, directly in steps/sec.
void handleAnalogSlider() {
  static unsigned long lastStepMicros = 0;

  int raw = analogRead(POT_SLIDER_PIN);

  // Disconnected-pot safety: an unplugged pot floats and jitters wildly between samples.
  // Refuse to move if it looks disconnected, rather than trusting noise as a real command.
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
  speedFrac = pow(speedFrac, POT_SPEED_CURVE);

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
  driver.rms_current(1000);  // set to your motor's rated current (mA)
  driver.microsteps(8);
  driver.pwm_autoscale(true);
  driver.TCOOLTHRS(0xFFFFF);
  driver.SGTHRS(60);  // stallguard sensitivity, the higher the more sensitive

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
    Serial.println("Analog input mode: skipping WiFi/OSC setup, using potentiometer.");
  } else {
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
