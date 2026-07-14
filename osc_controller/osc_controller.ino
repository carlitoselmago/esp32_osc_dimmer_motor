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
const int EN_PIN   = 27;
const int DIR_PIN  = 25;
const int STEP_PIN = 26;
const int RX_PIN   = 32;
const int TX_PIN   = 33;
const int DIAG_PIN = 14;

#define DRIVER_ADDRESS 0b00
#define R_SENSE        0.11f

HardwareSerial driverSerial(1);
TMC2209Stepper driver(&driverSerial, R_SENSE, DRIVER_ADDRESS);

// ---------- Dimmer output ----------
const int ledPin = 2;      // onboard LED, debug/visual feedback
const int dimmerPin = 15;  // D15 -> DEWIN PWM dimmer module
const int ledChannel = 0;
const int dimmerChannel = 1;
const int ledcFreq = 5000;
const int ledcResolution = 8; // 0-255

// ---------- WiFi / OSC ----------
const char* WIFI_SSID = "MANGO";
const char* WIFI_PASS = "remotamente";
const unsigned int OSC_PORT = 9000;
WiFiUDP udp;

// ---------- Motion parameters ----------
const int CAL_STEP_DELAY_US = 1200;  // slow, safe speed used only during calibration
const int STEP_PULSE_US     = 3;     // step pulse width during normal moves
const int BACKOFF_STEPS     = 200;   // steps to back off from each limit after detecting it
const int STALL_DEBOUNCE    = 30;    // consecutive HIGH DIAG reads required to trust a stall

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

float lastVelocidadSec = 3.0; // default move duration until /velocidad is received

// ---------- Easing ----------
float easeInOutCubic(float t) {
  if (t < 0.5f) return 4.0f * t * t * t;
  float f = -2.0f * t + 2.0f;
  return 1.0f - (f * f * f) / 2.0f;
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
  if (digitalRead(DIAG_PIN) == HIGH) {
    streak++;
  } else {
    streak = 0;
  }
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
    stepOnceSlow(false); // toward limit A
  }
  Serial.println("Limit A found, backing off...");
  for (int i = 0; i < BACKOFF_STEPS; i++) stepOnceSlow(true);
  currentSteps = 0;
  minSteps = 0;

  Serial.println("Searching for limit B...");
  long traveled = 0;
  while (!stalledDebounced()) {
    stepOnceSlow(true); // toward limit B
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
  if (!calibrated) return; // safety: ignore until calibration is done
  float norm = msg.getFloat(0);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;

  moveStartPos = currentSteps;
  moveTargetPos = (long)round(norm * maxSteps);
  moveStartTime = millis();
  moveDurationMs = (unsigned long)(lastVelocidadSec * 1000.0f);
  if (moveDurationMs < 50) moveDurationMs = 50; // avoid a zero/near-zero duration
  moving = true;

  Serial.print("New target: ");
  Serial.print(norm);
  Serial.print("  duration(ms): ");
  Serial.println(moveDurationMs);
}

void velocidadCallback(OSCMessage &msg) {
  float v = msg.getFloat(0);
  if (v > 0.0f) {
    lastVelocidadSec = v;
    Serial.print("New duration set (s): ");
    Serial.println(v);
  }
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
  setDimmer(value);
}

// Accepts either 0.0-1.0 (typical OSC float range) or 0-255 (int) and scales accordingly.
void setDimmer(float value) {
  int duty;
  if (value <= 1.0f && value >= 0.0f) {
    duty = (int)(value * 255.0f);
  } else {
    duty = (int)value;
  }
  duty = constrain(duty, 0, 255);

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(ledPin, duty);
    ledcWrite(dimmerPin, duty);
  #else
    ledcWrite(ledChannel, duty);
    ledcWrite(dimmerChannel, duty);
  #endif
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(ledPin, ledcFreq, ledcResolution);
    ledcAttach(dimmerPin, ledcFreq, ledcResolution);
  #else
    ledcSetup(ledChannel, ledcFreq, ledcResolution);
    ledcAttachPin(ledPin, ledChannel);
    ledcSetup(dimmerChannel, ledcFreq, ledcResolution);
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
  driver.rms_current(800); // set to your motor's rated current (mA)
  driver.microsteps(8);
  driver.pwm_autoscale(true);
  driver.TCOOLTHRS(0xFFFFF);
  driver.SGTHRS(60); // tuned from earlier testing, adjust if needed

  // Calibrate BEFORE touching WiFi/OSC, so nothing can be received or processed during it
  calibrate();

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

// ---------- Main loop ----------
void loop() {
  // --- read incoming OSC ---
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    OSCMessage msg;
    while (packetSize--) {
      msg.fill(udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch("/slider", sliderCallback);
      msg.dispatch("/velocidad", velocidadCallback);
      msg.dispatch("/dimmer", dimmerCallback);
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
