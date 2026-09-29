// RobotDyn (clone) AC Light Dimmer Module - standalone test
// Zero-cross + timed-trigger phase-cut dimming. Two modes (see USE_POT_CONTROL below):
// automatic 0->100->0% sweep, or live potentiometer control - both print the expected
// level to Serial. No slider/OSC code here - purely for validating the dimmer module
// and wiring before folding it into the main sketch.
//
// Wiring (ESP32), all on the same header:
//   Module VCC -> ESP32 3V3
//   Module GND -> ESP32 GND
//   Module Z-C -> ESP32 D4   (zero-cross detection input)
//   Module PSM -> ESP32 D18  (TRIAC trigger output)
//   Pot wiper  -> ESP32 D15  (ADC2 ch3 - fine here since this sketch never uses WiFi)
//   Pot outer legs -> 3V3 and GND (either way round; just flips the sweep direction)
//   Mains: IN = live in, N = neutral (also straight to the bulb's neutral), OUT = live to bulb

// ---------- Mode ----------
const bool USE_POT_CONTROL = true;  // true = pot sets brightness live, false = auto 0->100->0% sweep

// ---------- Pins ----------
const int ZC_PIN = 4;
const int PSM_PIN = 18;
const int POT_PIN = 15;
const float ADC_MAX = 4095.0f;

// ---------- Usable range clamp ----------
// LED drivers are commonly unreliable right at the phase-cut extremes (near-immediate
// trigger close to 100%, near-full-half-cycle delay close to 0%) - several RobotDyn-clone
// users report exactly this "cuts off past some level" symptom, fixed by staying a bit
// inside the theoretical 0-100 range. Tune per bulb; below DIM_MIN_PERCENT is forced fully
// off rather than interpolated, since that low end is usually the least reliable of all.
const int DIM_MIN_PERCENT = 8;
const int DIM_MAX_PERCENT = 92;

// ---------- Mains timing ----------
const float MAINS_FREQ_HZ = 50.0f;  // set to 60.0f if your mains is 60Hz
const unsigned long HALF_CYCLE_US = (unsigned long)(1000000.0f / MAINS_FREQ_HZ / 2.0f);
const unsigned long TRIGGER_PULSE_US = 100;  // gate trigger pulse width

// ---------- Zero-cross state (touched by ISR) ----------
volatile unsigned long zeroCrossMicros = 0;
volatile bool zeroCrossFlag = false;

void IRAM_ATTR onZeroCross() {
  zeroCrossMicros = micros();
  zeroCrossFlag = true;
}

// ---------- Dimming cycle state ----------
int brightnessPercent = 0;  // 0-100, current target level
int cycleDirection = 1;
const int LEVEL_STEP = 10;          // percent per step
const unsigned long LEVEL_HOLD_MS = 1500;  // how long to hold each level before moving to the next
unsigned long lastLevelChangeMs = 0;

unsigned long currentZeroCross = 0;
bool firedThisHalfCycle = false;

// Diagnostic: measured half-cycle period, to confirm ZC wiring/frequency assumption
unsigned long lastZeroCrossForPeriod = 0;
unsigned long lastPeriodPrintMs = 0;

void triggerPulse() {
  digitalWrite(PSM_PIN, HIGH);
  delayMicroseconds(TRIGGER_PULSE_US);
  digitalWrite(PSM_PIN, LOW);
}

// percent <= DIM_MIN_PERCENT -> delay set past the half-cycle so it never fires (fully off)
// percent >= DIM_MAX_PERCENT -> delay 0, fires immediately after zero-cross (fully on)
// In between, remapped onto the full delay range so the whole DIM_MIN..DIM_MAX span still
// covers dim->bright, just without ever asking the driver for the unreliable extremes.
unsigned long computeDelayUs(int percent) {
  if (percent <= DIM_MIN_PERCENT) return HALF_CYCLE_US + 1000;
  if (percent >= DIM_MAX_PERCENT) return 0;
  float frac = 1.0f - ((float)(percent - DIM_MIN_PERCENT) / (float)(DIM_MAX_PERCENT - DIM_MIN_PERCENT));
  return (unsigned long)(frac * HALF_CYCLE_US);
}

void setup() {
  Serial.begin(115200);
  pinMode(ZC_PIN, INPUT);
  pinMode(PSM_PIN, OUTPUT);
  digitalWrite(PSM_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), onZeroCross, RISING);

  Serial.println("RobotDyn AC dimmer test starting...");
  Serial.print("Assumed mains frequency: ");
  Serial.print(MAINS_FREQ_HZ);
  Serial.print(" Hz -> half-cycle: ");
  Serial.print(HALF_CYCLE_US);
  Serial.println(" us");
}

void loop() {
  // Pick up the latest zero-cross event from the ISR
  if (zeroCrossFlag) {
    noInterrupts();
    unsigned long zc = zeroCrossMicros;
    zeroCrossFlag = false;
    interrupts();

    // Diagnostic: print the measured half-cycle period every couple of seconds,
    // so you can confirm ZC is actually wired/toggling and matches MAINS_FREQ_HZ.
    if (lastZeroCrossForPeriod != 0 && millis() - lastPeriodPrintMs >= 2000) {
      lastPeriodPrintMs = millis();
      unsigned long measuredPeriodUs = zc - lastZeroCrossForPeriod;
      Serial.print("Measured half-cycle: ");
      Serial.print(measuredPeriodUs);
      Serial.println(" us (compare to HALF_CYCLE_US above)");
    }
    lastZeroCrossForPeriod = zc;

    currentZeroCross = zc;
    firedThisHalfCycle = false;
  }

  // Fire the trigger once per half-cycle, at the delay matching the current brightness
  if (!firedThisHalfCycle && currentZeroCross != 0) {
    unsigned long elapsed = micros() - currentZeroCross;
    unsigned long delayUs = computeDelayUs(brightnessPercent);
    if (elapsed >= delayUs) {
      triggerPulse();
      firedThisHalfCycle = true;
    }
  }

  if (USE_POT_CONTROL) {
    // Live pot control: read continuously, print only when the level actually changes.
    int raw = analogRead(POT_PIN);
    int potPercent = (int)((raw / ADC_MAX) * 100.0f);
    potPercent = constrain(potPercent, 0, 100);

    static int lastPrintedPercent = -1;
    static unsigned long lastPotPrintMs = 0;
    brightnessPercent = potPercent;
    if (potPercent != lastPrintedPercent && millis() - lastPotPrintMs >= 150) {
      lastPotPrintMs = millis();
      lastPrintedPercent = potPercent;
      Serial.print("Expected brightness level: ");
      Serial.print(potPercent);
      Serial.println("%");
    }
  } else {
    // Auto sweep: cycle brightness 0 -> 100 -> 0, printing the expected level at each step
    if (millis() - lastLevelChangeMs >= LEVEL_HOLD_MS) {
      lastLevelChangeMs = millis();

      Serial.print("Expected brightness level: ");
      Serial.print(brightnessPercent);
      Serial.println("%");

      brightnessPercent += cycleDirection * LEVEL_STEP;
      if (brightnessPercent >= 100) {
        brightnessPercent = 100;
        cycleDirection = -1;
      } else if (brightnessPercent <= 0) {
        brightnessPercent = 0;
        cycleDirection = 1;
      }
    }
  }
}
