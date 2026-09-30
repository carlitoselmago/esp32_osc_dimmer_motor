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
const int DIM_MIN_PERCENT = 22;//23;  // measured via raw Serial override: stable from ~38% up
const int DIM_MAX_PERCENT = 58;//80;  // measured via raw Serial override: flickers past ~80%
// Hysteresis around the off point: without a gap between "turn off" and "turn back on",
// ADC noise right at DIM_MIN_PERCENT can flip the computed percent between 0 and 1-2 from
// one reading to the next, intermittently firing a marginal late trigger instead of staying
// off - looks like "it turns on" near the dimmest end even though nothing deliberate changed.
const int DIM_OFF_ENTER_PERCENT = DIM_MIN_PERCENT;      // go off once at/below this
const int DIM_OFF_EXIT_PERCENT = DIM_MIN_PERCENT + 4;   // only turn back on once above this

// ---------- Mains timing ----------
const float MAINS_FREQ_HZ = 50.0f;  // set to 60.0f if your mains is 60Hz
const unsigned long HALF_CYCLE_US = (unsigned long)(1000000.0f / MAINS_FREQ_HZ / 2.0f);
const unsigned long TRIGGER_PULSE_US = 100;  // gate trigger pulse width
// Retrigger burst: LED drivers often draw current unevenly, which can let the TRIAC drop
// below its holding current and shut off mid-cycle even after a correct initial fire -
// a common cause of flicker. Sending several pulses through the rest of the half-cycle
// instead of just one relights it almost immediately if that happens.
const int TRIGGER_PULSE_COUNT = 6;
const unsigned long TRIGGER_PULSE_GAP_US = 150;  // gap between pulses in the burst

// ---------- Zero-cross state (touched by ISR) ----------
volatile unsigned long zeroCrossMicros = 0;
volatile bool zeroCrossFlag = false;

const unsigned long ZC_DEBOUNCE_US = 5000;  // reject edges closer together than this (noise, not real crossings)

void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
  if (now - zeroCrossMicros < ZC_DEBOUNCE_US) return;
  zeroCrossMicros = now;
  zeroCrossFlag = true;
}

// ---------- Dimming cycle state ----------
// Kept as float (not rounded to a whole percent) - with only ~100 possible integer values
// spread across the usable DIM_MIN..DIM_MAX window, the dimming visibly steps instead of
// flowing smoothly. The ADC itself has 4096 raw levels to work with; keep that resolution
// all the way to the delay calculation, only rounding for the Serial printout.
float brightnessPercent = 0.0f;  // 0-100, current target level
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
  for (int i = 0; i < TRIGGER_PULSE_COUNT; i++) {
    digitalWrite(PSM_PIN, HIGH);
    delayMicroseconds(TRIGGER_PULSE_US);
    digitalWrite(PSM_PIN, LOW);
    if (i < TRIGGER_PULSE_COUNT - 1) delayMicroseconds(TRIGGER_PULSE_GAP_US);
  }
}

// ---------- Manual override over Serial ----------
// Type a number 0-100 + Enter to hold that EXACT level, computed with the raw/unclamped
// mapping (ignores DIM_MIN_PERCENT/DIM_MAX_PERCENT entirely) - use this to characterize
// the true delay->behavior curve independent of whatever clamp values are set, since
// flicker tied to a fixed real delay (not to the pot's %) won't move just by changing the
// clamps. Type "pot" or "auto" to hand control back to the normal mode.
int serialOverridePercent = -1;  // -1 = no override

void checkSerialOverride() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("pot") || line.equalsIgnoreCase("auto")) {
    serialOverridePercent = -1;
    Serial.println("Override OFF: back to normal control.");
    return;
  }

  int percent = constrain(line.toInt(), 0, 100);
  serialOverridePercent = percent;
  Serial.print("Override ON - holding RAW (unclamped) level: ");
  Serial.print(percent);
  Serial.println("%");
}

// Raw, unclamped mapping - ignores DIM_MIN_PERCENT/DIM_MAX_PERCENT. Used by the Serial
// override above, for testing independent of whatever clamp values happen to be set.
unsigned long computeRawDelayUs(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  float frac = 1.0f - (percent / 100.0f);
  return (unsigned long)(frac * HALF_CYCLE_US);
}

// Hysteresis: stay off until percent rises past DIM_OFF_EXIT_PERCENT, stay on until it
// falls to/below DIM_OFF_ENTER_PERCENT. See the constants above for why this exists.
// percent >= DIM_MAX_PERCENT -> holds at the delay measured at DIM_MAX_PERCENT itself, NOT
// literal 0 - jumping all the way to an immediate trigger revisits the unreliable near-full-
// conduction zone we're specifically trying to avoid. In between, interpolated between the
// delay actually measured at DIM_MIN_PERCENT and DIM_MAX_PERCENT (not the full theoretical
// 0..HALF_CYCLE_US range), so every delay we ever ask for stays inside the validated-safe window.
unsigned long computeDelayUs(float percent) {
  static bool dimmerOff = true;
  if (dimmerOff) {
    if (percent > DIM_OFF_EXIT_PERCENT) dimmerOff = false;
  } else {
    if (percent <= DIM_OFF_ENTER_PERCENT) dimmerOff = true;
  }
  if (dimmerOff) return HALF_CYCLE_US + 1000;

  unsigned long delayAtMin = computeRawDelayUs((float)DIM_MIN_PERCENT);
  unsigned long delayAtMax = computeRawDelayUs((float)DIM_MAX_PERCENT);
  if (percent >= DIM_MAX_PERCENT) return delayAtMax;

  float frac = (percent - DIM_MIN_PERCENT) / (float)(DIM_MAX_PERCENT - DIM_MIN_PERCENT);
  return delayAtMin - (unsigned long)(frac * (delayAtMin - delayAtMax));
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
    unsigned long delayUs = (serialOverridePercent >= 0)
                               ? computeRawDelayUs(serialOverridePercent)
                               : computeDelayUs(brightnessPercent);
    if (elapsed >= delayUs) {
      triggerPulse();
      firedThisHalfCycle = true;
    }
  }

  checkSerialOverride();

  if (serialOverridePercent >= 0) {
    // Override active: pot/auto-sweep logic below is skipped entirely so it can't
    // clobber what's actually driving the output right now.
  } else if (USE_POT_CONTROL) {
    // Live pot control: keep full ADC resolution for the actual control value, only round
    // for the Serial printout (which is throttled separately so it doesn't spam on noise).
    int raw = analogRead(POT_PIN);
    float potPercentF = constrain((raw / ADC_MAX) * 100.0f, 0.0f, 100.0f);
    brightnessPercent = potPercentF;

    int potPercentRounded = (int)(potPercentF + 0.5f);
    static int lastPrintedPercent = -1;
    static unsigned long lastPotPrintMs = 0;
    if (potPercentRounded != lastPrintedPercent && millis() - lastPotPrintMs >= 150) {
      lastPotPrintMs = millis();
      lastPrintedPercent = potPercentRounded;
      Serial.print("Expected brightness level: ");
      Serial.print(potPercentRounded);
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
