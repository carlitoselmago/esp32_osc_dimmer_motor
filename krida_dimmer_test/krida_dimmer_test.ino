// Krida trailing-edge MOSFET AC LED Dimmer - standalone test
// Same idea as robotdyn_dimmer_test, adapted for a trailing-edge MOSFET module: instead of a
// short TRIAC trigger pulse fired late in the half-cycle, the gate is held HIGH from the
// zero-cross for a set time and then dropped. Gate HIGH time = power: 0% off, 100% full.
// Two modes (see USE_POT_CONTROL below): automatic 0->100->0% sweep, or live potentiometer
// control - both print the expected level to Serial. No slider/OSC code here - purely for
// validating the dimmer module and wiring before folding it into the main sketch.
//
// Wiring (ESP32) - same header positions as the RobotDyn test, so only the module changes:
//   Module ZC   -> ESP32 D4   (zero-cross detection input, falls at the crossing)
//   Module GATE -> ESP32 D18  (MOSFET gate drive, HIGH = conducting)
//   Module VCC/GND per the module's own markings. CHECK the supply voltage and logic level:
//   ESP32 pins are 3.3V and not 5V tolerant, so if the module runs on 5V its ZC output
//   needs a level shifter / divider before it reaches D4.
//   Pot wiper  -> ESP32 D15  (ADC2 ch3 - fine here since this sketch never uses WiFi)
//   Pot outer legs -> 3V3 and GND (either way round; just flips the sweep direction)
//   Mains: per the module's own terminal markings (dimmer goes in the live line to the load)

// ---------- Mode ----------
const bool USE_POT_CONTROL = true;  // true = pot sets brightness live, false = auto 0->100->0% sweep

// ---------- Pins ----------
//dimmer pins
const int ZC_PIN = 4; // Zero cross pin
const int GATE_PIN = 18;

//potentiometer
const int POT_PIN = 15;

const float ADC_MAX = 4095.0f;

// ---------- Usable range clamp ----------
// Starting guesses, not measured values - tune per bulb with the Serial override below.
// A trailing-edge MOSFET dimmer is built for LED drivers, so the top end is left fully open;
// the bottom still depends on the driver's minimum on-time, below which it can drop out or
// flicker. Anything at/below DIM_MIN_PERCENT is forced fully off rather than interpolated.
const int DIM_MIN_PERCENT = 5;
const int DIM_MAX_PERCENT = 100;
// Hysteresis around the off point: without a gap between "turn off" and "turn back on",
// ADC noise right at DIM_MIN_PERCENT can flip the computed percent between off and a
// sliver of on-time from one reading to the next, which shows up as flicker near the
// dimmest end even though nothing deliberate changed.
const int DIM_OFF_ENTER_PERCENT = DIM_MIN_PERCENT;      // go off once at/below this
const int DIM_OFF_EXIT_PERCENT = DIM_MIN_PERCENT + 4;   // only turn back on once above this

// ---------- Mains timing ----------
const float MAINS_FREQ_HZ = 50.0f;  // set to 60.0f if your mains is 60Hz
const unsigned long HALF_CYCLE_US = (unsigned long)(1000000.0f / MAINS_FREQ_HZ / 2.0f);
// The manufacturer's demo waits this long after the detected zero-cross edge before raising
// the gate - the detector's edge doesn't land exactly on the true crossing.
const unsigned long ZC_OFFSET_US = 100;
// No zero-cross for this long (3 half-cycles) -> assume the signal is lost and force the
// gate off, so a full-on level can't latch the output on without a mains reference.
const unsigned long ZC_LOST_US = HALF_CYCLE_US * 3;

// ---------- Zero-cross / gate state (touched by ISRs) ----------
volatile unsigned long zeroCrossMicros = 0;
volatile bool zeroCrossFlag = false;
volatile unsigned long gateOnTimeUs = 0;  // how long the gate stays HIGH each half-cycle; 0 = off

const unsigned long ZC_DEBOUNCE_US = 5000;  // reject edges closer together than this (noise, not real crossings)

hw_timer_t *gateTimer = nullptr;

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

// ---------- Dimming cycle state ----------
// Kept as float (not rounded to a whole percent) - with only ~100 possible integer values
// the dimming visibly steps instead of flowing smoothly. The ADC itself has 4096 raw levels
// to work with; keep that resolution all the way to the on-time calculation, only rounding
// for the Serial printout.
float brightnessPercent = 0.0f;  // 0-100, current target level
int cycleDirection = 1;
const int LEVEL_STEP = 10;          // percent per step
const unsigned long LEVEL_HOLD_MS = 1500;  // how long to hold each level before moving to the next
unsigned long lastLevelChangeMs = 0;

// Diagnostic: measured half-cycle period, to confirm ZC wiring/frequency assumption
unsigned long lastZeroCrossForPeriod = 0;
unsigned long lastPeriodPrintMs = 0;
unsigned long lastNoZeroCrossPrintMs = 0;

// ---------- Manual override over Serial ----------
// Type a number 0-100 + Enter to hold that EXACT level, computed with the raw/unclamped
// mapping (ignores DIM_MIN_PERCENT/DIM_MAX_PERCENT entirely) - use this to characterize
// where the bulb actually drops out or flickers, independent of whatever clamp values are
// set. Type "pot" or "auto" to hand control back to the normal mode.
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

// Raw, unclamped mapping - ignores DIM_MIN_PERCENT/DIM_MAX_PERCENT. 100% = the whole
// half-cycle, which is the manufacturer's (level * FREQ) / 255 with level = 255.
unsigned long computeRawOnTimeUs(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  return (unsigned long)((percent / 100.0f) * HALF_CYCLE_US);
}

// Hysteresis: stay off until percent rises past DIM_OFF_EXIT_PERCENT, stay on until it
// falls to/below DIM_OFF_ENTER_PERCENT. See the constants above for why this exists.
// While on, percent is held inside DIM_MIN_PERCENT..DIM_MAX_PERCENT.
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

void setup() {
  Serial.begin(115200);
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

  Serial.println("Krida trailing-edge dimmer test starting...");
  Serial.print("Assumed mains frequency: ");
  Serial.print(MAINS_FREQ_HZ);
  Serial.print(" Hz -> half-cycle: ");
  Serial.print(HALF_CYCLE_US);
  Serial.println(" us");
}

void loop() {
  // Pick up the latest zero-cross event from the ISR
  noInterrupts();
  bool crossed = zeroCrossFlag;
  unsigned long zc = zeroCrossMicros;
  zeroCrossFlag = false;
  interrupts();

  if (crossed) {
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
  }

  // Lost-ZC guard: with no zero-cross the ISR never runs, so nothing would ever drop a gate
  // left HIGH at full power. The ISR raises it again by itself once crossings come back.
  if (micros() - zc > ZC_LOST_US) {
    digitalWrite(GATE_PIN, LOW);
    if (millis() - lastNoZeroCrossPrintMs >= 2000) {
      lastNoZeroCrossPrintMs = millis();
      Serial.println("No zero-cross detected - gate held off. Check ZC wiring / mains / module supply.");
    }
  }

  checkSerialOverride();

  if (serialOverridePercent >= 0) {
    // Override active: pot/auto-sweep logic below is skipped entirely so it can't
    // clobber what's actually driving the output right now.
    gateOnTimeUs = computeRawOnTimeUs(serialOverridePercent);
  } else if (USE_POT_CONTROL) {
    // Live pot control: keep full ADC resolution for the actual control value, only round
    // for the Serial printout (which is throttled separately so it doesn't spam on noise).
    int raw = analogRead(POT_PIN);
    float potPercentF = constrain((raw / ADC_MAX) * 100.0f, 0.0f, 100.0f);
    brightnessPercent = potPercentF;
    gateOnTimeUs = computeOnTimeUs(brightnessPercent);

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
      Serial.print((int)brightnessPercent);
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
    gateOnTimeUs = computeOnTimeUs(brightnessPercent);
  }
}
