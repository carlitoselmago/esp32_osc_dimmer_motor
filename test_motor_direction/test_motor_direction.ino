// Spins one direction continuously, reverses when StallGuard detects resistance.
// Requires the TMCStepper library (Teemuatlut) - install via Library Manager.
//
// Wiring assumed:
//   EN=27, DIR=25, STEP=26
//   UART: D33(TX) -> 1k resistor -> PDN, D32(RX) -> same PDN
//   DIAG -> D14 (with 4.7k pull-up to 3.3V)

#include <TMCStepper.h>
#include <HardwareSerial.h>

const int EN_PIN   = 27;
const int DIR_PIN  = 25;
const int STEP_PIN = 26;
const int RX_PIN   = 32; // D32
const int TX_PIN   = 33; // D33
const int DIAG_PIN = 14; // D14

#define DRIVER_ADDRESS 0b00      // MS1/MS2 both floating on your board
#define R_SENSE        0.11f     // typical for BTT TMC2209 stepsticks, check yours

HardwareSerial driverSerial(1);
TMC2209Stepper driver(&driverSerial, R_SENSE, DRIVER_ADDRESS);

const int STEP_DELAY_US = 800; // same speed as before, tune later
unsigned long lastPrint = 0;

bool clockwise = true;
unsigned long lastFlip = 0;
const unsigned long FLIP_COOLDOWN_MS = 2000; // longer pause after reversing before trusting DIAG again
int stallStreak = 0;
const int STALL_DEBOUNCE = 30; // consecutive HIGH reads required before treating it as a real stall

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);
  digitalWrite(EN_PIN, LOW);

  driverSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.print("Driver version: ");
  Serial.println(driver.version());

  driver.begin();
  driver.toff(4);
  driver.rms_current(800);      // mA - set this to your motor's rated current per phase
  driver.microsteps(8);
  driver.pwm_autoscale(true);

  driver.TCOOLTHRS(0xFFFFF);    // keep StallGuard active across the whole speed range for testing
  driver.SGTHRS(60);            // tuned from your test, adjust further if needed

  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  Serial.println("UART connected. Reading SG_RESULT...");
}

void loop() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);

  bool stalledNow = digitalRead(DIAG_PIN) == HIGH;
  bool cooldownOver = millis() - lastFlip > FLIP_COOLDOWN_MS;

  if (stalledNow && cooldownOver) {
    stallStreak++;
  } else {
    stallStreak = 0;
  }

  if (stallStreak >= STALL_DEBOUNCE) {
    clockwise = !clockwise;
    digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
    lastFlip = millis();
    stallStreak = 0;
    Serial.print("Stall detected, reversing. Now going: ");
    Serial.println(clockwise ? "CW" : "CCW");
  }

  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("DIR: ");
    Serial.print(clockwise ? "CW " : "CCW");
    Serial.print("   SG_RESULT: ");
    Serial.print(driver.SG_RESULT());
    Serial.print("   DIAG pin: ");
    Serial.println(digitalRead(DIAG_PIN));
  }
}
