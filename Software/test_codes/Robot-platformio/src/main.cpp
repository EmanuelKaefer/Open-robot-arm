#include <TMCStepper.h>
#include <SPI.h>

#define EN_PIN 13
#define STEP_PIN 3
#define DIR_PIN 2
#define CS_PIN 53

#define R_SENSE 0.075f
#define RMS_CURRENT_MA 2100
#define MICROSTEPS 16

// STEP pulse timing (adjust to change speed)
#define STEP_HIGH_US 250
#define STEP_LOW_US 250

TMC5160Stepper driver(CS_PIN, R_SENSE);

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  // TMC5160 EN is active low on most modules.
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  SPI.begin();
  driver.begin();
  driver.toff(5);
  driver.rms_current(RMS_CURRENT_MA);
  driver.microsteps(MICROSTEPS);
  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
}

void pulseStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_LOW_US);
}

void loop() {
  digitalWrite(DIR_PIN, HIGH);
  for (uint16_t i = 0; i < 2000; i++) {
    pulseStep();
  }

  delay(300);

  digitalWrite(DIR_PIN, LOW);
  for (uint16_t i = 0; i < 2000; i++) {
    pulseStep();
  }

  delay(300);
}