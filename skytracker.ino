#include <Stepper.h>

// LEDs
int powerLED = 11;
int statusLED = 10;
int limitLED = 9;
const int BLINK_INTERVAL = 409; // Time for a full on-off cycle
int statusLEDstate = LOW;

// Switches
int trackModeSwitch = 13;
int returnModeSwitch = 12;
int homeDetect = 0;
int limitDetect = 1;

// Motors
const float siderealRPM = 0.00069634577; // 1.0 / (86164.0905 * (1.0/60.0));
const float revRatio = 100.0; // Motor revs per output rev
const int STEPS_PER_REV = 2000;
const int MOTOR_INTERVAL = 1009; // How often to rotate the motor
int MOTOR_SPEED = 1;
int stepsToRotate = 0;
int stepsElapsed = 0;
Stepper steppermotor(STEPS_PER_REV, 4, 6, 5, 7);

// Timers
unsigned long oldTimeLED = 0;
unsigned long oldTimeMotor = 0;
unsigned long startTimeMotor = 0;
unsigned long newTime = 0;
unsigned long timeElapsed = 0;

void setup() {
  pinMode(powerLED, OUTPUT);
  pinMode(statusLED, OUTPUT);
  pinMode(limitLED, OUTPUT);
  pinMode(trackModeSwitch, INPUT_PULLUP);
  pinMode(returnModeSwitch, INPUT_PULLUP);
  pinMode(homeDetect, INPUT_PULLUP);
  pinMode(limitDetect, INPUT_PULLUP);

  digitalWrite(powerLED, HIGH);
}

/* MAIN LOOP
1. Pass control based on switch input
*/
void loop() {
  statusLEDstate = LOW;
  digitalWrite(statusLED, statusLEDstate);

  // Check if limit indicator LED state needs to be changed
  if (
    digitalRead(homeDetect) == LOW ||
    digitalRead(limitDetect) == LOW) {
      digitalWrite(limitLED, HIGH);
  }
  else {
    digitalWrite(limitLED, LOW);
  }

  // Call motor movement methods as dictated by switch
  if (digitalRead(trackModeSwitch) == LOW) {
    track();
  }
  if (digitalRead(returnModeSwitch) == LOW) {
    returnToStart();
  }
}

/* RETURN TRACKER TO START POSITION
Moves tracker back to start position, then holds
1. Rotate stepper motor backwards at fast but safe speed until limit switch is
activated
2. Stop, exit function
*/
void returnToStart() {
  // Hold status LED on
  statusLEDstate = HIGH;
  digitalWrite(statusLED, statusLEDstate);

  // Calculate an appropriate number of steps (# steps in 0.1 seconds)
  stepsToRotate = (0.1 / 60.0) * MOTOR_SPEED * 5 * STEPS_PER_REV;
  steppermotor.setSpeed(MOTOR_SPEED * 5);
  while (
    digitalRead(homeDetect) == HIGH &&
    digitalRead(returnModeSwitch) == LOW) {
      // Move motor backwards
      steppermotor.step(-stepsToRotate);

      // Check if limit indicator LED state needs to be changed
      if (
        digitalRead(homeDetect) == LOW ||
        digitalRead(limitDetect) == LOW) {
          digitalWrite(limitLED, HIGH);
      }
      else {
        digitalWrite(limitLED, LOW);
      }
  }

  return;
}

/* ROTATE TRACKER AT EARTH SIDEREAL RATE
1. Rotate stepper motor forwards at sidereal speed until limit switch is
activated
2. Stop, exit function
*/
void track() {
  steppermotor.setSpeed(MOTOR_SPEED);
  oldTimeLED = millis();
  oldTimeMotor = millis();
  startTimeMotor = millis();

  while (
    digitalRead(limitDetect) == HIGH &&
    digitalRead(trackModeSwitch) == LOW) {
      newTime = millis();

      // Blink status LED
      if (newTime - oldTimeLED >= BLINK_INTERVAL / 2.0) {
        oldTimeLED = newTime;
        statusLEDstate = statusLEDstate == HIGH ? LOW : HIGH;
        digitalWrite(statusLED, statusLEDstate);
      }

      // Move motor forwards
      if (newTime - oldTimeMotor >= MOTOR_INTERVAL) {
        oldTimeMotor = newTime;

        // Calculate how much the motor should rotate based on
        // time and steps elapsed
        timeElapsed = newTime - startTimeMotor;
        stepsToRotate = (STEPS_PER_REV * revRatio * siderealRPM * (1.0/60.0) * (timeElapsed / 1000.0)) - stepsElapsed;
        steppermotor.step(stepsToRotate);
        stepsElapsed += stepsToRotate;
      }

      // Check if limit indicator LED state needs to be changed
      if (
        digitalRead(homeDetect) == LOW ||
        digitalRead(limitDetect) == LOW) {
          digitalWrite(limitLED, HIGH);
      }
      else {
        digitalWrite(limitLED, LOW);
      }
    }

  return;
}
