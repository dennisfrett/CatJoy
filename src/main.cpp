#include <Arduino.h>

#include <Servo.h>

#define MOVEMENT_PIN A0
#define SERVO_PIN 5
#define LED_PIN 6

#define MIN_SERVO_ANGLE 75
#define MAX_SERVO_ANGLE 180

class Motor {
public:
  void Init() {
    servo.attach(SERVO_PIN);
    servo.write(MIN_SERVO_ANGLE);
  }

  void Tick(long currentTime) {

    SetState(currentTime);

    if (currentTime - timeOutStart < 2000) {
      digitalWrite(LED_PIN, HIGH);

      return;
    }

    isTimedOut = false;

    digitalWrite(LED_PIN, LOW);

    if (!IsOn()) {
      return;
    }

    static long lastMove = 0;
    if (currentTime - lastMove < 300) {
      return;
    }

    lastMove = currentTime;

    if (goingUp) {
      servo.write(MAX_SERVO_ANGLE);
      goingUp = false;
    } else {
      servo.write(MIN_SERVO_ANGLE);
      goingUp = true;
    }
  }

  void TurnOn(long currentTime) {
    shouldMove = true;
    turnOnTime = currentTime;
    onDuration = random(5000, 10000);
  }

  void TurnOff(long currentTime) {
    shouldMove = false;
    turnOffTime = currentTime;
  }

  bool IsOn() const { return shouldMove; }

  void Timeout(long currentTime) {
    isTimedOut = true;
    timeOutStart = currentTime;
  }

  long turnOnTime = 0;

  bool isTimedOut = false;

private:
  void SetState(long currentTime) {
    // Turn off after certain time.
    if (IsOn() && (currentTime - turnOnTime > onDuration)) {
      Serial.println("Turning off after " + String(onDuration / 1000) + "s");
      TurnOff(currentTime);
      return;
    }

    // Turn on again after a while.
    if (!IsOn() && (currentTime - turnOffTime > 20000)) {
      Serial.println("Turning on again after 20s");
      TurnOn(currentTime);
    }
  }

  long onDuration = 5000;

  long turnOffTime = 0;
  bool shouldMove = true;

  bool goingUp = true;

  long timeOutStart = 0;

  Servo servo;
};

Motor motor;

void HandleMotor(long currentTime) {
  static long lastMovementStart = 0;
  static long downCount = 0;

  const bool movementDetected = analogRead(MOVEMENT_PIN) > 500;

  if (!motor.isTimedOut && motor.IsOn() &&
      currentTime - motor.turnOnTime > 1000) {
    if (movementDetected) {
      Serial.println("Timing out");
      motor.Timeout(currentTime);
    }
  }

  if (!movementDetected) {
    return;
  }

  // Only turn on motor when it's off.
  if (motor.IsOn()) {
    return;
  }

  motor.TurnOn(currentTime);
}

void setup() {
  Serial.begin(115200);
  motor.Init();

  pinMode(LED_PIN, OUTPUT);
  pinMode(MOVEMENT_PIN, INPUT_PULLUP);

  Serial.println("Beginning!");
}

void loop() {
  const auto currentTime = millis();

  HandleMotor(currentTime);
  motor.Tick(currentTime);

  digitalWrite(LED_BUILTIN, motor.isTimedOut);
}
