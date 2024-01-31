#include <Arduino.h>

#include <Servo.h>

#define MOVEMENT_PIN A0
#define SERVO_PIN 5
#define LED_PIN 6
#define BUZZER_PIN 2

#define MIN_SERVO_ANGLE 75
#define MAX_SERVO_ANGLE 180
#define START_SERVO_ANGLE ((MAX_SERVO_ANGLE + MIN_SERVO_ANGLE) / 2)

#define AUTO_ON_AFTER_WAIT true

class MovementBuzzer {
public:
  void Trigger(long currentTime) {
    // Don't trigger too often.
    if (currentTime - buzzerOnTime < 500) {
      return;
    }

    buzzerOnTime = currentTime;
    buzzerOn = true;
  }
  void Tick(long currentTime) {
    if (!buzzerOn) {
      digitalWrite(BUZZER_PIN, LOW);

      return;
    }

    digitalWrite(BUZZER_PIN, HIGH);

    if (currentTime - buzzerOnTime > 50) {
      buzzerOn = false;
    }
  }

private:
  long buzzerOnTime;
  bool buzzerOn = false;
};

class Movement {
public:
  bool IsDown(long currentTime) {
    const auto isDown = analogRead(MOVEMENT_PIN) < 200;
    if (isDown) {
      lastDown = currentTime;
    }

    return isDown;
  }

  long lastDown = 0;
};

Movement movement;

class Motor {
public:
  void Init() {
    servo.attach(SERVO_PIN);
    servo.write(START_SERVO_ANGLE);
  }

  void Tick(long currentTime) {

    SetState(currentTime);

    if (currentTime - timeOutStart < 500) {
      return;
    }

    isTimedOut = false;

    if (!IsOn()) {
      return;
    }

    static long lastMove = 0;
    if (currentTime - lastMove < 15) {
      return;
    }

    lastMove = currentTime;

    if (goingUp) {
      angle += angleIncrement;
    } else {
      angle -= angleIncrement;
    }

    if (angle > MAX_SERVO_ANGLE) {
      angle = MAX_SERVO_ANGLE;
      goingUp = false;
    } else if (angle < MIN_SERVO_ANGLE) {
      angle = MIN_SERVO_ANGLE;
      goingUp = true;
    }

    servo.write(angle);
  }

  void TurnOn(long currentTime) {
    shouldMove = true;
    turnOnTime = currentTime;
    onDuration = random(25000, 30000);
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
    const auto isDown = movement.IsDown(currentTime);

    if (!IsOn() && isDown) {
      TurnOn(currentTime);
      return;
    }

    if (IsOn() && isDown) {
      //    Timeout(currentTime);
      angleIncrement = 7;
    }

    if (IsOn() && !isDown) {
      angleIncrement = 2;
    }

    // Go to standby after 30s.
    if (currentTime - movement.lastDown > 300000) {
      TurnOff(currentTime);
      return;
    }

    // Turn off after certain time.
    if (IsOn() && (currentTime - turnOnTime > onDuration)) {
      Serial.println("Turning off after " + String(onDuration / 1000) + "s");
      TurnOff(currentTime);
      return;
    }

    // Turn on again after a while.
    if (AUTO_ON_AFTER_WAIT && !IsOn() && (currentTime - turnOffTime > 20000)) {
      Serial.println("Turning on again after 20s");
      TurnOn(currentTime);
    }
  }

  long onDuration = 5000;

  long turnOffTime = 0;
  bool shouldMove = true;

  bool goingUp = true;
  int angle = START_SERVO_ANGLE;

  long timeOutStart = 0;

  int angleIncrement = 1;

  Servo servo;
};

Motor motor;
MovementBuzzer buzzer;

// void HandleMotor(long currentTime) {
//   const bool movementDetected = analogRead(MOVEMENT_PIN) > 500;

//   if (movementDetected) {
//     buzzer.Trigger(currentTime);
//   }

//   if (!motor.isTimedOut && motor.IsOn() &&
//       currentTime - motor.turnOnTime > 1000) {
//     if (movementDetected) {
//       Serial.println("Timing out");
//       motor.Timeout(currentTime);
//     }
//   }

//   if (!movementDetected) {
//     return;
//   }

//   // Only turn on motor when it's off.
//   if (motor.IsOn()) {
//     return;
//   }

//   motor.TurnOn(currentTime);
// }

void setup() {
  Serial.begin(115200);
  motor.Init();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOVEMENT_PIN, INPUT_PULLUP);

  Serial.println("Beginning!");
}

void loop() {
  const auto currentTime = millis();

  // HandleMotor(currentTime);

  motor.Tick(currentTime);

  // Don't use buzzer for now.
  // buzzer.Tick(currentTime);

  digitalWrite(LED_BUILTIN, motor.isTimedOut);
}
