#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class ServoController {
  private:
    Adafruit_PWMServoDriver &pwm;
    int pin;
    int minPulse;
    int maxPulse;

  public:
    ServoController(Adafruit_PWMServoDriver &pwm, int pin, int minPulse, int maxPulse) : pwm(pwm), pin(pin), minPulse(minPulse), maxPulse(maxPulse) {}

    void setPosition(int angle) {
      int pulseLength = map(angle, 0, 180, minPulse, maxPulse);
      pwm.writeMicroseconds(pin, pulseLength);
    }
};

class QuadrupedRobot {
  private:
    ServoController LF0K;
    ServoController LF0H;
    ServoController RF0K;
    ServoController RF0H;
    ServoController LB0K;
    ServoController LB0H;
    ServoController RB0K;
    ServoController RB0H;

  public:
    QuadrupedRobot(Adafruit_PWMServoDriver &pwm) : LF0K(pwm, 12, 2300, 500), LF0H(pwm, 13, 2300, 500), RF0K(pwm, 9, 500, 2300), RF0H(pwm, 8, 500, 2300),
                       LB0K(pwm, 0, 2300, 500), LB0H(pwm, 1, 2300, 500), RB0K(pwm, 2, 500, 2300), RB0H(pwm, 3, 500, 2300) {}

    void moveLegs(int kneeAngle, int hipAngle) {
      LF0K.setPosition(kneeAngle);
      LF0H.setPosition(hipAngle);
      RF0K.setPosition(kneeAngle);
      RF0H.setPosition(hipAngle);
      LB0K.setPosition(kneeAngle);
      LB0H.setPosition(hipAngle);
      RB0K.setPosition(kneeAngle);
      RB0H.setPosition(hipAngle);
    }

    void moveAllKnee(int angle) {
      LF0K.setPosition(angle);
      RF0K.setPosition(angle);
      LB0K.setPosition(angle);
      RB0K.setPosition(angle);
    }

    void moveAllHip(int angle) {
      LF0H.setPosition(angle);
      RF0H.setPosition(angle);
      LB0H.setPosition(angle);
      RB0H.setPosition(angle);
    }

};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

