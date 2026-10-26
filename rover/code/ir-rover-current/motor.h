#pragma once

#include "signal_proc.h"
#include "DFRobot_INA219.h"


#define WINDOW_SIZE 20
#define SENSOR_ADDR 0x40

// sensor calibration values
#define CALIBRATION_READING 1005
#define CALIBRATION_CURRENT 100

// maximum current in mA
#define CURRENT_THRESHOLD 600
// time between current samples in ms
#define SAMPLE_MS 2
// how long to wait before activating current limit
#define SPIKE_IGNORE_TIME 120
// if defined, sets the current limit in code rather than using the pot
#define MANUAL_CURRENT_LIMIT 925

// maximum on a 10 bit 5v adc reading a 3.3v trimpot
#define POT_MAX ((2 * 1024) / 3)

class Current {
  public:
  Current() : sensor(&Wire, SENSOR_ADDR) {}

  void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, 0);
    while (!sensor.begin()) {
      Serial.println(F("Current sensor failed to initialize! retrying..."));
      delay(2000);
    }

    sensor.linearCalibrate(CALIBRATION_READING, CALIBRATION_CURRENT);
  }

  void update() {
    now = sensor.getCurrent_mA();
    average.push(now);
    // if (average.value() > maxAvg) {
    //   maxAvg = average.value();
    // }
  }

  bool isElevated(float threshold) {
    update();
    // bool elevated = (now / average.value()) > (1.0 + threshold);
    // digitalWrite(13, elevated);
    // return elevated;
    Serial.print(now); Serial.print(" ");
    Serial.print(average.value()); Serial.print(" ");
    Serial.print(threshold); Serial.print(" ");
    Serial.println();
    return average.value() > threshold;
  }

  void reset() {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      average.push(0);
    }
  }

  private:
  DFRobot_INA219_IIC sensor;
  RollingAvg<WINDOW_SIZE> average;
  float now = 0;
  float maxAvg = 0;
};



// H-bridge configuration
struct HBridge {
  byte dirPin1, dirPin2;
  byte pwmPin;

  void setup() {
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(pwmPin, OUTPUT);
  }

  void stop() {
    digitalWrite(dirPin1, 0);
    digitalWrite(dirPin2, 0);
    digitalWrite(pwmPin, 0);
  }

  void forward(float speed) {
    digitalWrite(dirPin1, 1);
    digitalWrite(dirPin2, 0);
    analogWrite(pwmPin, speed);
  }

  void reverse(float speed) {
    digitalWrite(dirPin1, 0);
    digitalWrite(dirPin2, 1);
    analogWrite(pwmPin, speed);
  }
};



// H-bridge motor controller
class Motor {
  protected:
  struct HBridge hBridge;
  float alpha;

  public:
  // constructor
  Motor(HBridge hBridge, float speed) 
    : alpha(speed), hBridge(hBridge) {}

  void setup() {
    hBridge.setup();
    setSpeed(0);
  }

  // set the motor speed
  // speed is scaled by the motor's maximum speed
  void setSpeed(float speed) {
    float s = alpha * speed; // scale
    if (speed == 0) {
      hBridge.stop();
    } else if (speed > 0) {
      hBridge.forward(255.0 * s);
    } else {
      hBridge.reverse(-255.0 * s);
    }
  }
};



class RampedMotor {
  private:
  Motor motor;
  float currentSpeed;
  float rate;
  unsigned long prevTime;

  public:
  RampedMotor(HBridge hBridge, float speed, float rate)
  : motor(hBridge, speed), 
    rate(rate),
    currentSpeed(0)
  {}

  void setup() {
    motor.setup();
    prevTime = millis();
  }

  void update(float speed) {
    double dt = millis() - prevTime;
    dt *= 0.001;
    prevTime = millis();

    if (speed == currentSpeed) {
      // no need to do anything!
      return;
    } else if (currentSpeed < speed) {
      currentSpeed += rate * dt;
      if (currentSpeed > speed) {
        currentSpeed = speed;
      }
    } else {
      currentSpeed -= rate * dt;
      if (currentSpeed < speed) {
        currentSpeed = speed;
      }
    }

    motor.setSpeed(currentSpeed);
  }

  float getCurrentSpeed() {
    return currentSpeed;
  }
};



class Wheels {
  public:
  Wheels(HBridge hLeft, HBridge hRight, float speed, float ramp, byte trimpotPin, float threshMin, float threshMax)
  : left(hLeft, speed, ramp), 
    right(hRight, speed, ramp),
    trimpotPin(trimpotPin),
    threshMin(threshMin),
    threshRange(threshMax - threshMin)
  {}

  void setup() {
    current.setup();
    left.setup();
    right.setup();
  }

  void readCurrentThreshold() {
    float trimpot = analogRead(trimpotPin);
    #ifdef MANUAL_CURRENT_LIMIT
    currentThreshold = MANUAL_CURRENT_LIMIT;
    #else
    currentThreshold = (threshRange * trimpot / POT_MAX) + threshMin;
    #endif
  }

  bool update(float l, float r) {
    left.update(l);
    right.update(r);
  }

  bool stop() {
    while (left.getCurrentSpeed() != 0 || right.getCurrentSpeed() != 0) { 
      update(0, 0);
    }
    // Serial.print(left.getCurrentSpeed()); Serial.print(" ");
    // Serial.print(right.getCurrentSpeed()); Serial.print(" ");
    // Serial.println();
    current.reset();
  }

  int run(float l, float r, unsigned long ms) {
    readCurrentThreshold();
    unsigned long end = millis() + ms;
    unsigned long ignore = millis() + SPIKE_IGNORE_TIME;
    while (millis() < end) {
      update(l, r);
      if (current.isElevated(currentThreshold) && millis() > ignore) {
        // :c
        stop();
        return -1;
      }
      delay(SAMPLE_MS);
      // Serial.print(left.getCurrentSpeed()); Serial.print(" ");
      // Serial.print(right.getCurrentSpeed()); Serial.print(" ");
      // Serial.println();
    }
    // finished ok!
    return 0;
  }

  private:
  Current current;
  RampedMotor left, right;
  float currentThreshold;
  byte trimpotPin;
  float threshRange, threshMin;
};
