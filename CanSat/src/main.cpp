#include "Arduino.h"
#include "DRV8835.h"
#include <SD.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define P 2
#define I 0.15//.05
#define D 1.25

int state1 = 0;
int state2 = 0;
double target = 0;
Adafruit_BNO055 bno;
double prev_e = 0;
double integral = 0;
uint32_t last_update = 0;

void initMotors() {
    pinMode(17, OUTPUT);
    pinMode(21, OUTPUT);
    pinMode(20, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);
    digitalWrite(20, HIGH);
    digitalWrite(17, HIGH);
    digitalWrite(15, HIGH);
    analogWrite(14, 129);
}

double PID(double error, double dt) {
    double p = P * error;
    double d = D * (error - prev_e) / dt;
    prev_e = error;
    integral += error * dt;
    double i = I * integral;
    return p + i + d;
}

void updateMotors(int percent) {
    // -100 <= percent <= 100.
    // Motor1 balance point = 129.
    // Motor2 balance point = ?.
    if (percent >= 0) {
        state1 = 129 + (1.26 * percent);
    } else {
        state1 = 129 + (1.29 * percent);
    }
    if (state1 > 255) {
        state1 = 255;
    } else if (state1 < 0) {
        state1 = 0;
    }
}

void setup() {
    Serial.begin(9600);
    bno = Adafruit_BNO055();
    bno.begin();
    initMotors();
    delay(100);
    target = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x() - 180;
    last_update = millis();
}

void loop() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    double angle = euler.x() - 180;
    double error = angle - target;
    if (error > 180) {
        error -= 360;
    }
    if (error < -180) {
        error += 360;
    }
    double output = PID(error, (millis() - last_update) / 1000.0);
    last_update = millis();
    updateMotors((int)output);
    Serial.printf("T:%d => angle:%f, error:%f, output:%f, state1:%d\n", millis(), angle, error, output, state1);
    analogWrite(14, state1);
    analogWrite(21, state1);
    delay(50);
    /*updateMotors(0);
    analogWrite(21, state1);
    analogWrite(14, state2);
    */
}