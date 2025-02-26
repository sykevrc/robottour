#include <avr/wdt.h>
#include "Wire.h"
#include <MPU6050_light.h>
//#include <PID_v1_bc.h"

// Constants
#define PWMA 5
#define PWMB 6
#define BIN_1 8
#define AIN_1 7
#define STBY 3

// Encoder pins
const int encoderL = 4;
const int encoderR = 5;
int lspeed, rspeed;

// Encoder tracking
volatile int lcount = 0;
volatile int rcount = 0;
bool rdir = true;
bool ldir = true;

// Robot parameters
const double wheelRadius = 6.3;
const double rev = 672.0;

// Motion variables
double theta = 0;
int prevSpeed = 0;
const int slewRate = 26;

// MPU6050
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(STBY, OUTPUT);

  attachInterrupt(encoderL, upL, RISING);
  attachInterrupt(encoderR, upR, RISING);

  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcOffsets();
}

void loop() {
  static int step = 0;

  mpu.update();
  theta = -mpu.getAngleZ();
  Serial.println(step);
  switch (step) {
    case 0:
      if (move(50, 0)) step++;
      break;
    case 1:
      if (turnto(90)) step++;
      break;
    case 2:
      if (move(50, 90)) step++;
      break;
    case 3:
      if (turnto(180)) step++;
      break;
    case 4:
      if (move(50, 180)) step++;
      break;
    case 5:
      if (turnto(270)) step++;
      break;
    case 6:
      if (move(50, 270)) step++;
      break;
    case 7:
      if (turnto(0)) step++;
      break;
    case 8:
      if (move(50, 360)) step = 9;  // Reset sequence
      break;
  }
}

bool move(int dist, int agle) {
  double convert = 1.5*(dist / (wheelRadius * 3.14159));
  int error = convert * rev - lcount;

  int targetSpeed;
  if (error > 4) {
    targetSpeed = constrain(error, 120, 200);
  } else if (error < -4) {
    targetSpeed = constrain(error, -200, -120);
  } else {
    targetSpeed = 0;
  }

  // Apply slew rate limiting
  if (targetSpeed > prevSpeed) {
    prevSpeed = min(prevSpeed + slewRate, targetSpeed);
  } else if (targetSpeed < prevSpeed) {
    prevSpeed = max(prevSpeed - slewRate, targetSpeed);
  }
  //Serial.println(String(prevSpeed)+ " "+ String(targetSpeed)+ " "+ String(error)+" "+ String(theta));

  if (theta > agle + 1) {
    rspeed = 0.6 * (prevSpeed - theta);  //decimal is agressiveness of correction
    lspeed = prevSpeed;
  } else if (theta < agle - 1) {
    lspeed = 0.6 * (prevSpeed - theta);  //decimal is agressiveness of correction
    rspeed = prevSpeed;
  } else {
    rspeed = prevSpeed;
    lspeed = prevSpeed;
  }
  //Serial.println(String(lspeed)+ " "+ String(rspeed)+ " "+ String(error)+" "+ String(theta));

  driveL(prevSpeed >= 0, lspeed);
  driveR(prevSpeed <= 0, rspeed);
  if (abs(error < 4)) {
    lcount = 0;
    rcount = 0;
    return true;
  }
  return false;
}
bool turnto(int ang) {
  int error = 5.9 * (ang - theta);
  if (ang - theta > 0) {
    int speed = constrain(error, -200, 200);
    driveL(speed <= 0, abs(speed));
    driveR(speed <= 0, abs(speed));

  } else {
    int speed = constrain(error, -200, 200);
    driveL(speed >= 0, -speed);
    driveR(speed >= 0, -speed);
  }
  if (abs(ang - theta) < 1) {
    lcount = 0;
    rcount = 0;
    return true;
  }
  return false;
}



// Motor Control Functions
void driveL(bool direction, int sp) {
  ldir = direction;
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN_1, direction ? HIGH : LOW);
  analogWrite(PWMA, sp);
}

void driveR(bool direction, int sp) {
  rdir = direction;
  digitalWrite(STBY, HIGH);
  digitalWrite(BIN_1, direction ? LOW : HIGH);
  analogWrite(PWMB, sp);
}

// Encoder Interrupts
void upL() {
  lcount += (ldir ? 1 : -1);
}

void upR() {
  rcount += (rdir ? -1 : 1);
}
