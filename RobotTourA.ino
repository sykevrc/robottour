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
const double rev = 192.0;

// Motion variables
double theta = 0;
int prevSpeed = 0;
const int slewRate = 15;

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

  mpu.update();
  theta = -mpu.getAngleZ();
  move(7,0);
}

bool move(int dist, int agle) {
  Serial.println(prevSpeed);
  double convert = (dist*100/ (wheelRadius * 3.14159));
  double error = 0.98*(convert * rev)-((lcount+rcount)/2);
  int targetSpeed;
  if(abs(error)<10){
    prevSpeed = 0;
    delay(100000);

  }
  if (error > 4) {
    targetSpeed = constrain(error, 25, 200);
  } else if (error < -4) {
    targetSpeed = constrain(error, -200, -25);
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
    lspeed = (prevSpeed - 2.3*abs(theta));  //decimal is agressiveness of correction
    rspeed = prevSpeed;
  } else if (theta < agle - 1) {
    rspeed = (prevSpeed - 2*abs(theta));  //decimal is agressiveness of correction
    lspeed = prevSpeed;
  } else {
    rspeed = prevSpeed;
    lspeed = prevSpeed;
  }
  //Serial.println(String(lspeed)+ " "+ String(rspeed)+ " "+ String(error)+" "+ String(theta));

  driveL(prevSpeed >= 0, lspeed);
  driveR(prevSpeed <= 0, rspeed);
  
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
