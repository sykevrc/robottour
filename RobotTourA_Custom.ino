#include "RoboClaw.h"
#include <math.h>
#include <cppQueue.h>
#include <SoftwareSerial.h>

// Create the serial communications object
SoftwareSerial serial = SoftwareSerial(10, 11);

// Create the RoboClaw object
RoboClaw roboclaw(&serial, 10000);
#define address 0x80
// Robot tuning constants
const float TICKS_PER_METER = 3280.0;
const float WHEEL_BASE = 0.18;
const float MAX_LINEAR_SPEED = 2500;
const float turn_speed = 1200;

int32_t left_ticks = 0;
int32_t right_ticks = 0;

// Motion command types
#define MAX_COMMANDS 10

enum class MotionType { FORWARD,
                        TURN };

struct MotionCommand {
  MotionType type;
  float value;  // meters for forward, radians for turn
};

class MotionQueue {
private:
  MotionCommand commands[MAX_COMMANDS];
  int head = 0;
  int tail = 0;
  int count = 0;

public:
  bool push(MotionCommand cmd) {
    if (count >= MAX_COMMANDS) return false;
    commands[tail] = cmd;
    tail = (tail + 1) % MAX_COMMANDS;
    count++;
    return true;
  }

  bool pop(MotionCommand &cmd) {
    if (count == 0) return false;
    cmd = commands[head];
    head = (head + 1) % MAX_COMMANDS;
    count--;
    return true;
  }

  bool isEmpty() const {
    return count == 0;
  }

  void clear() {
    head = 0;
    tail = 0;
    count = 0;
  }
};


// Command queue
MotionQueue commandQueue;



// Add commands
void moveForward(float meters) {
  commandQueue.push({ MotionType::FORWARD, meters });
}
void turn(float degrees) {
  float radians = degrees * M_PI / 180.0;
  commandQueue.push({ MotionType::TURN, radians });
}


// Ramping helper
float rampVelocity(float current, float target) {
  float diff = target - current;
  float step = 10;
  if (fabs(diff) < step) return target;
  return current + step * (diff > 0 ? 1 : -1);
}
float slowVelocity(float current, float target) {
  float diff = target - current;
  float step = 10;
  if (fabs(diff) < step) return target;
  return current - step * (diff > 0 ? 1 : -1);
}
// Execute all queued commands
void runMotionQueue() {
  float lin_speed = 0.0;
  float ang_speed = 0.0;
  MotionCommand cmd;
  while (commandQueue.pop(cmd)) {

    float traveled = 0.0;
    float angle_turned = 0.0;

    MotionCommand next_cmd;
    bool next_valid = commandQueue.pop(next_cmd);

    while (true) {

      left_ticks = roboclaw.ReadEncM1(address);
      right_ticks = roboclaw.ReadEncM2(address);

      if (cmd.type == MotionType::FORWARD && next_valid) {
        if (next_cmd.type == MotionType::TURN) {
          if (fabs(traveled) <= fabs(cmd.value)) {
            float dist_remaining = cmd.value - traveled;
            float target_speed = (dist_remaining > 0) ? MAX_LINEAR_SPEED : -MAX_LINEAR_SPEED;
            lin_speed = rampVelocity(lin_speed, target_speed);
            traveled = (left_ticks + right_ticks) / (2 * TICKS_PER_METER);
          }else{
            ang_speed = slowVelocity(lin_speed, turn_speed);
            angle_turned = (next_cmd.value>0) ? (left_ticks-right_ticks) : (right_ticks-left_ticks);
            float target_angle = fmod(abs(next_cmd.value),90);
            

          }
        }

      } else {
        float dist_remaining = cmd.value - traveled;
        float target_speed = (dist_remaining > 0) ? MAX_LINEAR_SPEED : -MAX_LINEAR_SPEED;
        lin_speed = rampVelocity(lin_speed, target_speed);
        traveled = (left_ticks + right_ticks) / (2 * TICKS_PER_METER);
      }
      float vL = lin_speed - ang_speed;
      float vR = lin_speed + ang_speed;
      roboclaw.SpeedM1(address, limitSpeed(vL * TICKS_PER_METER));
      roboclaw.SpeedM2(address, limitSpeed(vR * TICKS_PER_METER));


    }
  }

  roboclaw.SpeedM1(address, 0);
  roboclaw.SpeedM2(address, 0);
}



int limitSpeed(float val) {
  if (val > 2500) return 2500;
  if (val < -2500) return -2500;
  return static_cast<int>(val);
}

void setup() {
  Serial.begin(115200);
  roboclaw.begin(38400);
  pinMode(2, INPUT_PULLUP);

  roboclaw.ResetEncoders(address);
  delay(100);
  left_ticks = roboclaw.ReadEncM1(address);
  right_ticks = roboclaw.ReadEncM2(address);
  //commands
  moveForward(0.5);
  turn(90);
  turn(90);
  turn(-90);
  turn(-180);
  turn(180);
}

void loop() {
  runMotionQueue();  // executes motion commands with smooth transitions
  roboclaw.SpeedM1(address, 0);
  roboclaw.SpeedM2(address, 0);
  while (true)
    ;  // done
}
