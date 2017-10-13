#include <Servo.h>

// Servo pins.
const int LEFT_SERVO_PIN = 2;
const int CENTRAL_SERVO_PIN = 4;
const int RIGHT_SERVO_PIN = 7;

// Servo "zero" angle positions.
const long LEFT_SERVO_ZERO_VALUE = 90;
const long RIGHT_SERVO_ZERO_VALUE = 90;
const long CENTRAL_SERVO_ZERO_VALUE = 90;

// Amplitude of left and right servos.
const long SIDE_SERVOS_FULL_AMPLITUDE = 30;

// Half amplitude of left and right servos. Is used when robot is turning
// left or right while moving forward or backward.
const long SIDE_SERVOS_HALF_AMPLITUDE = 15;

// Amplitude of central servo.
const long CENTRAL_SERVO_AMPLITUDE = 20;

// Periods for different speeds.
const long STEP_PERIOD_VERY_SLOW = 2000;
const long STEP_PERIOD_SLOW = 1500;
const long STEP_PERIOD_FAST = 1000;
const long STEP_PERIOD_VERY_FAST = 500;

long lastMillis;
long globalPhase;
float angleShiftLeftServo;
float angleShiftRightServo;
float angleShiftCentralServo;
long stepPeriod;
long amplitudeLeftServo;
long amplitudeRightServo;
boolean isAttached;
boolean isStopped;

Servo LeftServo;
Servo RightServo;
Servo CentralServo;

void attachServos() {
  if (!isAttached) {
    LeftServo.attach(LEFT_SERVO_PIN);
    RightServo.attach(RIGHT_SERVO_PIN);
    CentralServo.attach(CENTRAL_SERVO_PIN);
    isAttached = true;
  }
}

// In some positions servos can make noise and vibrate.
// To avoid this noise and vibration detach servos when robot is stopped.
void detachServos() {
  if (isAttached) {
    LeftServo.detach();
    RightServo.detach();
    CentralServo.detach();
    isAttached = false;
  }
}

void setup() {
  // To complete moving process need to add bluetooth support
  // (Will be implemented according plan)
 
  attachServos();
  isStopped = true;
  lastMillis = millis();

  angleShiftLeftServo = 0;
  angleShiftRightServo = 0;
  angleShiftCentralServo = 0;
  
  stepPeriod = STEP_PERIOD_FAST;
}

// Gets angle for servo.
// Param amplitude - amplitude of oscillating process,
// param phaseMillis - current duration of oscillating,
// param shiftAndle - phase of oscillating process.
int getAngle(long amplitude, long phaseMillis, float shiftAngle) {
  float alpha = 2 * PI * phaseMillis / stepPeriod + shiftAngle;
  float angle = amplitude * sin(alpha);
  return (int)angle;
}

template<typename T,size_t N>
boolean hasCode(T (&commandCodes)[N], long code) {
  for (int i = 0; i < N; i++) {
    if (commandCodes[i] == code) {
      return true;
    }
  }
  return false;
}

void loop() {
  long millisNow = millis();
  long millisPassed = millisNow - lastMillis;
  if (isStopped) {
    // We should wait for half a second. After that we think that servos are in zero
    // position and we can detach them.
    if (millisPassed >= 500) {
      lastMillis = 0;
      detachServos();
    }

    globalPhase = 0;
  } else {
    lastMillis = millisNow;
    
    globalPhase += millisPassed;
    globalPhase = globalPhase % stepPeriod;
  }
  
  // Declaration of the structure that is used for received and decoded Bluetooth commands.
  decode_results results;
  
  // We can handle Bluetooth command if it is received and decoded successfully.
  if (bluetooth.decode(&results)) {
    
    if (hasCode(BLUETOOTH_COMMAND_FORWARD_CODES, results.value) ||
        hasCode(BLUETOOTH_COMMAND_FORWARD_LEFT_CODES, results.value) ||
        hasCode(BLUETOOTH_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
    
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = PI/2;
        
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(BLUETOOTH_COMMAND_FORWARD_LEFT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(BLUETOOTH_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if(hasCode(BLUETOOTH_COMMAND_BACKWARD_CODES, results.value) ||
              hasCode(BLUETOOTH_COMMAND_BACKWARD_LEFT_CODES, results.value) ||
              hasCode(BLUETOOTH_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {

      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = -PI/2;

      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(BLUETOOTH_COMMAND_BACKWARD_LEFT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(BLUETOOTH_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if (hasCode(BLUETOOTH_COMMAND_TURN_LEFT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = -PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(BLUETOOTH_COMMAND_TURN_RIGHT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(BLUETOOTH_COMMAND_STOP_CODES, results.value)) {
      attachServos();
      isStopped = true;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = 0;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_VERY_SLOW_CODES, results.value)) {
      // globalPhase correction to save servo positions when changing period.
      globalPhase = globalPhase * STEP_PERIOD_VERY_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_SLOW;
    } else if (hasCode(BLUETOOTH_COMMAND_SLOW_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_SLOW;
    } else if (hasCode(BLUETOOTH_COMMAND_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_FAST;
    } else if (hasCode(BLUETOOTH_COMMAND_VERY_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_VERY_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_FAST;
    }
    // Once a code has been decoded, the resume() method must be called to resume receiving codes
    bluetooth.resume();
  }
  
  if (isAttached) {
    LeftServo.write(LEFT_SERVO_ZERO_VALUE + getAngle(amplitudeLeftServo, globalPhase, angleShiftLeftServo));
    RightServo.write(RIGHT_SERVO_ZERO_VALUE + getAngle(amplitudeRightServo, globalPhase, angleShiftRightServo));
    CentralServo.write(CENTRAL_SERVO_ZERO_VALUE + getAngle(CENTRAL_SERVO_AMPLITUDE, globalPhase, angleShiftCentralServo));
  }
}
