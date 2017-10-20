#include <Servo.h>

// EN: Servo pins.
// RU: Цифровые выводы контролера, к которым подключены серводвигатели.
const int LEFT_SERVO_PIN = 9;
const int CENTRAL_SERVO_PIN = 10;
const int RIGHT_SERVO_PIN = 8;

// EN: Servo "zero" angle positions.
// RU: Центральное ("нулевое") положение серводвигателей в градусах.
const long LEFT_SERVO_ZERO_VALUE = 90;
const long RIGHT_SERVO_ZERO_VALUE = 90;
const long CENTRAL_SERVO_ZERO_VALUE = 90;

// EN: Amplitude of left and right servos.
// RU: Амплитула левого и правого серводвигателей.
const long SIDE_SERVOS_FULL_AMPLITUDE = 30;
// EN: Half amplitude of left and right servos. Is used when robot is turning
//     left or right while moving forward or backward.
// RU: Уменьшенная амплитула левого и правого серводвигателей. Используется
//     при поворотах совмещённых с движением вперёд или назад.
const long SIDE_SERVOS_HALF_AMPLITUDE = 15;
// EN: Amplitude of central servo.
// RU: Амплитула центрального серводвигателя.
const long CENTRAL_SERVO_AMPLITUDE = 20;

// EN: Periods for different speeds.
// RU: Периоды колебаний для различных скоростей.
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

// commands
int BLUETOOTH_COMMAND_FORWARD = 1;
int BLUETOOTH_COMMAND_FORWARD_LEFT = 2;
int BLUETOOTH_COMMAND_FORWARD_RIGHT = 3;
int BLUETOOTH_COMMAND_BACKWARD = 4;
int BLUETOOTH_COMMAND_BACKWARD_LEFT = 5;
int BLUETOOTH_COMMAND_BACKWARD_RIGHT = 6;
int BLUETOOTH_COMMAND_TURN_LEFT = 7;
int BLUETOOTH_COMMAND_TURN_RIGHT = 8;
int BLUETOOTH_COMMAND_STOP = 9;
int BLUETOOTH_COMMAND_VERY_SLOW = 10;
int BLUETOOTH_COMMAND_SLOW = 11;
int BLUETOOTH_COMMAND_FAST = 12;
int BLUETOOTH_COMMAND_VERY_FAST = 13;

int BLUETOOTH_COMMAND_MIN = 1;
int BLUETOOTH_COMMAND_MAX = 13;

int generateRandomCommand(){
  return random(BLUETOOTH_COMMAND_MIN, BLUETOOTH_COMMAND_MAX);
}

bool isCommand(int expectedCommand, int actualCommand){
  return expectedCommand == actualCommand;
}

void attachServos() {
  if (!isAttached) {
    LeftServo.attach(LEFT_SERVO_PIN);
    RightServo.attach(RIGHT_SERVO_PIN);
    CentralServo.attach(CENTRAL_SERVO_PIN);
    isAttached = true;
  }
}

// EN: In some positions servos can make noise and vibrate.
//     To avoid this noise and vibration detach servos when robot is stopped.
// RU: В некоторых положениях серводвигатели могут вибрировать и шуметь.
//     Чтобы это избежать во время остановок робота, сервы надо отключать.
void detachServos() {
  if (isAttached) {
    LeftServo.detach();
    RightServo.detach();
    CentralServo.detach();
    isAttached = false;
  }
}

void setup() {
  // EN: Start the IR receiver.
  // RU: Начинаем прослушивание ИК-сигналов
 
  attachServos();
  isStopped = false;
  lastMillis = millis();

  angleShiftLeftServo = 0;
  angleShiftRightServo = 0;
  angleShiftCentralServo = 0;
  
  stepPeriod = STEP_PERIOD_FAST;
}

// EN: Gets angle for servo.
//     Param amplitude - amplitude of oscillating process,
//     param phaseMillis - current duration of oscillating,
//     param shiftAndle - phase of oscillating process.
// RU: Получение угла для серводвигателя.
//     Параметр amplitude - амплитуда колебаний,
//     Параметр phaseMillis - текущая продолжительность колебаний,
//     Параметр shiftAndle - фаза колебаний.
int getAngle(long amplitude, long phaseMillis, float shiftAngle) {
  float alpha = 2 * PI * phaseMillis / stepPeriod + shiftAngle;
  float angle = amplitude * sin(alpha);
  return (int)angle;
}

void loop() {
  long millisNow = millis();
  long millisPassed = millisNow - lastMillis;
  if (isStopped) {
    // EN: We should wait for half a second. After that we think that servos are in zero
    //     position and we can detach them.
    // RU: Ждём полсекунды, чтобы серводвигатели вышли в нулевое положение и отключаем их.
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
  
   int command = generateRandomCommand();

   if (isCommand(BLUETOOTH_COMMAND_FORWARD, command) ||
        isCommand(BLUETOOTH_COMMAND_FORWARD_LEFT, command) ||
        isCommand(BLUETOOTH_COMMAND_FORWARD_RIGHT, command)) {
    
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = PI/2;
        
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (isCommand(BLUETOOTH_COMMAND_FORWARD_LEFT, command)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (isCommand(BLUETOOTH_COMMAND_FORWARD_RIGHT, command)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if(isCommand(BLUETOOTH_COMMAND_BACKWARD, command) ||
              isCommand(BLUETOOTH_COMMAND_BACKWARD_LEFT, command) ||
              isCommand(BLUETOOTH_COMMAND_BACKWARD_RIGHT, command)) {

      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = -PI/2;

      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (isCommand(BLUETOOTH_COMMAND_BACKWARD_LEFT, command)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (isCommand(BLUETOOTH_COMMAND_BACKWARD_RIGHT, command)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if (isCommand(BLUETOOTH_COMMAND_TURN_LEFT, command)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = -PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (isCommand(BLUETOOTH_COMMAND_TURN_RIGHT, command)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (isCommand(BLUETOOTH_COMMAND_STOP, command)) {
      attachServos();
      isStopped = true;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = 0;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (isCommand(BLUETOOTH_COMMAND_VERY_SLOW, command)) {
      // EN: globalPhase correction to save servo positions when changing period.
      // RU: Корректировка globalPhase чтобы сохранить положение серв при смене периода колебаний ног.
      globalPhase = globalPhase * STEP_PERIOD_VERY_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_SLOW;
    } else if (isCommand(BLUETOOTH_COMMAND_SLOW, command)) {
      globalPhase = globalPhase * STEP_PERIOD_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_SLOW;
    } else if (isCommand(BLUETOOTH_COMMAND_FAST, command)) {
      globalPhase = globalPhase * STEP_PERIOD_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_FAST;
    } else if (isCommand(BLUETOOTH_COMMAND_VERY_FAST, command)) {
      globalPhase = globalPhase * STEP_PERIOD_VERY_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_FAST;
    }
  
    if (isAttached) {
      LeftServo.write(LEFT_SERVO_ZERO_VALUE + getAngle(amplitudeLeftServo, globalPhase, angleShiftLeftServo));
      RightServo.write(RIGHT_SERVO_ZERO_VALUE + getAngle(amplitudeRightServo, globalPhase, angleShiftRightServo));
      CentralServo.write(CENTRAL_SERVO_ZERO_VALUE + getAngle(CENTRAL_SERVO_AMPLITUDE, globalPhase, angleShiftCentralServo));
    }
}
