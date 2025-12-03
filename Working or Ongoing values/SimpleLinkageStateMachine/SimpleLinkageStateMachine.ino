// Simple linkage state machine using ME350 homing and PID-style position control.
// Pin assignments follow the ME350GCFinal reference sketch.

#include <Encoder.h>

//============================================
// PIN DEFINITIONS
//============================================
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_ENA 11
#define MOTOR_IN2 12
#define MOTOR_IN3 13
#define LIMIT_LEFT 8
#define LIMIT_RIGHT 9
#define ON_OFF_SWITCH_PIN 5

#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

//============================================
// HARDWARE OBJECTS
//============================================
Encoder encoder(ENCODER_A, ENCODER_B);

//============================================
// STATE MACHINE
//============================================
enum State {
  CALIBRATE,
  IDLE,
  CHOOSE_TARGET,
  MOVE_TO_TARGET,
  DWELL
};

State state = CALIBRATE;

// Direction helpers
const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

//============================================
// CONFIGURATION
//============================================
// Nominal encoder positions for each lane (update as needed)
long targetPositions[4] = {-75, -340, -600, -1200};
const long WAIT_POSITION = -600;

// Proximity ranges for simple normalization (max, min)
int proxRange[4][2] = {
  {615, 90},
  {635, 120},
  {620, 145},
  {590, 85}
};

const int targetBand = 8;               // Acceptable error band in encoder counts
const unsigned long dwellTime = 600;    // Time to wait at a target (ms)
const float engageThreshold = 0.25;     // Normalized proximity level to engage
const float CALIBRATE_VOLTAGE = -3.5;   // Negative drives toward the left limit

//============================================
// PID CONTROLLER
//============================================
const float KP = 0.02;
const float KI = 0.0025;
const float KD = 0.018;
const float SUPPLY_VOLTAGE = 12.0;
const float FRICTION_COMP_VOLTAGE = 0.25;

const int MIN_VEL_COMP_COUNT = 2;
const long MIN_VEL_COMP_TIME = 10000; // microseconds

float errorIntegral = 0.0;
float motorVelocity = 0.0;
int previousMotorPosition = 0;
unsigned long previousVelCompTime = 0;
unsigned long executionDuration = 0;
unsigned long lastExecutionTime = 0;

//============================================
// RUNTIME STATE
//============================================
long desiredPosition = WAIT_POSITION;
int activeTarget = -1;
unsigned long dwellStart = 0;
bool systemEnabled = false;
bool autoMode = false;

//============================================
// FORWARD DECLARATIONS
//============================================
void handleSerialCommands();
void handleCalibrate();
void handleIdle();
void handleChooseTarget();
void handleMoveToTarget();
void handleDwell();
float readNormalizedProximity(int index);
void runPositionController(long currentPosition);
void updateVelocity(long currentPosition);
void setMotorVoltage(float voltage);
void stopMotor();
void printStatus(long currentPosition);

//============================================
// SETUP
//============================================
void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("Simple Linkage State Machine"));
  Serial.println(F("============================"));
  Serial.println(F("Commands: G=start, S=stop, H=home"));
}

//============================================
// LOOP
//============================================
void loop() {
  unsigned long now = micros();
  executionDuration = now - lastExecutionTime;
  lastExecutionTime = now;

  long motorPosition = encoder.read();
  updateVelocity(motorPosition);

  systemEnabled = digitalRead(ON_OFF_SWITCH_PIN) == HIGH;

  handleSerialCommands();

  switch (state) {
    case CALIBRATE:
      handleCalibrate();
      break;

    case IDLE:
      handleIdle();
      break;

    case CHOOSE_TARGET:
      handleChooseTarget();
      break;

    case MOVE_TO_TARGET:
      handleMoveToTarget();
      break;

    case DWELL:
      handleDwell();
      break;
  }

  // Limit switch check to refresh zero
  if (state != CALIBRATE && digitalRead(LIMIT_LEFT) == LOW) {
    encoder.write(0);
    errorIntegral = 0;
  }

  if (systemEnabled && state != CALIBRATE) {
    runPositionController(motorPosition);
  } else if (!systemEnabled) {
    stopMotor();
    errorIntegral = 0;
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    printStatus(motorPosition);
  }
}

//============================================
// SERIAL COMMANDS
//============================================
void handleSerialCommands() {
  if (Serial.available() == 0) return;

  char cmd = Serial.read();
  if (cmd == 'G' || cmd == 'g') {
    autoMode = true;
    if (state == IDLE) {
      state = CHOOSE_TARGET;
    }
    Serial.println(F("AUTO MODE"));
  } else if (cmd == 'S' || cmd == 's') {
    autoMode = false;
    stopMotor();
    state = IDLE;
    Serial.println(F("STOP"));
  } else if (cmd == 'H' || cmd == 'h') {
    autoMode = false;
    state = CALIBRATE;
    Serial.println(F("HOMING"));
  }
}

//============================================
// STATE HANDLERS
//============================================
void handleCalibrate() {
  if (!systemEnabled) {
    stopMotor();
    return;
  }

  // Drive toward the left limit switch using a constant voltage until the switch is hit
  if (digitalRead(LIMIT_LEFT) == HIGH) {
    setMotorVoltage(CALIBRATE_VOLTAGE);
  } else {
    stopMotor();
    encoder.write(0);
    errorIntegral = 0;
    desiredPosition = WAIT_POSITION;
    activeTarget = -1;
    state = IDLE;
    Serial.println(F("HOME COMPLETE"));
  }
}

void handleIdle() {
  stopMotor();
  desiredPosition = WAIT_POSITION;

  if (autoMode && systemEnabled) {
    state = CHOOSE_TARGET;
  }
}

void handleChooseTarget() {
  if (!autoMode || !systemEnabled) {
    state = IDLE;
    return;
  }

  float bestDist = 1.1;
  int bestLane = -1;
  for (int i = 0; i < 4; i++) {
    float dist = readNormalizedProximity(i);
    if (dist < bestDist) {
      bestDist = dist;
      bestLane = i;
    }
  }

  if (bestLane >= 0 && bestDist < engageThreshold) {
    activeTarget = bestLane;
    desiredPosition = targetPositions[bestLane];
    state = MOVE_TO_TARGET;
    Serial.print(F("TARGET L"));
    Serial.println(bestLane + 1);
  } else {
    activeTarget = -1;
    desiredPosition = WAIT_POSITION;
    state = MOVE_TO_TARGET;
  }
}

void handleMoveToTarget() {
  if (!systemEnabled) {
    stopMotor();
    return;
  }

  long current = encoder.read();
  long error = desiredPosition - current;

  if (abs(error) <= targetBand) {
    dwellStart = millis();
    state = DWELL;
    return;
  }
}

void handleDwell() {
  stopMotor();

  if (!autoMode || !systemEnabled) {
    state = IDLE;
    return;
  }

  if (millis() - dwellStart >= dwellTime) {
    state = CHOOSE_TARGET;
  }
}

//============================================
// HELPERS
//============================================
float readNormalizedProximity(int index) {
  int raw = analogRead(index == 0 ? PROX_SENSOR_1 :
                        index == 1 ? PROX_SENSOR_2 :
                        index == 2 ? PROX_SENSOR_3 : PROX_SENSOR_4);
  int maxVal = proxRange[index][0];
  int minVal = proxRange[index][1];
  float normalized = (raw - minVal) / (float)(maxVal - minVal);
  normalized = constrain(normalized, 0.0, 1.0);
  return normalized;
}

void runPositionController(long currentPosition) {
  float positionError = desiredPosition - currentPosition;
  float dt = executionDuration / 1000000.0;
  if (dt <= 0) return;

  errorIntegral += positionError * dt;
  float velocityError = -motorVelocity; // desired velocity = 0

  float desiredVoltage = KP * positionError + KI * errorIntegral + KD * velocityError;

  if (desiredVoltage > 0) {
    desiredVoltage += FRICTION_COMP_VOLTAGE;
  } else if (desiredVoltage < 0) {
    desiredVoltage -= FRICTION_COMP_VOLTAGE;
  }

  if (abs(desiredVoltage) > SUPPLY_VOLTAGE) {
    desiredVoltage = constrain(desiredVoltage, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
    errorIntegral -= positionError * dt; // anti-windup
  }

  setMotorVoltage(desiredVoltage);
}

void updateVelocity(long currentPosition) {
  unsigned long now = micros();
  if (abs(currentPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT ||
      (now - previousVelCompTime) > MIN_VEL_COMP_TIME) {
    motorVelocity = (currentPosition - previousMotorPosition) * 1000000.0 /
                    (now - previousVelCompTime);
    previousMotorPosition = currentPosition;
    previousVelCompTime = now;
  }
}

void setMotorVoltage(float voltage) {
  float clipped = constrain(voltage, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
  int pwm = (int)(fabs(clipped) * 255.0 / SUPPLY_VOLTAGE);
  pwm = constrain(pwm, 0, 255);

  analogWrite(MOTOR_ENA, pwm);

  if (clipped >= 0) {
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
  }
}

void stopMotor() {
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
}

void printStatus(long currentPosition) {
  Serial.print(F("State: "));
  Serial.print(state);
  Serial.print(F(" | Pos: "));
  Serial.print(currentPosition);
  Serial.print(F(" -> "));
  Serial.print(desiredPosition);
  Serial.print(F(" | Target: "));
  Serial.println(activeTarget + 1);
}
