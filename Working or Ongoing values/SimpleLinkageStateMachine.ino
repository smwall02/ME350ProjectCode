// Simple linkage state machine inspired by ME350 linkage code notes.
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
  HOME,
  IDLE,
  CHOOSE_TARGET,
  MOVE_TO_TARGET,
  DWELL
};

State state = HOME;

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

const int targetBand = 8;          // Acceptable error band in encoder counts
const unsigned long dwellTime = 600;  // Time to wait at a target (ms)
const float engageThreshold = 0.25;   // Normalized proximity level to engage

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
void setMotor(int direction, int pwm = 160);
void stopMotor();
void homeToLeft();
void chooseTarget();
void moveToTarget();
void dwellAtTarget();
float readNormalizedProximity(int index);
void printStatus();

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
  // Check on/off switch
  systemEnabled = digitalRead(ON_OFF_SWITCH_PIN) == HIGH;

  // Serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'G' || cmd == 'g') {
      autoMode = true;
      state = CHOOSE_TARGET;
      Serial.println(F("AUTO MODE"));
    } else if (cmd == 'S' || cmd == 's') {
      autoMode = false;
      stopMotor();
      state = IDLE;
      Serial.println(F("STOP"));
    } else if (cmd == 'H' || cmd == 'h') {
      autoMode = false;
      state = HOME;
      Serial.println(F("HOMING"));
    }
  }

  // State machine
  switch (state) {
    case HOME:
      homeToLeft();
      break;

    case IDLE:
      stopMotor();
      desiredPosition = WAIT_POSITION;
      if (autoMode && systemEnabled) {
        state = CHOOSE_TARGET;
      }
      break;

    case CHOOSE_TARGET:
      chooseTarget();
      break;

    case MOVE_TO_TARGET:
      moveToTarget();
      break;

    case DWELL:
      dwellAtTarget();
      break;
  }

  // Periodic status for debugging
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    printStatus();
  }
}

//============================================
// STATE HANDLERS
//============================================
void homeToLeft() {
  if (!systemEnabled) {
    stopMotor();
    return;
  }

  // Drive toward the left limit switch
  if (digitalRead(LIMIT_LEFT) == HIGH) {
    setMotor(BACKWARD, 180);
  } else {
    stopMotor();
    encoder.write(0);
    desiredPosition = WAIT_POSITION;
    activeTarget = -1;
    state = IDLE;
    Serial.println(F("HOME COMPLETE"));
  }
}

void chooseTarget() {
  if (!autoMode || !systemEnabled) {
    state = IDLE;
    return;
  }

  // Select the closest zombie based on normalized proximity
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
    // No strong target; wait at neutral position
    activeTarget = -1;
    desiredPosition = WAIT_POSITION;
    state = MOVE_TO_TARGET;
  }
}

void moveToTarget() {
  if (!systemEnabled) {
    stopMotor();
    return;
  }

  long current = encoder.read();
  long error = desiredPosition - current;

  if (abs(error) <= targetBand) {
    stopMotor();
    dwellStart = millis();
    state = DWELL;
    return;
  }

  int direction = (error > 0) ? FORWARD : BACKWARD;
  int pwm = constrain(abs(error) * 2, 140, 255);
  setMotor(direction, pwm);
}

void dwellAtTarget() {
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

void setMotor(int direction, int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(MOTOR_ENA, pwm);

  if (direction == FORWARD) {
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (direction == BACKWARD) {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
  } else {
    stopMotor();
  }
}

void stopMotor() {
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
}

void printStatus() {
  Serial.print(F("State: "));
  Serial.print(state);
  Serial.print(F(" | Pos: "));
  Serial.print(encoder.read());
  Serial.print(F(" -> "));
  Serial.print(desiredPosition);
  Serial.print(F(" | Target: "));
  Serial.println(activeTarget + 1);
}
