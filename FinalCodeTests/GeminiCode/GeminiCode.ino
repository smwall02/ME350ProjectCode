// ME350 Zombie Defense - Hardcoded Logic
// Optimized for Safety (10V limit) + Hardcoded Calibration Values from Logs

#include <Encoder.h>
#include <EEPROM.h>

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

Encoder encoder(ENCODER_A, ENCODER_B);

//============================================
// EEPROM ADDRESSES
//============================================
const int EEPROM_KP = 1;
const int EEPROM_KI = 5;
const int EEPROM_KD = 9;
const int EEPROM_FRICTION_LEFT = 13;
const int EEPROM_FRICTION_RIGHT = 17;
const int EEPROM_VOLTAGE = 25;
const int EEPROM_LANES_BASE = 29;
const int EEPROM_PROX_BASE = 50;

//============================================
// STATE MACHINE
//============================================
enum State {
  IDLE = 0,
  CHOOSE_TARGET = 1,
  MOVE_TO_TARGET = 2,
  DWELL = 3
};
State state = IDLE;

// Direction constants
const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

//============================================
// TARGET POSITIONS & LOGIC
//============================================
// HARDCODED FROM LOGS
long targetPositions[4] = {-101, -357, -605, -1253};
const long WAIT_POSITION = -605;
const int TARGET_BAND = 15;

// Hysteresis prevents "jitter" between two zombies at similar distances
const float TARGET_HYSTERESIS = 5.0; // The new target must be 5% closer to danger to switch

//============================================
// SENSOR CALIBRATION
// [0] = 100% (Safe/Start/Max), [1] = 0% (Danger/End/Min)
// HARDCODED FROM LOGS:
// L1[618,78] L2[650,110] L3[636,139] L4[600,70]
//============================================
int ProxRange[4][2] = {
  {618, 78},   // Lane 1
  {650, 110},  // Lane 2
  {636, 139},  // Lane 3
  {600, 70}    // Lane 4
};

//============================================
// PROXIMITY SENSORS
//============================================
struct ProxSensor {
  float currVal;
  float prevVal;
  unsigned long prevTime;
  int pin;
  int direction;
};

ProxSensor sensors[4];
float zombiePosition[4]; // 0-100 scale (0 = DANGER/Photosensor, 100 = SAFE)

// Optimized Filter Alpha (0.80 is faster than 0.92)
const float ALPHA = 0.80; 

const int STOP_TIMEOUT_MS = 150;
const int NOISE_LIMIT = 8;
const int DIR_CONFIRM_COUNT = 3;
int dirForwardCount[4] = {0, 0, 0, 0};
int dirBackwardCount[4] = {0, 0, 0, 0};

//============================================
// TARGETING VARS
//============================================
int activeTarget = -1;
long desiredPosition = WAIT_POSITION;
long previousPosition = WAIT_POSITION;

unsigned long arrivalTime = 0;
const unsigned long DWELL_TIMEOUT_MS = 1000;
const unsigned long BACKWARD_CONFIRM_MS = 150;
unsigned long backwardStartTime = 0;

// Correction for approach direction
const int APPROACH_CORRECTION = 3;

//============================================
// PID CONTROLLER
//============================================
// HARDCODED FROM LOGS
// PID: 0.0177/0.0000/0.0015
// Friction: 1.550
float KP = 0.0177;
float KI = 0.0000;
float KD = 0.0015;
float FRICTION_LEFT = 1.550;
float FRICTION_RIGHT = 1.550;

// Safety Limit: 10.0V Max
float SUPPLY_VOLTAGE = 10.0; 

float errorIntegral = 0;
float motorVelocity = 0;
int previousMotorPosition = 0;
unsigned long previousVelCompTime = 0;
unsigned long executionDuration = 0;
unsigned long lastExecutionTime = 0;

//============================================
// SYSTEM CONTROL
//============================================
bool autoMode = false;
bool systemEnabled = false;
unsigned long lastPrintTime = 0;

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
  Serial.println(F("ME350 Zombie Defense - Hardcoded F25"));
  Serial.println(F("===================================="));
  
  // OPTIONAL: Uncomment to force load old settings from EEPROM, 
  // but we prefer the hardcoded values above.
  // loadSettings();
  
  // Enforce voltage limit
  if(SUPPLY_VOLTAGE > 10.0) SUPPLY_VOLTAGE = 10.0;
  
  int pins[4] = {PROX_SENSOR_1, PROX_SENSOR_2, PROX_SENSOR_3, PROX_SENSOR_4};
  for (int i = 0; i < 4; i++) {
    sensors[i].pin = pins[i];
    sensors[i].currVal = analogRead(pins[i]);
    sensors[i].prevVal = sensors[i].currVal;
    sensors[i].prevTime = millis();
    sensors[i].direction = STOPPED;
  }
  
  stopMotor();
  printHelp();
  printCurrentSettings();
}

//============================================
// MAIN LOOP
//============================================
void loop() {
  unsigned long nowMicro = micros();
  executionDuration = nowMicro - lastExecutionTime;
  lastExecutionTime = nowMicro;
  
  handleSerial();
  computeMotorVelocity();
  updateSensors();
  calculatePositions();

  // State Machine
  if (autoMode && systemEnabled) {
    switch (state) {
      case IDLE:
        // Should not stay in IDLE during autoMode with this version
        state = CHOOSE_TARGET;
        break;
      case CHOOSE_TARGET:
        chooseTarget();
        break;
      case MOVE_TO_TARGET:
        moveToTarget();
        break;
      case DWELL:
        dwell();
        break;
    }
  }
  
  // Motor Safety Check & Control
  if (systemEnabled && digitalRead(ON_OFF_SWITCH_PIN) == HIGH) {
    runPIDController();
  } else {
    stopMotor();
    errorIntegral = 0;
    if (autoMode) {
      autoMode = false;
      Serial.println(F("Safety Switch Disengaged -> STOP"));
    }
  }
  
  // Logging
  if (autoMode && (millis() - lastPrintTime >= 250)) {
    lastPrintTime = millis();
    printStatus();
  }
}

//============================================
// TARGETING LOGIC
//============================================
void chooseTarget() {
  int bestLane = -1;
  float lowestPosition = 101.0; // Lower is more dangerous (0% = at sensor)
  
  // 1. Find the absolute most dangerous zombie
  for (int i = 0; i < 4; i++) {
    // Only target forward moving zombies or stopped zombies near the end
    if (sensors[i].direction == FORWARD || (sensors[i].direction == STOPPED && zombiePosition[i] < 20)) {
      if (zombiePosition[i] < lowestPosition) {
        lowestPosition = zombiePosition[i];
        bestLane = i;
      }
    }
  }
  
  // 2. Apply Hysteresis
  if (activeTarget != -1 && bestLane != -1 && bestLane != activeTarget) {
    float currentTargetPos = zombiePosition[activeTarget];
    
    // If the new 'best' is not at least X% closer than the current target, stick with current.
    if (lowestPosition > (currentTargetPos - TARGET_HYSTERESIS)) {
        bestLane = activeTarget; 
        lowestPosition = currentTargetPos;
    }
  }

  // 3. Update Targets
  if (bestLane >= 0) {
    if (activeTarget != bestLane) {
        Serial.print(F("NEW TARGET L"));
        Serial.print(bestLane + 1);
        Serial.print(F(" @ "));
        Serial.print(lowestPosition, 0);
        Serial.println(F("%"));
        
        previousPosition = encoder.read();
        activeTarget = bestLane;
    }
    
    desiredPosition = targetPositions[bestLane];
    state = MOVE_TO_TARGET;
    
  } else {
    activeTarget = -1;
    desiredPosition = WAIT_POSITION; // Go to a central holding position
  }
}

void moveToTarget() {
  long currentPos = encoder.read();
  
  // Check if we are within the error band
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    
    // Simple approach correction
    long finalTarget = desiredPosition;
    if (previousPosition > desiredPosition) finalTarget -= APPROACH_CORRECTION;
    else if (previousPosition < desiredPosition) finalTarget += APPROACH_CORRECTION;
    
    desiredPosition = finalTarget;
    
    // Transition to Dwell
    arrivalTime = millis();
    backwardStartTime = 0;
    
    if (activeTarget >= 0) {
      state = DWELL;
    } else {
      state = CHOOSE_TARGET;
    }
  }
}

void dwell() {
  unsigned long now = millis();
  unsigned long dwellTime = now - arrivalTime;
  
  if (activeTarget == -1) {
      state = CHOOSE_TARGET;
      return;
  }

  // HIT DETECTION: Sensor sees zombie moving BACKWARD
  if (sensors[activeTarget].direction == BACKWARD) {
    if (backwardStartTime == 0) backwardStartTime = now;
    
    if (now - backwardStartTime >= BACKWARD_CONFIRM_MS) {
      Serial.print(F("HIT CONFIRMED L"));
      Serial.println(activeTarget + 1);
      
      activeTarget = -1;
      state = CHOOSE_TARGET;
      return;
    }
  } else {
    backwardStartTime = 0;
  }
  
  // Timeout
  if (dwellTime >= DWELL_TIMEOUT_MS) {
    state = CHOOSE_TARGET; // Re-evaluate
    return;
  }
}

//============================================
// PID CONTROLLER
//============================================
void computeMotorVelocity() {
  long currentPos = encoder.read();
  unsigned long currentTime = micros();
  
  // Compute velocity every 10ms
  if ((currentTime - previousVelCompTime) > 10000) {
    motorVelocity = (double)(currentPos - previousMotorPosition) * 1000000.0 / 
                    (currentTime - previousVelCompTime);
    previousMotorPosition = currentPos;
    previousVelCompTime = currentTime;
  }
}

void runPIDController() {
  long currentPos = encoder.read();
  float positionError = desiredPosition - currentPos;
  
  // Deadband
  if (abs(positionError) <= (TARGET_BAND/2) && abs(motorVelocity) < 50) {
    stopMotor();
    errorIntegral = 0;
    return;
  }
  
  // Integral term
  errorIntegral += positionError * (float)executionDuration / 1000000.0;
  errorIntegral = constrain(errorIntegral, -1000, 1000);
  
  // Derivative on MEASUREMENT
  float velocityError = 0 - motorVelocity;
  
  float voltage = (KP * positionError) + (KI * errorIntegral) + (KD * velocityError);
  
  // Friction Feedforward
  if (positionError < -TARGET_BAND) {
    voltage -= FRICTION_LEFT;
  } else if (positionError > TARGET_BAND) {
    voltage += FRICTION_RIGHT;
  }
  
  // Voltage Clamp
  if (abs(voltage) > SUPPLY_VOLTAGE) {
    // Anti-windup
    if (((voltage > 0) && (positionError > 0)) || ((voltage < 0) && (positionError < 0))) {
        errorIntegral -= positionError * (float)executionDuration / 1000000.0;
    }
    voltage = constrain(voltage, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
  }
  
  setMotorVoltage(voltage);
}

void setMotorVoltage(float voltage) {
  int pwm = (int)(abs(voltage) * 255.0 / SUPPLY_VOLTAGE);
  pwm = constrain(pwm, 0, 255);
  
  analogWrite(MOTOR_ENA, pwm);
  
  if (voltage >= 0) {
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

void homeToLeftLimit() {
  Serial.println(F("Homing..."));
  unsigned long startTime = millis();
  
  // 5 Second Timeout for Safety
  while (digitalRead(LIMIT_LEFT) == LOW) {
    setMotorVoltage(2.5); 
    delay(10);
    if (millis() - startTime > 5000) {
        Serial.println(F("HOMING FAILED - TIMEOUT"));
        stopMotor();
        return; 
    }
  }
  
  Serial.println(F("Holding at limit..."));
  unsigned long holdStart = millis();
  while (millis() - holdStart < 500) {
    setMotorVoltage(1.5);
    delay(10);
  }
  
  encoder.write(0);
  errorIntegral = 0;
  stopMotor();
  Serial.println(F("Homed! Pos=0"));
}

//============================================
// SENSOR UPDATES
//============================================
void updateSensors() {
  unsigned long now = millis();
  for (int i = 4; i < 4; i++) { // Filter loop fix in next line
  }
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(sensors[i].pin);
    
    sensors[i].currVal = ALPHA * sensors[i].currVal + (1.0 - ALPHA) * raw;
    
    float delta = sensors[i].currVal - sensors[i].prevVal;
    
    if (abs(delta) < NOISE_LIMIT) {
      if (now - sensors[i].prevTime >= STOP_TIMEOUT_MS) {
        sensors[i].direction = STOPPED;
      }
      dirForwardCount[i] = 0;
      dirBackwardCount[i] = 0;
    } 
    else if (delta < 0) { // Moving FORWARD (Toward 0)
      dirForwardCount[i]++;
      dirBackwardCount[i] = 0;
      if (dirForwardCount[i] >= DIR_CONFIRM_COUNT) {
        sensors[i].direction = FORWARD;
        sensors[i].prevVal = sensors[i].currVal;
        sensors[i].prevTime = now;
      }
    } 
    else { // Moving BACKWARD (Toward 100)
      dirBackwardCount[i]++;
      dirForwardCount[i] = 0;
      if (dirBackwardCount[i] >= DIR_CONFIRM_COUNT) {
        sensors[i].direction = BACKWARD;
        sensors[i].prevVal = sensors[i].currVal;
        sensors[i].prevTime = now;
      }
    }
  }
}

void calculatePositions() {
  for (int i = 0; i < 4; i++) {
    float raw = sensors[i].currVal;
    float minVal = ProxRange[i][1];  // 0%
    float maxVal = ProxRange[i][0];  // 100%
    
    if (maxVal != minVal) {
      zombiePosition[i] = (raw - minVal) / (maxVal - minVal) * 100.0;
    } else {
      zombiePosition[i] = 50.0;
    }
  }
}

//============================================
// SERIAL COMMANDS
//============================================
void handleSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.peek();
  
  if (cmd == 'V' || cmd == 'v') {
    Serial.read();
    float value = Serial.parseFloat();
    if (value > 10.0) value = 10.0; // Safety Cap
    SUPPLY_VOLTAGE = value;
    Serial.print(F("V=")); Serial.println(SUPPLY_VOLTAGE, 1);
    return;
  }
  
  cmd = Serial.read();
  switch (cmd) {
    case 'G': case 'g': 
      // AUTO MODE START - NO CALIBRATION
      Serial.println(F("AUTO START (Hardcoded)"));
      autoMode = true;
      systemEnabled = true;
      homeToLeftLimit();
      state = CHOOSE_TARGET;
      break;
      
    case 'S': case 's': 
      Serial.println(F("STOPPED"));
      autoMode = false;
      systemEnabled = false;
      stopMotor();
      break;
      
    case 'H': case 'h': 
      Serial.println(F("HOMING"));
      autoMode = false;
      homeToLeftLimit();
      break;
      
    case '?':
      printHelp();
      break;
  }
}

void printHelp() {
  Serial.println(F("Commands: G=Go, S=Stop, H=Home"));
}

void printCurrentSettings() {
  Serial.println(F("=== HARDCODED SETTINGS ==="));
  Serial.print(F("PID: ")); Serial.print(KP, 4);
  Serial.print(F("/")); Serial.print(KI, 4);
  Serial.print(F("/")); Serial.println(KD, 4);
  Serial.print(F("Friction: ")); Serial.println(FRICTION_LEFT, 3);
  
  Serial.println(F("Prox [Max(100%), Min(0%)]:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("L")); Serial.print(i+1);
    Serial.print(F(": [")); Serial.print(ProxRange[i][0]);
    Serial.print(F(", ")); Serial.print(ProxRange[i][1]);
    Serial.println(F("]"));
  }
}

void printStatus() {
  Serial.print(F("Tgt:")); Serial.print(activeTarget + 1);
  Serial.print(F(" Pos:")); Serial.print(encoder.read());
  Serial.print(F(" Err:")); Serial.print(desiredPosition - encoder.read());
  Serial.println();
}

void saveSettings() {} // Disabled in hardcoded version
void loadSettings() {} // Disabled in hardcoded version
