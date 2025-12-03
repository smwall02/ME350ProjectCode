// ME350 Zombie Defense - Simple Logic with Auto-Calibration
// Target the forward-moving zombie that is furthest down its lane

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
// TARGET POSITIONS (encoder counts)
//============================================
long targetPositions[4] = {-73, -341, -594, -1200};
const long WAIT_POSITION = -605;
const int TARGET_BAND = 5;

//============================================
// SENSOR CALIBRATION
// [0] = value at 100% (start of lane, near prox sensor - HIGH reading)
// [1] = value at 0% (end of lane, near photosensor - LOW reading)
//============================================
int ProxRange[4][2] = {
  {618, 78},   // Lane 1
  {650, 110},  // Lane 2
  {636, 139},  // Lane 3
  {600, 70}    // Lane 4
};

//============================================
// CALIBRATION SYSTEM
//============================================
bool calibrationActive = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 12000;  // 10 seconds
unsigned long lastCalibrationLogTime = 0;

// Track min/max readings per lane during calibration
int calibrationMin[4] = {1023, 1023, 1023, 1023};  // Lowest reading (at end/0%)
int calibrationMax[4] = {0, 0, 0, 0};              // Highest reading (at start/100%)
int calibrationStart[4] = {0, 0, 0, 0};            // Initial reading
int calibrationRaw[4] = {0, 0, 0, 0};              // Latest raw readings
bool calibrationUpdated[4] = {false, false, false, false};

//============================================
// PROXIMITY SENSORS
//============================================
struct ProxSensor {
  float currVal;
  float prevVal;
  unsigned long prevTime;
  int pin;
  int direction;
  float velocity;           // Rate of position change (%/second)
  float prevPosition;       // For velocity calculation
  unsigned long velTime;    // Time of last velocity calc
};

ProxSensor sensors[4];
float zombiePosition[4];  // 0-100 scale (0 = end of lane, 100 = start)

const float ALPHA = 0.92;
const int STOP_TIMEOUT_MS = 150;
const int NOISE_LIMIT = 10;
const int DIR_CONFIRM_COUNT = 3;
int dirForwardCount[4] = {0, 0, 0, 0};
int dirBackwardCount[4] = {0, 0, 0, 0};

//============================================
// TARGETING
//============================================
int activeTarget = -1;
long desiredPosition = WAIT_POSITION;
long previousPosition = WAIT_POSITION;  // Track where we came from for correction

// Dwell timing
unsigned long arrivalTime = 0;
const unsigned long DWELL_TIME_MS = 250;  // Fixed time to stay at target

// Correction for approach direction (accounts for mechanical slop)
const int APPROACH_CORRECTION = 3;  // Encoder counts to overshoot

//============================================
// PID CONTROLLER
//============================================
float KP = 0.015;
float KI = 0.003;
float KD = 0.020;
float FRICTION_LEFT = 0.25;
float FRICTION_RIGHT = 0.25;
float SUPPLY_VOLTAGE = 12.0;

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
  Serial.println(F("ME350 Zombie Defense - Simple + Calibration"));
  Serial.println(F("============================================"));
  
  loadSettings();
  
  int pins[4] = {PROX_SENSOR_1, PROX_SENSOR_2, PROX_SENSOR_3, PROX_SENSOR_4};
  for (int i = 0; i < 4; i++) {
    sensors[i].pin = pins[i];
    sensors[i].currVal = analogRead(pins[i]);
    sensors[i].prevVal = sensors[i].currVal;
    sensors[i].prevTime = millis();
    sensors[i].direction = STOPPED;
    sensors[i].velocity = 0;
    sensors[i].prevPosition = 50.0;
    sensors[i].velTime = millis();
  }
  
  stopMotor();
  printHelp();
  printCurrentSettings();
}

//============================================
// MAIN LOOP
//============================================
void loop() {
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();
  
  handleSerial();
  computeMotorVelocity();
  updateSensors();
  calculatePositions();
  calculateZombieVelocities();
  
  // Update calibration if active
  if (calibrationActive) {
    updateCalibration();
  }
  
  //--- STATE MACHINE ---
  // CRITICAL: Skip state machine during calibration - mechanism must stay at position 0
  if (calibrationActive) {
    // State machine disabled during calibration
    // PID controller will maintain position 0
    desiredPosition = 0;
  } else if (autoMode && systemEnabled) {
    switch (state) {
      case IDLE:
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
  
  //--- MOTOR CONTROL ---
  if (systemEnabled && digitalRead(ON_OFF_SWITCH_PIN) == HIGH) {
    runPIDController();
  } else {
    stopMotor();
    errorIntegral = 0;
    if (autoMode) {
      autoMode = false;
      Serial.println(F("Switch OFF"));
    }
  }
  
}

//============================================
// CALIBRATION FUNCTIONS
//============================================
void startCalibration() {
  calibrationActive = true;
  calibrationStartTime = millis();
  lastCalibrationLogTime = 0;
  
  // CRITICAL: Move mechanism to encoder home (position 0) and keep it there during calibration
  desiredPosition = 0;
  systemEnabled = true;  // Ensure PID controller is active to maintain position
  state = IDLE;          // Prevent state machine from running during calibration
  
  Serial.println(F("=== CALIBRATION STARTED (10s) ==="));
  Serial.println(F("Mechanism holding at home position (0)..."));
  Serial.println(F("Capturing initial readings as 100% (start)..."));
  
  for (int i = 0; i < 4; i++) {
    // Capture initial RAW reading - this is the 100% position (start of lane)
    calibrationStart[i] = analogRead(sensors[i].pin);
    calibrationMin[i] = 1023;  // Will track down to find minimum (end/0%)
    calibrationMax[i] = calibrationStart[i];
    calibrationUpdated[i] = false;
    
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": Initial = "));
    Serial.print(calibrationStart[i]);
    Serial.println(F(" (100%)"));
  }
  
  Serial.println(F("Move targets to 0% (end) and back!"));
}

void updateCalibration() {
  if (!calibrationActive) return;
  
  unsigned long elapsed = millis() - calibrationStartTime;
  
  if (elapsed >= CALIBRATION_DURATION) {
    applyCalibration();
    calibrationActive = false;
    
    // Now start the targeting state machine
    state = CHOOSE_TARGET;
    
    Serial.print(F("CAL DONE: "));
    for (int i = 0; i < 4; i++) {
      Serial.print(F("L"));
      Serial.print(i + 1);
      Serial.print(F("["));
      Serial.print(ProxRange[i][0]);
      Serial.print(F(","));
      Serial.print(ProxRange[i][1]);
      Serial.print(F("] "));
    }
    Serial.println();
    Serial.println(F("Starting targeting..."));
    return;
  }
  
  // Track min/max for each lane using RAW readings
  for (int i = 0; i < 4; i++) {
    int rawVal = analogRead(sensors[i].pin);
    calibrationRaw[i] = rawVal;
    
    if (rawVal < calibrationMin[i]) {
      calibrationMin[i] = rawVal;  // Minimum = end of lane (0%)
      calibrationUpdated[i] = true;
    }
    if (rawVal > calibrationMax[i]) {
      calibrationMax[i] = rawVal;  // Maximum = start of lane (100%)
      calibrationUpdated[i] = true;
    }
  }
  
  // Log every second
  if (elapsed - lastCalibrationLogTime >= 1000) {
    Serial.print(F("[CAL] t="));
    Serial.print(elapsed / 1000);
    Serial.print(F("s "));
    for (int i = 0; i < 4; i++) {
      Serial.print(F("L"));
      Serial.print(i + 1);
      Serial.print(F("("));
      Serial.print(calibrationRaw[i]);
      Serial.print(F("/"));
      Serial.print(calibrationMin[i]);
      Serial.print(F("-"));
      Serial.print(calibrationMax[i]);
      Serial.print(F(") "));
    }
    Serial.println();
    lastCalibrationLogTime = elapsed;
  }
}

void applyCalibration() {
  Serial.println(F("Applying calibration:"));
  
  for (int i = 0; i < 4; i++) {
    int startVal = max(calibrationStart[i], calibrationMax[i]);  // 100% (start)
    int endVal = calibrationMin[i];  // 0% (end)
    int range = startVal - endVal;
    
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": 100%="));
    Serial.print(startVal);
    Serial.print(F(" 0%="));
    Serial.print(endVal);
    Serial.print(F(" range="));
    Serial.print(range);
    
    if (!calibrationUpdated[i]) {
      Serial.println(F(" [NO MOVEMENT - keeping default]"));
      continue;
    }
    
    if (range > 150 && endVal < startVal) {
      // Add small margins for noise
      int newMax = min(startVal + 2, 1023);
      int newMin = max(endVal - 2, 0);
      int finalRange = newMax - newMin;
      
      if (finalRange > 120) {
        ProxRange[i][0] = newMax;  // 100% (start, high reading)
        ProxRange[i][1] = newMin;  // 0% (end, low reading)
        
        // Save to EEPROM
        EEPROM.put(EEPROM_PROX_BASE + i * 4, newMax);
        EEPROM.put(EEPROM_PROX_BASE + 16 + i * 4, newMin);
        
        Serial.println(F(" [APPLIED & SAVED]"));
      } else {
        Serial.println(F(" [range too small]"));
      }
    } else {
      Serial.println(F(" [invalid - keeping default]"));
    }
  }
}

//============================================
// UPDATE SENSORS
//============================================
void updateSensors() {
  unsigned long now = millis();
  
  for (int i = 0; i < 4; i++) {
    // Low-pass filter
    int raw = analogRead(sensors[i].pin);
    sensors[i].currVal = ALPHA * sensors[i].currVal + (1.0 - ALPHA) * raw;
    
    float delta = sensors[i].currVal - sensors[i].prevVal;
    
    if (abs(delta) < NOISE_LIMIT) {
      // No significant change
      if (now - sensors[i].prevTime >= STOP_TIMEOUT_MS) {
        sensors[i].direction = STOPPED;
      }
      dirForwardCount[i] = 0;
      dirBackwardCount[i] = 0;
    } 
    else if (delta < 0) {
      // Sensor value decreasing = zombie moving toward 0% (FORWARD)
      dirForwardCount[i]++;
      dirBackwardCount[i] = 0;
      if (dirForwardCount[i] >= DIR_CONFIRM_COUNT) {
        sensors[i].direction = FORWARD;
        sensors[i].prevVal = sensors[i].currVal;
        sensors[i].prevTime = now;
      }
    } 
    else {
      // Sensor value increasing = zombie moving toward 100% (BACKWARD)
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

//============================================
// CALCULATE POSITIONS (0-100 scale)
// 0 = end of lane (at photosensor) = DANGER
// 100 = start of lane (at prox sensor) = SAFE
//============================================
void calculatePositions() {
  for (int i = 0; i < 4; i++) {
    float raw = sensors[i].currVal;
    float minVal = ProxRange[i][1];  // Value at 0%
    float maxVal = ProxRange[i][0];  // Value at 100%
    
    if (maxVal != minVal) {
      zombiePosition[i] = (raw - minVal) / (maxVal - minVal) * 100.0;
    } else {
      zombiePosition[i] = 50.0;
    }
    // Don't clamp - allow values outside 0-100 for calibration visibility
  }
}

//============================================
// ZOMBIE VELOCITY CALCULATION
//============================================
void calculateZombieVelocities() {
  unsigned long now = millis();
  const unsigned long VEL_INTERVAL = 50;  // Update velocity every 50ms
  
  for (int i = 0; i < 4; i++) {
    if (now - sensors[i].velTime >= VEL_INTERVAL) {
      float dt = (now - sensors[i].velTime) / 1000.0;  // seconds
      if (dt > 0) {
        // Velocity in %/second (negative = moving toward 0%)
        float newVel = (zombiePosition[i] - sensors[i].prevPosition) / dt;
        // Smooth velocity estimate
        sensors[i].velocity = 0.7 * sensors[i].velocity + 0.3 * newVel;
      }
      sensors[i].prevPosition = zombiePosition[i];
      sensors[i].velTime = now;
    }
  }
}

//============================================
// CALCULATE TIME TO IMPACT (TTI)
// Returns estimated seconds until zombie reaches 0%
// Lower TTI = more urgent
//============================================
float calculateTTI(int lane) {
  float pos = zombiePosition[lane];
  float vel = sensors[lane].velocity;  // negative = moving toward 0%
  
  // If moving forward (velocity negative), calculate time to reach 0%
  if (vel < -1.0) {
    return pos / (-vel);  // Time in seconds
  }
  
  // If stopped or moving backward, return large value (low priority)
  return 9999.0;
}

//============================================
// STATE: CHOOSE TARGET (TTI-based)
// Target the zombie with lowest time-to-impact
//============================================
void chooseTarget() {
  int bestLane = -1;
  float bestTTI = 9999.0;
  
  for (int i = 0; i < 4; i++) {
    // Only consider forward-moving zombies
    if (sensors[i].direction == FORWARD) {
      float tti = calculateTTI(i);
      
      if (tti < bestTTI) {
        bestTTI = tti;
        bestLane = i;
      }
    }
  }
  
  if (bestLane >= 0) {
    activeTarget = bestLane;
    previousPosition = encoder.read();
    desiredPosition = targetPositions[bestLane];
    state = MOVE_TO_TARGET;
  } else {
    activeTarget = -1;
    desiredPosition = WAIT_POSITION;
  }
}

//============================================
// STATE: MOVE TO TARGET
// Applies small correction based on approach direction
//============================================
void moveToTarget() {
  long currentPos = encoder.read();
  
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    // Arrived at target - apply approach correction
    // Overshoot slightly in the direction we were traveling
    long correctedPosition = desiredPosition;
    
    if (previousPosition > desiredPosition) {
      // Came from right (higher values), moving left - overshoot left
      correctedPosition = desiredPosition - APPROACH_CORRECTION;
    } else if (previousPosition < desiredPosition) {
      // Came from left (lower values), moving right - overshoot right
      correctedPosition = desiredPosition + APPROACH_CORRECTION;
    }
    
    // Apply the correction
    desiredPosition = correctedPosition;
    
    arrivalTime = millis();
    
    if (activeTarget >= 0) {
      state = DWELL;
    } else {
      state = CHOOSE_TARGET;
    }
  }
}

//============================================
// STATE: DWELL
// Hold at target for fixed duration, then choose next target
//============================================
void dwell() {
  unsigned long dwellTime = millis() - arrivalTime;
  
  if (dwellTime >= DWELL_TIME_MS) {
    activeTarget = -1;
    state = CHOOSE_TARGET;
  }
}

//============================================
// PID CONTROLLER
//============================================
void computeMotorVelocity() {
  long currentPos = encoder.read();
  unsigned long currentTime = micros();
  
  if (abs(currentPos - previousMotorPosition) > 2 || 
      (currentTime - previousVelCompTime) > 10000) {
    motorVelocity = (double)(currentPos - previousMotorPosition) * 1000000.0 / 
                    (currentTime - previousVelCompTime);
    previousMotorPosition = currentPos;
    previousVelCompTime = currentTime;
  }
}

void runPIDController() {
  long currentPos = encoder.read();
  float positionError = desiredPosition - currentPos;
  
  if (abs(positionError) <= TARGET_BAND && abs(motorVelocity) < 80) {
    stopMotor();
    errorIntegral = 0;
    return;
  }
  
  errorIntegral += positionError * (float)executionDuration / 1000000.0;
  errorIntegral = constrain(errorIntegral, -1000, 1000);
  
  float velocityError = 0 - motorVelocity;
  float voltage = KP * positionError + KI * errorIntegral + KD * velocityError;
  
  if (positionError < -5) {
    voltage -= FRICTION_LEFT;
  } else if (positionError > 5) {
    voltage += FRICTION_RIGHT;
  }
  
  if (abs(voltage) > SUPPLY_VOLTAGE) {
    errorIntegral -= positionError * (float)executionDuration / 1000000.0;
    voltage = constrain(voltage, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
  }
  
  setMotorVoltage(voltage);
}

//============================================
// MOTOR CONTROL
//============================================
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
  
  // Move toward left limit switch SOFTLY at 2.0V
  while (digitalRead(LIMIT_LEFT) == LOW) {
    setMotorVoltage(2.0);
    delay(10);
  }
  
  // Hold against limit switch gently at 2.0V for 500ms to eliminate bounce
  Serial.println(F("Holding at limit..."));
  unsigned long holdStart = millis();
  const unsigned long HOLD_TIME = 500;
  
  while (millis() - holdStart < HOLD_TIME) {
    setMotorVoltage(2.0);
    delay(10);
  }
  
  // Set encoder zero while held firmly against switch
  encoder.write(0);
  errorIntegral = 0;
  
  // Continue holding briefly to confirm position
  setMotorVoltage(2.0);
  delay(100);
  
  stopMotor();
  delay(50);
  
  Serial.println(F("Homed! Pos=0"));
}

//============================================
// SERIAL COMMANDS
//============================================
void handleSerial() {
  if (!Serial.available()) return;
  
  char cmd = Serial.peek();
  
  // Handle multi-character commands first
  if (cmd == 'K' || cmd == 'k') {
    Serial.read();
    if (Serial.available() > 0) {
      char param = Serial.read();
      float value = Serial.parseFloat();
      if (param == 'P' || param == 'p') { KP = value; Serial.print(F("KP=")); Serial.println(KP, 4); }
      else if (param == 'I' || param == 'i') { KI = value; Serial.print(F("KI=")); Serial.println(KI, 4); }
      else if (param == 'D' || param == 'd') { KD = value; Serial.print(F("KD=")); Serial.println(KD, 4); }
    }
    return;
  }
  
  if (cmd == 'F' || cmd == 'f') {
    Serial.read();
    if (Serial.available() > 0) {
      char side = Serial.read();
      float value = Serial.parseFloat();
      if (side == 'L' || side == 'l') { FRICTION_LEFT = value; Serial.print(F("FL=")); Serial.println(FRICTION_LEFT, 3); }
      else if (side == 'R' || side == 'r') { FRICTION_RIGHT = value; Serial.print(F("FR=")); Serial.println(FRICTION_RIGHT, 3); }
    }
    return;
  }
  
  if (cmd == 'V' || cmd == 'v') {
    Serial.read();
    float value = Serial.parseFloat();
    if (value > 0 && value <= 24) { SUPPLY_VOLTAGE = value; }
    Serial.print(F("V=")); Serial.println(SUPPLY_VOLTAGE, 1);
    return;
  }
  
  if (cmd == 'L' || cmd == 'l') {
    Serial.read();
    int lane = Serial.parseInt();
    if (lane >= 1 && lane <= 4) {
      long pos = Serial.parseInt();
      if (pos != 0) {
        targetPositions[lane - 1] = pos;
        Serial.print(F("L")); Serial.print(lane); Serial.print(F("=")); Serial.println(pos);
      }
    }
    return;
  }
  
  // Single character commands
  cmd = Serial.read();
  
  switch (cmd) {
    case 'G': case 'g':
    case 'T': case 't':
    case 'Y': case 'y':
      Serial.println(F("AUTO START"));
      autoMode = true;
      systemEnabled = true;
      activeTarget = -1;
      state = CHOOSE_TARGET;
      break;
      
    case 'S': case 's':
      Serial.println(F("STOP"));
      autoMode = false;
      systemEnabled = false;
      calibrationActive = false;
      stopMotor();
      break;
      
    case 'H': case 'h':
      Serial.println(F("HOME"));
      autoMode = false;
      calibrationActive = false;
      homeToLeftLimit();
      break;
      
    case '1': case '2': case '3': case '4':
      if (calibrationActive) {
        Serial.println(F("Calibration active - wait"));
      } else {
        int lane = cmd - '1';
        Serial.print(F("Move to L"));
        Serial.println(lane + 1);
        desiredPosition = targetPositions[lane];
        autoMode = false;
        systemEnabled = true;
      }
      break;
      
    case 'C': case 'c':
      while (!Serial.available()) delay(10);
      {
        char laneChar = Serial.read();
        if (laneChar >= '1' && laneChar <= '4') {
          int lane = laneChar - '1';
          long pos = encoder.read();
          targetPositions[lane] = pos;
          Serial.print(F("L"));
          Serial.print(lane + 1);
          Serial.print(F(" = "));
          Serial.println(pos);
        }
      }
      break;
      
    case 'X': case 'x':
      // Show calibration status
      Serial.println(F("=== CALIBRATION STATUS ==="));
      for (int i = 0; i < 4; i++) {
        Serial.print(F("L"));
        Serial.print(i + 1);
        Serial.print(F(": Range["));
        Serial.print(ProxRange[i][0]);
        Serial.print(F(","));
        Serial.print(ProxRange[i][1]);
        Serial.print(F("] Raw="));
        Serial.print((int)sensors[i].currVal);
        Serial.print(F(" Pos="));
        Serial.print(zombiePosition[i], 0);
        Serial.println(F("%"));
      }
      if (calibrationActive) {
        Serial.print(F("Calibrating... "));
        Serial.print((millis() - calibrationStartTime) / 1000);
        Serial.println(F("s"));
      }
      break;
      
    case 'R': case 'r':
      printRawSensors();
      break;
      
    case 'P': case 'p':
      printCurrentSettings();
      break;
      
    case 'W': case 'w':
      saveSettings();
      break;
      
    case 'D': case 'd':
      loadSettings();
      Serial.println(F("Loaded from EEPROM"));
      break;
      
    case '?':
      printHelp();
      break;
  }
}

void printRawSensors() {
  Serial.print(F("RAW: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(analogRead(sensors[i].pin));
    Serial.print(F(" "));
  }
  Serial.print(F("| POS: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(zombiePosition[i], 0);
    Serial.print(F("% "));
  }
  Serial.println();
}

void printCurrentSettings() {
  Serial.println(F("=== SETTINGS ==="));
  Serial.print(F("PID: "));
  Serial.print(KP, 4);
  Serial.print(F("/"));
  Serial.print(KI, 4);
  Serial.print(F("/"));
  Serial.println(KD, 4);
  Serial.print(F("Friction: "));
  Serial.print(FRICTION_LEFT, 3);
  Serial.print(F("/"));
  Serial.println(FRICTION_RIGHT, 3);
  Serial.print(F("Voltage: "));
  Serial.println(SUPPLY_VOLTAGE, 1);
  Serial.print(F("Encoder: "));
  Serial.println(encoder.read());
  
  Serial.println(F("Lane positions:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(targetPositions[i]);
  }
  
  Serial.println(F("Prox ranges [100%, 0%]:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": ["));
    Serial.print(ProxRange[i][0]);
    Serial.print(F(", "));
    Serial.print(ProxRange[i][1]);
    Serial.println(F("]"));
  }
}

void printHelp() {
  Serial.println(F("=== COMMANDS ==="));
  Serial.println(F("  G/T/Y - Start auto mode"));
  Serial.println(F("  S - Stop"));
  Serial.println(F("  H - Home to limit"));
  Serial.println(F("  1-4 - Move to lane"));
  Serial.println(F("  C1-C4 - Capture lane position"));
  Serial.println(F("  X - Show calibration status"));
  Serial.println(F("  R - Raw sensor values"));
  Serial.println(F("  P - Print settings"));
  Serial.println(F("  W - Save to EEPROM"));
  Serial.println(F("  D - Load from EEPROM"));
  Serial.println(F("  KP/KI/KD<val> - Set PID"));
  Serial.println(F("  FL/FR<val> - Set friction"));
  Serial.println(F("  V<val> - Set voltage"));
  Serial.println(F("  L<n> <pos> - Set lane position"));
}

//============================================
// EEPROM
//============================================
void saveSettings() {
  Serial.println(F("Saving..."));
  
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.put(EEPROM_VOLTAGE, SUPPLY_VOLTAGE);
  
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * 4, targetPositions[i]);
    EEPROM.put(EEPROM_PROX_BASE + i * 4, ProxRange[i][0]);
    EEPROM.put(EEPROM_PROX_BASE + 16 + i * 4, ProxRange[i][1]);
  }
  
  Serial.println(F("Saved!"));
}

void loadSettings() {
  EEPROM.get(EEPROM_KP, KP);
  EEPROM.get(EEPROM_KI, KI);
  EEPROM.get(EEPROM_KD, KD);
  EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.get(EEPROM_VOLTAGE, SUPPLY_VOLTAGE);
  
  if (isnan(KP) || KP < 0 || KP > 1) KP = 0.015;
  if (isnan(KI) || KI < 0 || KI > 1) KI = 0.003;
  if (isnan(KD) || KD < 0 || KD > 1) KD = 0.020;
  if (isnan(FRICTION_LEFT) || FRICTION_LEFT < 0) FRICTION_LEFT = 0.25;
  if (isnan(FRICTION_RIGHT) || FRICTION_RIGHT < 0) FRICTION_RIGHT = 0.25;
  if (isnan(SUPPLY_VOLTAGE) || SUPPLY_VOLTAGE < 1 || SUPPLY_VOLTAGE > 24) SUPPLY_VOLTAGE = 12.0;
  
  for (int i = 0; i < 4; i++) {
    long pos;
    EEPROM.get(EEPROM_LANES_BASE + i * 4, pos);
    if (pos > -2000 && pos < 100) {
      targetPositions[i] = pos;
    }
    
    int maxVal, minVal;
    EEPROM.get(EEPROM_PROX_BASE + i * 4, maxVal);
    EEPROM.get(EEPROM_PROX_BASE + 16 + i * 4, minVal);
    int range = maxVal - minVal;
    if (maxVal > 0 && maxVal < 1024 && minVal > 0 && minVal < 1024 && range > 150) {
      ProxRange[i][0] = maxVal;
      ProxRange[i][1] = minVal;
    }
  }
}

//============================================
// STATUS
//============================================
void printStatus() {
  Serial.print(F("E:"));
  Serial.print(encoder.read());
  Serial.print(F(" D:"));
  Serial.print(desiredPosition);
  Serial.print(F(" T:"));
  Serial.print(activeTarget + 1);
  Serial.print(F(" | "));
  
  for (int i = 0; i < 4; i++) {
    Serial.print((int)zombiePosition[i]);
    if (sensors[i].direction == FORWARD) Serial.print(F("F"));
    else if (sensors[i].direction == BACKWARD) Serial.print(F("B"));
    else Serial.print(F("-"));
    Serial.print(F(" "));
  }
  
  if (calibrationActive) {
    Serial.print(F("CAL:"));
    Serial.print((millis() - calibrationStartTime) / 1000);
    Serial.print(F("s"));
  }
  
  Serial.println();
}
