// ME350 Zombie Defense - OPTIMIZED for Maximum Interception
// Urgency-based targeting with velocity tracking
// Never let a zombie reach the end!

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
//============================================
int ProxRange[4][2] = {
  {615, 88},
  {634, 124},
  {622, 147},
  {590, 80}
};

//============================================
// CALIBRATION SYSTEM
//============================================
bool calibrationActive = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 12000;
unsigned long lastCalibrationLogTime = 0;

int calibrationMin[4] = {1023, 1023, 1023, 1023};
int calibrationMax[4] = {0, 0, 0, 0};
int calibrationStart[4] = {0, 0, 0, 0};
int calibrationRaw[4] = {0, 0, 0, 0};
bool calibrationUpdated[4] = {false, false, false, false};

//============================================
// PROXIMITY SENSORS - OPTIMIZED
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
float zombiePosition[4];

// OPTIMIZED: Faster response with less filtering
const float ALPHA = 0.85;              // Reduced from 0.92 for faster response
const int STOP_TIMEOUT_MS = 100;       // Reduced from 150
const int NOISE_LIMIT = 8;             // Reduced from 10 for sensitivity
const int DIR_CONFIRM_COUNT = 2;       // Reduced from 3 for faster detection
int dirForwardCount[4] = {0, 0, 0, 0};
int dirBackwardCount[4] = {0, 0, 0, 0};

//============================================
// URGENCY-BASED TARGETING
//============================================
const float DANGER_POSITION = 25.0;     // High priority threshold
const float ACTIVE_THRESHOLD = 85.0;    // Consider zombies below this

// Urgency calculation weights
const float POSITION_WEIGHT = 1.0;      // Weight for position
const float VELOCITY_WEIGHT = 0.5;      // Weight for velocity (time-to-impact)

//============================================
// TARGETING
//============================================
int activeTarget = -1;
long desiredPosition = WAIT_POSITION;
long previousPosition = WAIT_POSITION;

// OPTIMIZED: Faster dwell timing
unsigned long arrivalTime = 0;
const unsigned long DWELL_TIMEOUT_MS = 600;       // Reduced from 1000
const unsigned long BACKWARD_CONFIRM_MS = 80;     // Reduced from 150
unsigned long backwardStartTime = 0;

const int APPROACH_CORRECTION = 3;

//============================================
// PID CONTROLLER - SLIGHTLY MORE AGGRESSIVE
//============================================
float KP = 0.018;          // Increased from 0.015
float KI = 0.004;          // Increased from 0.003
float KD = 0.022;          // Increased from 0.020
float FRICTION_LEFT = 0.28;
float FRICTION_RIGHT = 0.28;
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
  Serial.println(F("ME350 Zombie Defense - OPTIMIZED"));
  Serial.println(F("================================"));
  
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
// MAIN LOOP - OPTIMIZED
//============================================
void loop() {
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();
  
  handleSerial();
  computeMotorVelocity();
  updateSensors();
  calculatePositions();
  calculateZombieVelocities();  // NEW: Track zombie speeds
  
  if (calibrationActive) {
    updateCalibration();
  }
  
  //--- STATE MACHINE ---
  if (calibrationActive) {
    desiredPosition = 0;
  } else if (autoMode && systemEnabled) {
    switch (state) {
      case IDLE:
        break;
        
      case CHOOSE_TARGET:
        chooseTargetOptimized();
        break;
      
      case MOVE_TO_TARGET:
        moveToTarget();
        break;
      
      case DWELL:
        dwellOptimized();
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
  
  //--- STATUS OUTPUT ---
  if (autoMode && (millis() - lastPrintTime >= 250)) {
    lastPrintTime = millis();
    printStatus();
  }
}

//============================================
// NEW: ZOMBIE VELOCITY CALCULATION
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
// CALCULATE URGENCY SCORE
// Lower score = more urgent (needs attention first)
// Combines position and time-to-impact
//============================================
float calculateUrgency(int lane) {
  float pos = zombiePosition[lane];
  float vel = sensors[lane].velocity;  // negative = moving toward 0%
  
  // Base urgency is position (lower position = more urgent)
  float urgency = pos * POSITION_WEIGHT;
  
  // Adjust for velocity - fast movers are more urgent
  if (vel < -5.0) {  // Moving forward significantly
    // Estimate time to reach 0% (in seconds)
    float timeToEnd = pos / (-vel);
    // Faster zombies get lower (more urgent) score
    urgency += timeToEnd * VELOCITY_WEIGHT * 10.0;
  } else if (vel > 5.0) {
    // Moving backward - less urgent
    urgency += 50.0;
  } else {
    // Stopped or slow - moderate urgency based on position
    urgency += 20.0;
  }
  
  return urgency;
}

//============================================
// OPTIMIZED: CHOOSE TARGET BY URGENCY
//============================================
void chooseTargetOptimized() {
  int bestLane = -1;
  float bestUrgency = 9999.0;
  
  for (int i = 0; i < 4; i++) {
    // Consider any zombie that's active (below threshold)
    if (zombiePosition[i] < ACTIVE_THRESHOLD) {
      // Must be moving forward OR in danger zone
      if (sensors[i].direction == FORWARD || 
          zombiePosition[i] < DANGER_POSITION) {
        
        float urgency = calculateUrgency(i);
        
        if (urgency < bestUrgency) {
          bestUrgency = urgency;
          bestLane = i;
        }
      }
    }
  }
  
  if (bestLane >= 0) {
    activeTarget = bestLane;
    previousPosition = encoder.read();
    desiredPosition = targetPositions[bestLane];
    
    Serial.print(F("TARGET L"));
    Serial.print(bestLane + 1);
    Serial.print(F(" pos="));
    Serial.print(zombiePosition[bestLane], 0);
    Serial.print(F("% vel="));
    Serial.print(sensors[bestLane].velocity, 1);
    Serial.print(F(" urg="));
    Serial.println(bestUrgency, 0);
    
    state = MOVE_TO_TARGET;
  } else {
    // No active targets - pre-position strategically
    activeTarget = -1;
    desiredPosition = calculatePrePosition();
  }
}

//============================================
// NEW: STRATEGIC PRE-POSITIONING
// Move toward the most likely next threat
//============================================
long calculatePrePosition() {
  float weightedSum = 0;
  float totalWeight = 0;
  
  for (int i = 0; i < 4; i++) {
    // Weight by how close/active each lane is
    float weight = 0;
    
    if (zombiePosition[i] < 95.0) {
      // Closer zombies get more weight
      weight = (100.0 - zombiePosition[i]) / 100.0;
      
      // Forward-moving zombies get extra weight
      if (sensors[i].direction == FORWARD) {
        weight *= 2.0;
      }
    }
    
    if (weight > 0) {
      weightedSum += targetPositions[i] * weight;
      totalWeight += weight;
    }
  }
  
  if (totalWeight > 0) {
    return (long)(weightedSum / totalWeight);
  }
  
  return WAIT_POSITION;
}

//============================================
// STATE: MOVE TO TARGET (unchanged logic)
//============================================
void moveToTarget() {
  long currentPos = encoder.read();
  
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    long correctedPosition = desiredPosition;
    
    if (previousPosition > desiredPosition) {
      correctedPosition = desiredPosition - APPROACH_CORRECTION;
    } else if (previousPosition < desiredPosition) {
      correctedPosition = desiredPosition + APPROACH_CORRECTION;
    }
    
    desiredPosition = correctedPosition;
    arrivalTime = millis();
    backwardStartTime = 0;
    
    if (activeTarget >= 0) {
      Serial.print(F("ARRIVED L"));
      Serial.print(activeTarget + 1);
      Serial.print(F(" @"));
      Serial.print(zombiePosition[activeTarget], 0);
      Serial.println(F("%"));
      state = DWELL;
    } else {
      state = CHOOSE_TARGET;
    }
  }
}

//============================================
// OPTIMIZED: FASTER DWELL
//============================================
void dwellOptimized() {
  unsigned long now = millis();
  unsigned long dwellTime = now - arrivalTime;
  
  // Quick hit detection
  if (sensors[activeTarget].direction == BACKWARD) {
    if (backwardStartTime == 0) {
      backwardStartTime = now;
    }
    if (now - backwardStartTime >= BACKWARD_CONFIRM_MS) {
      Serial.print(F("HIT L"));
      Serial.print(activeTarget + 1);
      Serial.print(F(" in "));
      Serial.print(dwellTime);
      Serial.println(F("ms"));
      activeTarget = -1;
      state = CHOOSE_TARGET;
      return;
    }
  } else {
    backwardStartTime = 0;
  }
  
  // Check if zombie stopped (shorter wait)
  if (sensors[activeTarget].direction == STOPPED && dwellTime >= 300) {
    Serial.print(F("STOPPED L"));
    Serial.println(activeTarget + 1);
    activeTarget = -1;
    state = CHOOSE_TARGET;
    return;
  }
  
  // Check if zombie moved significantly backward (definite hit)
  if (zombiePosition[activeTarget] > 60.0 && dwellTime > 200) {
    Serial.print(F("CLEAR L"));
    Serial.println(activeTarget + 1);
    activeTarget = -1;
    state = CHOOSE_TARGET;
    return;
  }
  
  // Timeout
  if (dwellTime >= DWELL_TIMEOUT_MS) {
    Serial.print(F("TIMEOUT L"));
    Serial.println(activeTarget + 1);
    activeTarget = -1;
    state = CHOOSE_TARGET;
    return;
  }
}

//============================================
// CALIBRATION FUNCTIONS (unchanged)
//============================================
void startCalibration() {
  calibrationActive = true;
  calibrationStartTime = millis();
  lastCalibrationLogTime = 0;
  
  desiredPosition = 0;
  systemEnabled = true;
  state = IDLE;
  
  Serial.println(F("=== CALIBRATION STARTED (10s) ==="));
  Serial.println(F("Mechanism holding at home position (0)..."));
  
  for (int i = 0; i < 4; i++) {
    calibrationStart[i] = analogRead(sensors[i].pin);
    calibrationMin[i] = 1023;
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
  
  for (int i = 0; i < 4; i++) {
    int rawVal = analogRead(sensors[i].pin);
    calibrationRaw[i] = rawVal;
    
    if (rawVal < calibrationMin[i]) {
      calibrationMin[i] = rawVal;
      calibrationUpdated[i] = true;
    }
    if (rawVal > calibrationMax[i]) {
      calibrationMax[i] = rawVal;
      calibrationUpdated[i] = true;
    }
  }
  
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
    int startVal = max(calibrationStart[i], calibrationMax[i]);
    int endVal = calibrationMin[i];
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
      int newMax = min(startVal + 2, 1023);
      int newMin = max(endVal - 2, 0);
      int finalRange = newMax - newMin;
      
      if (finalRange > 120) {
        ProxRange[i][0] = newMax;
        ProxRange[i][1] = newMin;
        
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
// UPDATE SENSORS - OPTIMIZED
//============================================
void updateSensors() {
  unsigned long now = millis();
  
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
    else if (delta < 0) {
      dirForwardCount[i]++;
      dirBackwardCount[i] = 0;
      if (dirForwardCount[i] >= DIR_CONFIRM_COUNT) {
        sensors[i].direction = FORWARD;
        sensors[i].prevVal = sensors[i].currVal;
        sensors[i].prevTime = now;
      }
    } 
    else {
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
// CALCULATE POSITIONS
//============================================
void calculatePositions() {
  for (int i = 0; i < 4; i++) {
    float raw = sensors[i].currVal;
    float minVal = ProxRange[i][1];
    float maxVal = ProxRange[i][0];
    
    if (maxVal != minVal) {
      zombiePosition[i] = (raw - minVal) / (maxVal - minVal) * 100.0;
    } else {
      zombiePosition[i] = 50.0;
    }
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
  
  while (digitalRead(LIMIT_LEFT) == LOW) {
    setMotorVoltage(2.0);
    delay(10);
  }
  
  Serial.println(F("Holding at limit..."));
  unsigned long holdStart = millis();
  const unsigned long HOLD_TIME = 500;
  
  while (millis() - holdStart < HOLD_TIME) {
    setMotorVoltage(2.0);
    delay(10);
  }
  
  encoder.write(0);
  errorIntegral = 0;
  
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
  
  cmd = Serial.read();
  
  switch (cmd) {
    case 'G': case 'g':
    case 'T': case 't':
    case 'Y': case 'y':
      Serial.println(F("AUTO START + CALIBRATION"));
      autoMode = true;
      systemEnabled = true;
      activeTarget = -1;
      startCalibration();
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
      Serial.println(F("=== STATUS ==="));
      for (int i = 0; i < 4; i++) {
        Serial.print(F("L"));
        Serial.print(i + 1);
        Serial.print(F(": pos="));
        Serial.print(zombiePosition[i], 0);
        Serial.print(F("% vel="));
        Serial.print(sensors[i].velocity, 1);
        Serial.print(F(" dir="));
        if (sensors[i].direction == FORWARD) Serial.print(F("FWD"));
        else if (sensors[i].direction == BACKWARD) Serial.print(F("BWD"));
        else Serial.print(F("STP"));
        Serial.print(F(" urg="));
        Serial.println(calculateUrgency(i), 0);
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
  Serial.print(F("| VEL: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(sensors[i].velocity, 0);
    Serial.print(F(" "));
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
  Serial.println(F("  G/T/Y - Start auto + calibration"));
  Serial.println(F("  S - Stop"));
  Serial.println(F("  H - Home to limit"));
  Serial.println(F("  1-4 - Move to lane"));
  Serial.println(F("  C1-C4 - Capture lane position"));
  Serial.println(F("  X - Show status + urgency"));
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
  
  if (isnan(KP) || KP < 0 || KP > 1) KP = 0.018;
  if (isnan(KI) || KI < 0 || KI > 1) KI = 0.004;
  if (isnan(KD) || KD < 0 || KD > 1) KD = 0.022;
  if (isnan(FRICTION_LEFT) || FRICTION_LEFT < 0) FRICTION_LEFT = 0.28;
  if (isnan(FRICTION_RIGHT) || FRICTION_RIGHT < 0) FRICTION_RIGHT = 0.28;
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
    if (zombiePosition[i] < DANGER_POSITION) Serial.print(F("!"));
    Serial.print(F(" "));
  }
  
  if (calibrationActive) {
    Serial.print(F("CAL:"));
    Serial.print((millis() - calibrationStartTime) / 1000);
    Serial.print(F("s"));
  }
  
  Serial.println();
}
