// ME 350 Lab 11 - Smooth Control with Adaptive Voltage Boost
// Team 25 - FIXED VERSION

#include <Encoder.h>
#include <EEPROM.h>

#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_ENA 11
#define MOTOR_IN2 12
#define MOTOR_IN3 13
#define LIMIT_LEFT 8
#define LIMIT_RIGHT 9

#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

// Flip (enable) switch
#define ON_OFF_SWITCH_PIN 5  // HIGH = enabled, LOW = stop

Encoder encoder(ENCODER_A, ENCODER_B);

#define EEPROM_FRICTION_RIGHT 0
#define EEPROM_FRICTION_LEFT 4
#define EEPROM_CALIBRATED 8
#define EEPROM_RIGHT_LIMIT 9
#define EEPROM_RANGE_CAL 13

enum State {
  CALIBRATE,
  IDLE,
  CHOOSE_ACTIVE_TARGET,
  MOVE_TO_TARGET,
  FINE_TUNE_POSITION
};

State currentState = CALIBRATE;

long LANE_1_POSITION = -80;
long LANE_2_POSITION = -344;
long LANE_3_POSITION = -585;
long LANE_4_POSITION = -1232;

long getLanePosition(int lane) {
  switch(lane) {
    case 0: return LANE_1_POSITION;
    case 1: return LANE_2_POSITION;
    case 2: return LANE_3_POSITION;
    case 3: return LANE_4_POSITION;
    default: return LANE_2_POSITION;
  }
}

long LEFT_LIMIT = 0;
long RIGHT_LIMIT = -1300;
bool rangeCalibrated = false;

struct Target {
  int rawValue;
  int prevRawValue;
  int smoothedValue;
  int distance;
  bool exists;
  bool movingForward;
  unsigned long lastUpdateTime;
  float approachRate;
  int maxObserved;
  int minObserved;
};

Target targets[4];

int activeTargetLane = -1;
long activeTargetPosition = 0;

const float SENSOR_ALPHA = 0.7;
const int PROX_DETECTION_THRESHOLD = 80;
const int PROX_DIRECTION_THRESHOLD = 15;

unsigned long targetReachedTime = 0;
const unsigned long DWELL_TIME = 800;
unsigned long lightOnTime = 0;
int lightOnBaseline = 0;
bool needsFineTune = false;
int fineTuneDirection = 1;
int fineTuneAttempts = 0;

float KP = 0.28;
float KI = 0.015;
float KD = 0.012;

const float MAX_VOLTAGE = 8.0;
const int DEADBAND = 0;
const float MAX_INTEGRAL = 400.0;
const unsigned long CONTROL_PERIOD = 10;

// FIX #1: Better naming - these are the voltages needed to overcome friction
// FRICTION_LEFT is used when moving to MORE NEGATIVE positions (away from home)
// FRICTION_RIGHT is used when moving to LESS NEGATIVE positions (toward home)
float FRICTION_LEFT = 1.55;   // For moving toward more negative (right/away)
float FRICTION_RIGHT = 2.90;  // For moving toward less negative (left/toward home)
bool frictionCalibrated = false;

const float HOMING_VOLTAGE = 5.5;

long targetPosition = 0;
float errorIntegral = 0;
float lastError = 0;
unsigned long lastControlTime = 0;

bool systemEnabled = false;
bool autoMode = false;

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 25;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 400;

long stuckCheckPosition = 0;
unsigned long stuckCheckTime = 0;
float voltageBoost = 0.0;
const float MAX_VOLTAGE_BOOST = 3.0;
const float VOLTAGE_BOOST_INCREMENT = 0.2;
const unsigned long STUCK_CHECK_INTERVAL = 250;

void saveFrictionCalibration() {
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.write(EEPROM_CALIBRATED, 1);
}

void loadFrictionCalibration() {
  byte calibrated = EEPROM.read(EEPROM_CALIBRATED);
  if (calibrated == 1) {
    EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
    EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
    frictionCalibrated = true;
    Serial.print(F("✓ Friction: L="));
    Serial.print(FRICTION_LEFT, 2);
    Serial.print(F("V (→neg) R="));
    Serial.print(FRICTION_RIGHT, 2);
    Serial.println(F("V (→0)"));
  }
}

void saveRangeCalibration() {
  EEPROM.put(EEPROM_RIGHT_LIMIT, RIGHT_LIMIT);
  EEPROM.write(EEPROM_RANGE_CAL, 1);
}

void loadRangeCalibration() {
  byte calibrated = EEPROM.read(EEPROM_RANGE_CAL);
  if (calibrated == 1) {
    EEPROM.get(EEPROM_RIGHT_LIMIT, RIGHT_LIMIT);
    rangeCalibrated = true;
    Serial.print(F("✓ Range: "));
    Serial.print(LEFT_LIMIT);
    Serial.print(F(" to "));
    Serial.println(RIGHT_LIMIT);
  }
}

// FIX #2: Corrected friction direction logic
float getAdaptiveFriction(long currentPos, long targetPos) {
  float error = targetPos - currentPos;
  
  // If error is negative, target is MORE negative than current → need to move toward more negative
  // That requires FRICTION_LEFT
  // If error is positive, target is LESS negative than current → need to move toward less negative (toward 0)
  // That requires FRICTION_RIGHT
  bool movingTowardMoreNegative = (error < 0);
  float baseFriction = movingTowardMoreNegative ? FRICTION_LEFT : FRICTION_RIGHT;
  
  float positionFactor = 1.0;
  long absPos = abs(currentPos);
  
  if (absPos > 1000) {
    positionFactor = 1.12;
  } else if (absPos > 600) {
    positionFactor = 1.06;
  } else if (absPos > 300) {
    positionFactor = 1.0;
  } else {
    positionFactor = 0.92;
  }
  
  float absError = abs(error);
  float errorScale = 1.0;
  
  if (absError > 500) {
    errorScale = 1.0;
  } else if (absError > 300) {
    errorScale = 0.9;
  } else if (absError > 150) {
    errorScale = 0.75;
  } else if (absError > 80) {
    errorScale = 0.6;
  } else if (absError > 40) {
    errorScale = 0.5;
  } else if (absError > 20) {
    errorScale = 0.4;
  } else if (absError > 10) {
    errorScale = 0.35;
  } else if (absError > 5) {
    errorScale = 0.3;
  } else {
    errorScale = 0.25;
  }
  
  return baseFriction * positionFactor * errorScale;
}

bool calibrateRange() {
  Serial.println(F("\n=== RANGE CALIBRATION ===\n"));
  
  autoMode = false;
  systemEnabled = false;
  stopMotor();
  delay(200);
  
  Serial.println(F("Homing to LEFT..."));
  if (!homeToLeftLimit()) {
    Serial.println(F("✗ Failed\n"));
    return false;
  }
  
  LEFT_LIMIT = encoder.read();
  Serial.print(F("Left: "));
  Serial.println(LEFT_LIMIT);
  delay(500);
  
  Serial.println(F("Finding RIGHT limit..."));
  
  setMotor(-5.5);
  unsigned long startTime = millis();
  
  while (!rightPressed() && (millis() - startTime) < 20000) {
    delay(10);
  }
  
  if (rightPressed()) {
    delay(300);
    RIGHT_LIMIT = encoder.read();
    stopMotor();
    delay(200);
    
    Serial.print(F("Right: "));
    Serial.println(RIGHT_LIMIT);
    
    long range = abs(RIGHT_LIMIT - LEFT_LIMIT);
    Serial.print(F("Range: "));
    Serial.print(range);
    Serial.println(F(" counts"));
    
    rangeCalibrated = true;
    saveRangeCalibration();
    
    homeToLeftLimit();
    
    Serial.println(F("✓ Complete\n"));
    return true;
    
  } else {
    stopMotor();
    Serial.println(F("✗ Timeout\n"));
    return false;
  }
}

bool calibrateFriction() {
  Serial.println(F("\n=== FRICTION CALIBRATION ===\n"));
  
  if (!rangeCalibrated) {
    Serial.print(F("Range not calibrated. Run now? (Y/N): "));
    
    while (!Serial.available()) delay(10);
    char response = toupper(Serial.read());
    while (Serial.available()) Serial.read();
    Serial.println(response);
    
    if (response == 'Y') {
      if (!calibrateRange()) return false;
    } else {
      Serial.println(F("Cannot calibrate friction without range\n"));
      return false;
    }
  }
  
  autoMode = false;
  systemEnabled = false;
  stopMotor();
  delay(200);
  
  Serial.println(F("Homing..."));
  if (!homeToLeftLimit()) {
    Serial.println(F("✗ Failed\n"));
    return false;
  }
  delay(500);
  
  Serial.println(F("\nTesting LEFT friction (→ more negative)..."));
  
  long startPos = encoder.read();
  float testVoltage = 0.5;
  const float voltageStep = 0.15;
  const int movementThreshold = 20;
  bool motionDetected = false;
  
  FRICTION_LEFT = 0;
  
  while (testVoltage <= 6.0 && !motionDetected) {
    startPos = encoder.read();
    
    Serial.print(F("  "));
    Serial.print(testVoltage, 2);
    Serial.print(F("V... "));
    
    setMotor(-testVoltage);  // Negative voltage moves toward more negative
    delay(350);
    
    long endPos = encoder.read();
    long movement = abs(endPos - startPos);
    
    stopMotor();
    delay(150);
    
    Serial.print(movement);
    Serial.println(F(" counts"));
    
    if (movement >= movementThreshold) {
      FRICTION_LEFT = testVoltage;
      motionDetected = true;
      Serial.print(F("  ✓ FRICTION_LEFT = "));
      Serial.print(FRICTION_LEFT, 2);
      Serial.println(F("V\n"));
      break;
    } else {
      testVoltage += voltageStep;
    }
  }
  
  if (!motionDetected) {
    Serial.println(F("  Using default 1.6V\n"));
    FRICTION_LEFT = 1.6;
  }
  
  delay(500);
  
  long middlePos = (LEFT_LIMIT + RIGHT_LIMIT) / 2;
  Serial.print(F("Moving to middle ("));
  Serial.print(middlePos);
  Serial.println(F(")..."));
  
  systemEnabled = true;
  currentState = MOVE_TO_TARGET;
  targetPosition = middlePos;
  errorIntegral = 0;
  lastError = 0;
  voltageBoost = 0;
  
  unsigned long moveStart = millis();
  while (abs(encoder.read() - targetPosition) > 20 && (millis() - moveStart) < 10000) {
    runPIDControl();
    delay(10);
  }
  
  stopMotor();
  systemEnabled = false;
  delay(500);
  
  Serial.print(F("Position: "));
  Serial.println(encoder.read());
  
  Serial.println(F("\nTesting RIGHT friction (→ less negative)..."));
  
  testVoltage = 0.5;
  motionDetected = false;
  FRICTION_RIGHT = 0;
  
  while (testVoltage <= 6.0 && !motionDetected) {
    startPos = encoder.read();
    
    Serial.print(F("  "));
    Serial.print(testVoltage, 2);
    Serial.print(F("V... "));
    
    setMotor(testVoltage);  // Positive voltage moves toward less negative
    delay(350);
    
    long endPos = encoder.read();
    long movement = abs(endPos - startPos);
    
    stopMotor();
    delay(150);
    
    Serial.print(movement);
    Serial.println(F(" counts"));
    
    if (movement >= movementThreshold) {
      FRICTION_RIGHT = testVoltage;
      motionDetected = true;
      Serial.print(F("  ✓ FRICTION_RIGHT = "));
      Serial.print(FRICTION_RIGHT, 2);
      Serial.println(F("V\n"));
      break;
    } else {
      testVoltage += voltageStep;
    }
  }
  
  if (!motionDetected) {
    Serial.println(F("  Using default 2.9V\n"));
    FRICTION_RIGHT = 2.9;
  }
  
  Serial.println(F("=== COMPLETE ==="));
  Serial.print(F("LEFT (→neg): "));
  Serial.print(FRICTION_LEFT, 2);
  Serial.print(F("V  RIGHT (→0): "));
  Serial.print(FRICTION_RIGHT, 2);
  Serial.println(F("V\n"));
  
  frictionCalibrated = true;
  saveFrictionCalibration();
  
  homeToLeftLimit();
  
  Serial.println(F("✓ Ready\n"));
  return true;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  
  pinMode(LIMIT_LEFT, INPUT_PULLUP);  // active HIGH per hardware wiring
  pinMode(LIMIT_RIGHT, INPUT_PULLUP); // active HIGH per hardware wiring
  
  pinMode(PROX_SENSOR_1, INPUT);
  pinMode(PROX_SENSOR_2, INPUT);
  pinMode(PROX_SENSOR_3, INPUT);
  pinMode(PROX_SENSOR_4, INPUT);

  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);
  
  stopMotor();
  delay(500);
  
  for (int i = 0; i < 4; i++) {
    targets[i].rawValue = analogRead(PROX_SENSOR_1 + i);
    targets[i].prevRawValue = targets[i].rawValue;
    targets[i].smoothedValue = targets[i].rawValue;
    targets[i].distance = 1000;
    targets[i].exists = false;
    targets[i].movingForward = false;
    targets[i].lastUpdateTime = millis();
    targets[i].approachRate = 0;
    targets[i].maxObserved = 0;
    targets[i].minObserved = 1023;
  }
  
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   ME 350 LAB 11 - ADAPTIVE BOOST          ║"));
  Serial.println(F("║   Team 25 - Smart Voltage Control         ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  loadRangeCalibration();
  loadFrictionCalibration();
  
  Serial.println(F("STARTUP:"));
  Serial.println(F("  R - Calibrate range"));
  Serial.println(F("  F - Calibrate friction"));
  Serial.println(F("  C - Calibrate lanes"));
  Serial.println(F("  A - Start AUTO\n"));
  
  printHelp();
}

void loop() {
  unsigned long currentTime = millis();

  // Flip switch safety: if off, stop and skip control
  if (digitalRead(ON_OFF_SWITCH_PIN) == LOW) {
    stopMotor();
    systemEnabled = false;
    errorIntegral = 0;
    voltageBoost = 0;
    return;
  }

  if (Serial.available() > 0) {
    processCommand();
  }
  
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentTime;
    readAndLearnProximity();
  }
  
  if (systemEnabled && (currentTime - lastControlTime >= CONTROL_PERIOD)) {
    lastControlTime = currentTime;
    
    if (autoMode) {
      stateMachine();
    }
    
    runPIDControl();
  }
  
  if (autoMode && (currentTime - lastPrintTime >= PRINT_INTERVAL)) {
    lastPrintTime = currentTime;
    printStatus();
  }
  
  checkLimitSwitches();
}

// FIX #3: Improved homing that ensures encoder is at 0
bool homeToLeftLimit() {
  stopMotor();
  delay(100);
  
  // If already at limit, back off first
  if (leftPressed()) {
    Serial.println(F("  Backing off..."));
    setMotor(-3.0);
    delay(300);
    stopMotor();
    delay(200);
  }
  
  // Now approach the limit switch
  unsigned long startTime = millis();
  setMotor(5.5);
  
  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }
  
  if (leftPressed()) {
    stopMotor();
    delay(200);
    
    // Reset encoder multiple times to ensure it sticks
    encoder.write(0);
    delay(50);
    encoder.write(0);
    delay(50);
    
    LEFT_LIMIT = 0;
    
    // Verify
    long pos = encoder.read();
    Serial.print(F("  Encoder: "));
    Serial.println(pos);
    
    return true;
  } else {
    stopMotor();
    return false;
  }
}

void stateMachine() {
  
  switch (currentState) {
    
    case CALIBRATE: {
      targetPosition = 0;
      
      if (leftPressed() && abs(encoder.read()) < 5) {
        encoder.write(0);
        stopMotor();
        errorIntegral = 0;
        lastError = 0;
        voltageBoost = 0;
        currentState = IDLE;
        Serial.println(F("✓ Homed → IDLE"));
      } else {
        setMotor(HOMING_VOLTAGE);
      }
      break;
    }
    
    case IDLE: {
      targetPosition = 0;
      
      bool threatDetected = false;
      for (int i = 0; i < 4; i++) {
        if (targets[i].exists && targets[i].movingForward) {
          threatDetected = true;
          break;
        }
      }
      
      if (threatDetected) {
        currentState = CHOOSE_ACTIVE_TARGET;
      }
      break;
    }
    
    case CHOOSE_ACTIVE_TARGET: {
      int closestLane = -1;
      int minDistance = 1001;
      
      for (int i = 0; i < 4; i++) {
        if (targets[i].exists && targets[i].movingForward) {
          if (targets[i].distance < minDistance) {
            minDistance = targets[i].distance;
            closestLane = i;
          }
        }
      }
      
      if (closestLane >= 0) {
        activeTargetLane = closestLane;
        activeTargetPosition = getLanePosition(closestLane);
        targetPosition = activeTargetPosition;
        errorIntegral = 0;
        lastError = 0;
        voltageBoost = 0;
        stuckCheckPosition = encoder.read();
        stuckCheckTime = millis();
        needsFineTune = false;
        fineTuneAttempts = 0;
        
        Serial.print(F("🎯 Lane "));
        Serial.print(closestLane + 1);
        Serial.print(F(" @"));
        Serial.println(activeTargetPosition);
        
        currentState = MOVE_TO_TARGET;
      } else {
        currentState = IDLE;
      }
      break;
    }
    
    case MOVE_TO_TARGET: {
      targetPosition = activeTargetPosition;
      
      long currentPos = encoder.read();
      long error = targetPosition - currentPos;
      
      if (abs(error) == 0) {
        voltageBoost = 0;
        
        if (targetReachedTime == 0) {
          targetReachedTime = millis();
          lightOnTime = millis();
          lightOnBaseline = targets[activeTargetLane].rawValue;
          Serial.println(F("✓ ZERO ERROR"));
        }
        
        unsigned long onTargetTime = millis() - targetReachedTime;
        
        if (onTargetTime > 250 && onTargetTime < DWELL_TIME && !needsFineTune) {
          int currentReading = targets[activeTargetLane].rawValue;
          int delta = currentReading - lightOnBaseline;
          
          if (delta > -8) {
            needsFineTune = true;
            fineTuneDirection = 1;
            fineTuneAttempts = 0;
            currentState = FINE_TUNE_POSITION;
            Serial.println(F("Fine-tuning..."));
            break;
          }
        }
        
        if (onTargetTime >= DWELL_TIME) {
          Serial.println(F("✓ Dwell done"));
          
          targetReachedTime = 0;
          currentState = CHOOSE_ACTIVE_TARGET;
        }
        
      } else {
        targetReachedTime = 0;
      }
      
      break;
    }
    
    case FINE_TUNE_POSITION: {
      if (fineTuneAttempts >= 8) {
        targetReachedTime = 0;
        voltageBoost = 0;
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
      targetPosition = activeTargetPosition + (fineTuneDirection * 2 * fineTuneAttempts);
      
      long currentPos = encoder.read();
      if (abs(targetPosition - currentPos) == 0) {
        delay(150);
        
        int currentReading = targets[activeTargetLane].rawValue;
        int delta = currentReading - lightOnBaseline;
        
        if (delta < -8) {
          Serial.println(F("✓ Fine-tuned"));
          
          lightOnTime = millis();
          lightOnBaseline = currentReading;
          voltageBoost = 0;
          currentState = MOVE_TO_TARGET;
          targetReachedTime = millis() - 300;
        } else {
          fineTuneAttempts++;
          if (fineTuneAttempts % 3 == 0) {
            fineTuneDirection *= -1;
          }
        }
      }
      
      break;
    }
  }
}

void readAndLearnProximity() {
  
  for (int i = 0; i < 4; i++) {
    targets[i].prevRawValue = targets[i].rawValue;
    
    int newRaw = analogRead(PROX_SENSOR_1 + i);
    
    targets[i].smoothedValue = (SENSOR_ALPHA * targets[i].smoothedValue) + 
                               ((1.0 - SENSOR_ALPHA) * newRaw);
    targets[i].rawValue = targets[i].smoothedValue;
    
    if (targets[i].rawValue > targets[i].maxObserved) {
      targets[i].maxObserved = targets[i].rawValue;
    }
    if (targets[i].rawValue < targets[i].minObserved && targets[i].rawValue > 50) {
      targets[i].minObserved = targets[i].rawValue;
    }
    
    int range = targets[i].maxObserved - targets[i].minObserved;
    if (range > 100) {
      int normalizedDist = map(targets[i].rawValue, 
                              targets[i].maxObserved, 
                              targets[i].minObserved, 
                              0, 1000);
      targets[i].distance = constrain(normalizedDist, 0, 1000);
    } else {
      int normalizedDist = map(targets[i].rawValue, 900, 100, 0, 1000);
      targets[i].distance = constrain(normalizedDist, 0, 1000);
    }
    
    targets[i].exists = (targets[i].rawValue > (targets[i].minObserved + PROX_DETECTION_THRESHOLD));
    
    int delta = targets[i].rawValue - targets[i].prevRawValue;
    
    if (abs(delta) > PROX_DIRECTION_THRESHOLD) {
      if (delta > 0) {
        targets[i].movingForward = true;
        targets[i].lastUpdateTime = millis();
        
        float dt = SENSOR_READ_INTERVAL / 1000.0;
        targets[i].approachRate = delta / dt;
      } else if (delta < -PROX_DIRECTION_THRESHOLD) {
        targets[i].movingForward = false;
        targets[i].lastUpdateTime = millis();
        targets[i].approachRate = 0;
      }
    }
    
    if (millis() - targets[i].lastUpdateTime > 1500) {
      targets[i].movingForward = false;
      targets[i].approachRate = 0;
    }
  }
}

void runPIDControl() {
  if (currentState == CALIBRATE) {
    return;
  }
  
  long currentPosition = encoder.read();
  float error = targetPosition - currentPosition;
  
  if (abs(error) == 0) {
    stopMotor();
    errorIntegral = 0;
    voltageBoost = 0;
    stuckCheckPosition = currentPosition;
    stuckCheckTime = millis();
    return;
  }
  
  if (millis() - stuckCheckTime >= STUCK_CHECK_INTERVAL) {
    long movement = abs(currentPosition - stuckCheckPosition);
    
    if (movement < 2 && abs(error) > 3) {
      voltageBoost += VOLTAGE_BOOST_INCREMENT;
      voltageBoost = constrain(voltageBoost, 0, MAX_VOLTAGE_BOOST);
      
      if (voltageBoost > 0.3) {
        Serial.print(F("⚡ Boost: +"));
        Serial.print(voltageBoost, 2);
        Serial.println(F("V"));
      }
    } else if (movement > 8) {
      if (voltageBoost > 0) {
        voltageBoost -= VOLTAGE_BOOST_INCREMENT * 0.5;
        if (voltageBoost < 0) voltageBoost = 0;
      }
    }
    
    stuckCheckPosition = currentPosition;
    stuckCheckTime = millis();
  }
  
  float dt = CONTROL_PERIOD / 1000.0;
  
  if (abs(error) < 150) {
    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  } else {
    errorIntegral *= 0.90;
  }
  
  float errorDerivative = (error - lastError) / dt;
  errorDerivative = constrain(errorDerivative, -300, 300);
  
  float pidVoltage = (KP * error) + (KI * errorIntegral) + (KD * errorDerivative);
  
  float frictionVoltage = getAdaptiveFriction(currentPosition, targetPosition);
  
  float totalVoltage;
  if (error < 0) {
    // Need to move toward MORE NEGATIVE, so add friction compensation
    totalVoltage = pidVoltage - frictionVoltage - voltageBoost;
  } else {
    // Need to move toward LESS NEGATIVE (toward 0), so add friction compensation
    totalVoltage = pidVoltage + frictionVoltage + voltageBoost;
  }
  
  totalVoltage = constrain(totalVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);
  
  if (abs(error) > 0) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
  }
  
  lastError = error;
}

void setMotor(float voltage) {
  if (digitalRead(ON_OFF_SWITCH_PIN) == LOW) {
    stopMotor();
    return;
  }

  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;
  
  if (voltage > 0) {
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (voltage < 0) {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
  }
  
  analogWrite(MOTOR_ENA, pwm);
}

void stopMotor() {
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  analogWrite(MOTOR_ENA, 0);
}

bool leftPressed() {
  return digitalRead(LIMIT_LEFT) == HIGH;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == HIGH;
}

void checkLimitSwitches() {
  if (leftPressed() || rightPressed()) {
    if (autoMode && abs(targetPosition) > 10) {
      stopMotor();
      errorIntegral = 0;
      voltageBoost = 0;
    }
  }
}

void manualCalibration() {
  Serial.println(F("\n=== LANE CALIBRATION ===\n"));
  
  autoMode = false;
  systemEnabled = false;
  stopMotor();
  
  Serial.println(F("Lane 1, press ENTER"));
  waitForEnter();
  LANE_1_POSITION = encoder.read();
  
  Serial.println(F("Lane 2, press ENTER"));
  waitForEnter();
  LANE_2_POSITION = encoder.read();
  
  Serial.println(F("Lane 3, press ENTER"));
  waitForEnter();
  LANE_3_POSITION = encoder.read();
  
  Serial.println(F("Lane 4, press ENTER"));
  waitForEnter();
  LANE_4_POSITION = encoder.read();
  
  Serial.println(F("\n✓ Complete"));
  Serial.print(F("1:")); Serial.println(LANE_1_POSITION);
  Serial.print(F("2:")); Serial.println(LANE_2_POSITION);
  Serial.print(F("3:")); Serial.println(LANE_3_POSITION);
  Serial.print(F("4:")); Serial.println(LANE_4_POSITION);
  Serial.println();
}

void waitForEnter() {
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();
}

void processCommand() {
  char cmd = Serial.read();
  while (Serial.available()) Serial.read();
  
  cmd = toupper(cmd);
  
  switch (cmd) {
    case 'A':
      Serial.println(F("\n=== AUTO START ==="));
      Serial.println(F("Homing first..."));
      
      autoMode = false;
      systemEnabled = false;
      stopMotor();
      
      if (homeToLeftLimit()) {
        Serial.println(F("✓ Homed"));
        
        // FIX #4: Make sure we're actually at 0 before starting
        delay(300);
        long pos = encoder.read();
        if (abs(pos) > 10) {
          Serial.print(F("⚠ Position not zero ("));
          Serial.print(pos);
          Serial.println(F("), forcing reset"));
          encoder.write(0);
          delay(100);
        }
        
        autoMode = true;
        systemEnabled = true;
        currentState = IDLE;
        targetPosition = 0;
        errorIntegral = 0;
        lastError = 0;
        voltageBoost = 0;
        lastControlTime = millis();
        stuckCheckTime = millis();
        stuckCheckPosition = 0;
        
        Serial.println(F("AUTO ACTIVE\n"));
      } else {
        Serial.println(F("✗ Homing failed\n"));
      }
      break;
      
    case 'S':
      Serial.println(F("\n⏹ STOP\n"));
      autoMode = false;
      systemEnabled = false;
      stopMotor();
      errorIntegral = 0;
      voltageBoost = 0;
      break;
      
    case 'Z':
      Serial.println(F("\n=== HOMING ==="));
      autoMode = false;
      systemEnabled = false;
      stopMotor();
      
      if (homeToLeftLimit()) {
        Serial.println(F("✓ Done\n"));
      }
      break;
      
    case 'R':
      calibrateRange();
      break;
      
    case 'F':
      calibrateFriction();
      break;
      
    case 'C':
      manualCalibration();
      break;
      
    case 'D':
      printDetailedStatus();
      break;
      
    case 'M':
      continuousMonitor();
      break;
      
    case '1':
    case '2':
    case '3':
    case '4': {
      if (autoMode) break;
      
      int lane = cmd - '1';
      long targetPos = getLanePosition(lane);
      
      Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
      Serial.print(F("LANE "));
      Serial.println(lane + 1);
      Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
      Serial.print(F("Start: "));
      Serial.println(encoder.read());
      Serial.print(F("Target: "));
      Serial.println(targetPos);
      
      targetPosition = targetPos;
      errorIntegral = 0;
      lastError = 0;
      voltageBoost = 0;
      stuckCheckPosition = encoder.read();
      stuckCheckTime = millis();
      
      systemEnabled = true;
      currentState = MOVE_TO_TARGET;
      lastControlTime = millis();
      
      unsigned long moveStart = millis();
      unsigned long lastPrint = 0;
      
      while (abs(encoder.read() - targetPosition) > 0) {
        runPIDControl();
        delay(10);
        
        if (millis() - lastPrint > 300) {
          long pos = encoder.read();
          long err = targetPosition - pos;
          Serial.print(F("  "));
          Serial.print(pos);
          Serial.print(F(" / "));
          Serial.print(targetPosition);
          Serial.print(F(" / E:"));
          Serial.print((int)err);
          if (voltageBoost > 0.1) {
            Serial.print(F(" +"));
            Serial.print(voltageBoost, 1);
            Serial.print(F("V"));
          }
          Serial.println();
          lastPrint = millis();
        }
        
        if (millis() - moveStart > 10000) {
          Serial.println(F("\n⚠ TIMEOUT"));
          break;
        }
      }
      
      stopMotor();
      systemEnabled = false;
      voltageBoost = 0;
      
      long finalPos = encoder.read();
      long finalError = targetPosition - finalPos;
      
      Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
      if (finalError == 0) {
        Serial.println(F("✓ ZERO ERROR"));
      } else {
        Serial.println(F("✗ INCOMPLETE"));
      }
      Serial.print(F("Final: "));
      Serial.println(finalPos);
      Serial.print(F("Target: "));
      Serial.println(targetPosition);
      Serial.print(F("Error: "));
      Serial.println((int)finalError);
      Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
      break;
    }
      
    case 'H':
      printHelp();
      break;
  }
}

void printStatus() {
  long pos = encoder.read();
  long error = targetPosition - pos;
  
  Serial.print(F("P:"));
  Serial.print(pos);
  Serial.print(F("→"));
  Serial.print(targetPosition);
  Serial.print(F(" E:"));
  Serial.print((int)error);
  
  if (voltageBoost > 0.1) {
    Serial.print(F(" +"));
    Serial.print(voltageBoost, 1);
    Serial.print(F("V"));
  }
  
  if (activeTargetLane >= 0) {
    Serial.print(F(" L"));
    Serial.print(activeTargetLane + 1);
  }
  
  Serial.print(F(" ["));
  for (int i = 0; i < 4; i++) {
    if (targets[i].exists) {
      Serial.print(i + 1);
      Serial.print(targets[i].movingForward ? F("↑") : F("↓"));
    } else {
      Serial.print(F("-"));
    }
  }
  Serial.println(F("]"));
}

void printDetailedStatus() {
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("STATUS"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  Serial.print(F("Mode: "));
  Serial.println(autoMode ? F("AUTO") : F("MANUAL"));
  Serial.print(F("Pos: "));
  Serial.println(encoder.read());
  Serial.print(F("Target: "));
  Serial.println(targetPosition);
  Serial.print(F("Error: "));
  Serial.println((int)(targetPosition - encoder.read()));
  Serial.print(F("Boost: "));
  Serial.print(voltageBoost, 2);
  Serial.println(F("V"));
  
  Serial.println(F("\nRange:"));
  Serial.print(F("  "));
  Serial.print(LEFT_LIMIT);
  Serial.print(F(" to "));
  Serial.print(RIGHT_LIMIT);
  Serial.print(F(" "));
  Serial.println(rangeCalibrated ? F("[CAL]") : F("[?]"));
  
  Serial.println(F("\nLanes:"));
  Serial.print(F("  1:")); Serial.println(LANE_1_POSITION);
  Serial.print(F("  2:")); Serial.println(LANE_2_POSITION);
  Serial.print(F("  3:")); Serial.println(LANE_3_POSITION);
  Serial.print(F("  4:")); Serial.println(LANE_4_POSITION);
  
  Serial.println(F("\nFriction:"));
  Serial.print(F("  L:")); Serial.print(FRICTION_LEFT, 2);
  Serial.print(F("V (→neg) R:")); Serial.print(FRICTION_RIGHT, 2);
  Serial.print(F("V (→0) "));
  Serial.println(frictionCalibrated ? F("[CAL]") : F("[?]"));
  
  Serial.println(F("\nPID:"));
  Serial.print(F("  Kp=")); Serial.print(KP, 3);
  Serial.print(F(" Ki=")); Serial.print(KI, 3);
  Serial.print(F(" Kd=")); Serial.println(KD, 3);
  
  Serial.println(F("\nProximity:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(targets[i].rawValue);
    Serial.print(F(" ["));
    Serial.print(targets[i].minObserved);
    Serial.print(F("-"));
    Serial.print(targets[i].maxObserved);
    Serial.print(F("] "));
    Serial.print(targets[i].exists ? F("Y") : F("N"));
    Serial.print(F(" "));
    Serial.println(targets[i].movingForward ? F("FWD") : F("back"));
  }
  
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void continuousMonitor() {
  Serial.println(F("\n=== MONITOR ===\n"));
  
  autoMode = false;
  systemEnabled = false;
  stopMotor();
  
  while (!Serial.available()) {
    Serial.print(F("Enc:"));
    Serial.print(encoder.read());
    Serial.print(F(" L:"));
    Serial.print(leftPressed() ? "1" : "0");
    Serial.print(F(" R:"));
    Serial.print(rightPressed() ? "1" : "0");
    Serial.print(F(" P:"));
    
    for (int i = 0; i < 4; i++) {
      Serial.print(analogRead(PROX_SENSOR_1 + i));
      Serial.print(F(" "));
    }
    
    Serial.println();
    delay(100);
  }
  
  while (Serial.available()) Serial.read();
  Serial.println(F("\n✓ Done\n"));
}

void printHelp() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("COMMANDS"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("  Z - Home"));
  Serial.println(F("  R - Cal range"));
  Serial.println(F("  F - Cal friction"));
  Serial.println(F("  C - Cal lanes"));
  Serial.println(F("  A - Auto"));
  Serial.println(F("  S - Stop"));
  Serial.println(F("  1-4 - Move"));
  Serial.println(F("  D - Status"));
  Serial.println(F("  M - Monitor"));
  Serial.println(F("  H - Help"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}
