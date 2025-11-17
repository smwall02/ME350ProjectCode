// ============================================================================
// ME 350 - Plants vs Zombies - COMPETITION CODE - IMPROVED
// Enhanced target switching and zombie tracking
// ============================================================================

#include <Encoder.h>

// ============================================
// PIN DEFINITIONS
// ============================================
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

Encoder encoder(ENCODER_A, ENCODER_B);

// ============================================
// STATE MACHINE DEFINITIONS
// ============================================
enum State {
  CALIBRATE = 1,
  FIND_RANGE = 2,
  CHOOSE_ACTIVE_TARGET = 3,
  MOVE_TO_TARGET = 4
};

State currentState = CALIBRATE;

// Direction constants for zombie movement
const int FORWARD = 1;   // Moving toward photosensor (threat!)
const int BACKWARD = -1; // Moving away from photosensor
const int STOPPED = 0;   // Not moving

// ============================================
// TARGET POSITION CALIBRATION
// ============================================
long TARGET_1_POSITION = -74;
long TARGET_2_POSITION = -307;
long TARGET_3_POSITION = -547;
long TARGET_4_POSITION = -1080;

const long WAIT_POSITION_OFFSET = 2;
long WAIT_POSITION = TARGET_3_POSITION;
long LOWER_BOUND = 0;
long UPPER_BOUND = -1400;

long targetPositions[4] = {
  TARGET_1_POSITION,
  TARGET_2_POSITION,
  TARGET_3_POSITION,
  TARGET_4_POSITION
};

// ============================================
// DYNAMIC SENSOR CALIBRATION
// ============================================
int ProxRange[4][2] = {
  {800, 100},
  {800, 100},
  {800, 100},
  {800, 100}
};

bool sensorCalibrated = false;
bool dynamicCalibrationActive = false;
bool rangeFindingComplete = false;
unsigned long calibrationStartTime = 0;
const unsigned long DYNAMIC_CALIBRATION_TIME = 10000;

int dynamicMin[4];
int dynamicMax[4];

// ============================================
// PROXIMITY SENSOR PROCESSING
// ============================================
struct ProximitySensor {
  float currVal;
  float prevVal;
  unsigned long prevChangeTime;
  int pin;
  int direction;
  int prevDirection;  // NEW: Track previous direction
  int forwardCount;
  int backwardCount;
};

ProximitySensor ProxSensors[4];

// Sensor filtering parameters
const float alpha = 0.925;
const int stopTimeout = 250;
const int lowerNoiseLimit = 5;
const int upperNoiseLimit = 8;
const int noiseThreshold = 225;
int noiseLimit = 8;

// Target selection
int activeTargetIndex = -1;
int previousTargetIndex = -1;  // NEW: Track previous target
long activeTargetPosition = WAIT_POSITION;
float closestZombieDist = 2.0;
float zombieDistances[4];
bool WAIT_POS = true;

// ============================================
// FRICTION COMPENSATION
// ============================================
float FRICTION_COMP_VOLTAGE = 2.2;
float FRICTION_BIAS = 0.25;

float adaptiveFrictionVoltage = 0;
long lastAdaptivePosition = 0;
unsigned long adaptiveStartTime = 0;
bool adaptiveLearning = false;
bool adaptiveLearned = false;

// ============================================
// PID CONTROL PARAMETERS
// ============================================
float KP = 0.020;
float KI = 0.005;
float KD = 0.004;

float KP_active = KP;
float KI_active = KI;
float KD_active = KD;

const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 0.8;
const int TARGET_BAND = 5;
const float MAX_INTEGRAL = 1200.0;
const unsigned long CONTROL_PERIOD = 10;

// ============================================
// MOTION CONTROL STATE
// ============================================
long desiredPosition = 0;
float errorIntegral = 0;
float lastError = 0;
float motorVelocity = 0;
int previousMotorPosition = 0;
long previousVelCompTime = 0;

const int MIN_VEL_COMP_COUNT = 2;
const long MIN_VEL_COMP_TIME = 10000;

unsigned long lastControlTime = 0;
unsigned long lastSensorTime = 0;
unsigned long arrivalTime = 0;
const int targetActivateTime = 350;

// ============================================
// SYSTEM STATE
// ============================================
bool systemEnabled = false;
bool autoMode = false;
unsigned long lastPrintTime = 0;
unsigned long moveStartTime = 0;
bool targetReached = false;

const float CALIBRATION_VOLTAGE = 4.0;
const float HOMING_VOLTAGE = 4.0;
const float RANGE_FINDING_VOLTAGE = -3.5;

// NEW: Target success detection
unsigned long targetHitTime = 0;
const unsigned long MIN_HIT_TIME = 150;  // Minimum time to confirm hit

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);
  
  pinMode(PROX_SENSOR_1, INPUT);
  pinMode(PROX_SENSOR_2, INPUT);
  pinMode(PROX_SENSOR_3, INPUT);
  pinMode(PROX_SENSOR_4, INPUT);
  
  ProxSensors[0].pin = PROX_SENSOR_1;
  ProxSensors[1].pin = PROX_SENSOR_2;
  ProxSensors[2].pin = PROX_SENSOR_3;
  ProxSensors[3].pin = PROX_SENSOR_4;
  
  for (int i = 0; i < 4; i++) {
    ProxSensors[i].currVal = analogRead(ProxSensors[i].pin);
    ProxSensors[i].prevVal = ProxSensors[i].currVal;
    ProxSensors[i].prevChangeTime = millis();
    ProxSensors[i].direction = STOPPED;
    ProxSensors[i].prevDirection = STOPPED;
    ProxSensors[i].forwardCount = 0;
    ProxSensors[i].backwardCount = 0;
  }
  
  stopMotor();
  delay(500);
  
  printWelcome();
  printHelp();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  if (Serial.available() > 0) {
    processCommand();
  }
  
  if (currentTime - lastSensorTime >= 20) {
    lastSensorTime = currentTime;
    updateAllSensors();
    
    if (dynamicCalibrationActive) {
      updateDynamicCalibration();
    }
  }
  
  if (currentTime - lastControlTime >= CONTROL_PERIOD) {
    lastControlTime = currentTime;
    
    updateVelocity();
    
    if (autoMode && !dynamicCalibrationActive) {
      runStateMachine();
    }
    
    if (autoMode || systemEnabled) {
      runMotionControl();
    }
    
    if (autoMode && (currentTime - lastPrintTime >= 500)) {
      lastPrintTime = currentTime;
      
      if (dynamicCalibrationActive) {
        printCalibrationProgress();
      } else {
        printCompactStatus();
      }
    }
  }
  
  checkLimitSwitches();
}

// ============================================
// SENSOR UPDATE
// ============================================
void updateAllSensors() {
  for (int i = 0; i < 4; i++) {
    // Store previous direction
    ProxSensors[i].prevDirection = ProxSensors[i].direction;
    
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal + 
                             (1.0 - alpha) * analogRead(ProxSensors[i].pin);
    
    if (ProxSensors[i].currVal >= noiseThreshold) {
      noiseLimit = upperNoiseLimit;
    } else {
      noiseLimit = lowerNoiseLimit;
    }
    
    if (abs(ProxSensors[i].currVal - ProxSensors[i].prevVal) < noiseLimit) {
      if (millis() - ProxSensors[i].prevChangeTime >= stopTimeout) {
        ProxSensors[i].direction = STOPPED;
      }
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].backwardCount = 0;
      
    } else if (ProxSensors[i].currVal - ProxSensors[i].prevVal < 0) {
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;
      
      if (ProxSensors[i].forwardCount > 3) {
        ProxSensors[i].direction = FORWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = millis();
      }
      
    } else {
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      
      if (ProxSensors[i].backwardCount > 3) {
        ProxSensors[i].direction = BACKWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = millis();
      }
    }
  }
}

// ============================================
// VELOCITY COMPUTATION
// ============================================
void updateVelocity() {
  long currentPos = encoder.read();
  long deltaPos = currentPos - previousMotorPosition;
  long deltaTime = micros() - previousVelCompTime;
  
  if (abs(deltaPos) > MIN_VEL_COMP_COUNT || deltaTime > MIN_VEL_COMP_TIME) {
    motorVelocity = (double)deltaPos * 1000000.0 / deltaTime;
    previousMotorPosition = currentPos;
    previousVelCompTime = micros();
  }
}

// ============================================
// RANGE FINDING
// ============================================
bool findEncoderRange() {
  Serial.println(F("\n📏 FINDING ENCODER RANGE..."));
  Serial.println(F("Moving to right limit...\n"));
  
  unsigned long startTime = millis();
  setMotor(RANGE_FINDING_VOLTAGE);
  
  long lastPos = encoder.read();
  unsigned long stuckTime = 0;
  
  while ((millis() - startTime) < 15000) {
    delay(50);
    
    long currentPos = encoder.read();
    
    if (digitalRead(LIMIT_RIGHT) == HIGH) {
      stopMotor();
      delay(200);
      
      UPPER_BOUND = encoder.read();
      
      Serial.print(F("✓ Found right limit at: "));
      Serial.println(UPPER_BOUND);
      
      Serial.println(F("Returning to home position...\n"));
      delay(500);
      
      if (homeToLeftLimit()) {
        Serial.print(F("✓ Range found: "));
        Serial.print(LOWER_BOUND);
        Serial.print(F(" to "));
        Serial.println(UPPER_BOUND);
        Serial.print(F("Total travel: "));
        Serial.print(abs(UPPER_BOUND - LOWER_BOUND));
        Serial.println(F(" counts\n"));
        return true;
      }
    }
    
    if (abs(currentPos - lastPos) < 5) {
      stuckTime += 50;
      if (stuckTime > 2000) {
        stopMotor();
        Serial.println(F("⚠️  Movement stopped, assuming limit reached"));
        UPPER_BOUND = encoder.read() + 20;
        
        if (homeToLeftLimit()) {
          return true;
        }
      }
    } else {
      stuckTime = 0;
      lastPos = currentPos;
    }
  }
  
  stopMotor();
  Serial.println(F("✗ Range finding timeout\n"));
  return false;
}

// ============================================
// DYNAMIC SENSOR CALIBRATION
// ============================================
void startDynamicCalibration() {
  Serial.println(F("\n📊 STARTING DYNAMIC SENSOR CALIBRATION..."));
  Serial.println(F("Observing sensor ranges for 10 seconds..."));
  Serial.println(F("Targets should be moving during this time!\n"));
  
  dynamicCalibrationActive = true;
  calibrationStartTime = millis();
  
  for (int i = 0; i < 4; i++) {
    dynamicMin[i] = 1023;
    dynamicMax[i] = 0;
  }
}

void updateDynamicCalibration() {
  if (!dynamicCalibrationActive) return;
  
  for (int i = 0; i < 4; i++) {
    int reading = analogRead(ProxSensors[i].pin);
    
    if (reading < dynamicMin[i]) {
      dynamicMin[i] = reading;
    }
    if (reading > dynamicMax[i]) {
      dynamicMax[i] = reading;
    }
  }
  
  if (millis() - calibrationStartTime >= DYNAMIC_CALIBRATION_TIME) {
    finalizeDynamicCalibration();
  }
}

void finalizeDynamicCalibration() {
  dynamicCalibrationActive = false;
  
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("DYNAMIC CALIBRATION COMPLETE!"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  bool allValid = true;
  
  for (int i = 0; i < 4; i++) {
    int range = dynamicMax[i] - dynamicMin[i];
    
    Serial.print(F("Lane "));
    Serial.print(i + 1);
    Serial.print(F(": MIN="));
    Serial.print(dynamicMin[i]);
    Serial.print(F(", MAX="));
    Serial.print(dynamicMax[i]);
    Serial.print(F(", Range="));
    Serial.print(range);
    
    if (range < 50) {
      Serial.println(F(" ⚠️  WARNING: Small range!"));
      allValid = false;
    } else {
      Serial.println(F(" ✓"));
    }
    
    ProxRange[i][0] = dynamicMax[i];
    ProxRange[i][1] = dynamicMin[i];
  }
  
  if (allValid) {
    sensorCalibrated = true;
    Serial.println(F("\n✓ Sensors calibrated successfully!"));
    Serial.println(F("Starting target tracking...\n"));
  } else {
    Serial.println(F("\n⚠️  Some sensors may not have seen full range."));
    Serial.println(F("Proceeding anyway...\n"));
    sensorCalibrated = true;
  }
}

void printCalibrationProgress() {
  unsigned long elapsed = millis() - calibrationStartTime;
  unsigned long remaining = DYNAMIC_CALIBRATION_TIME - elapsed;
  
  Serial.print(F("📊 Calibrating... "));
  Serial.print(remaining / 1000);
  Serial.print(F("s left | Ranges: "));
  
  for (int i = 0; i < 4; i++) {
    int range = dynamicMax[i] - dynamicMin[i];
    Serial.print(range);
    Serial.print(F(" "));
  }
  Serial.println();
}

// ============================================
// STATE MACHINE - IMPROVED
// ============================================
void runStateMachine() {
  switch (currentState) {
    
    case CALIBRATE:
      desiredPosition = LOWER_BOUND;
      
      if (!dynamicCalibrationActive && rangeFindingComplete && sensorCalibrated) {
        Serial.println(F("State: CALIBRATE → CHOOSE_ACTIVE_TARGET (tracking enabled)\n"));
        currentState = CHOOSE_ACTIVE_TARGET;
        systemEnabled = true;
      }
      else if (!dynamicCalibrationActive && !rangeFindingComplete) {
        Serial.println(F("State: CALIBRATE → FIND_RANGE\n"));
        currentState = FIND_RANGE;
      }
      break;
    
    case FIND_RANGE:
      if (!rangeFindingComplete) {
        stopMotor();
        systemEnabled = false;
        
        if (findEncoderRange()) {
          rangeFindingComplete = true;
          startDynamicCalibration();
          desiredPosition = LOWER_BOUND;
          
          Serial.println(F("State: FIND_RANGE → CALIBRATE (sensor cal)\n"));
          currentState = CALIBRATE;
          systemEnabled = true;
        } else {
          Serial.println(F("✗ Range finding failed, stopping\n"));
          autoMode = false;
          systemEnabled = false;
        }
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      activeTargetIndex = -1;
      closestZombieDist = 2.0;
      
      for (int i = 0; i < 4; i++) {
        zombieDistances[i] = (ProxSensors[i].currVal - ProxRange[i][1]) / 
                             (float)(ProxRange[i][0] - ProxRange[i][1]);
        
        zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);
        
        // IMPROVED: Only target zombies moving FORWARD
        // Lower distance = closer to photo sensor = more dangerous
        if (ProxSensors[i].direction == FORWARD && 
            zombieDistances[i] < closestZombieDist) {
          closestZombieDist = zombieDistances[i];
          activeTargetIndex = i;
        }
      }
      
      if (activeTargetIndex >= 0) {
        activeTargetPosition = targetPositions[activeTargetIndex];
        WAIT_POS = false;
        
        int percentToPhoto = (int)((1.0 - zombieDistances[activeTargetIndex]) * 100);
        
        Serial.print(F("🎯 Target: Lane "));
        Serial.print(activeTargetIndex + 1);
        Serial.print(F(" ("));
        Serial.print(percentToPhoto);
        Serial.print(F("% to photo = DANGER!, dist="));
        Serial.print(zombieDistances[activeTargetIndex], 2);
        Serial.println(F(")"));
        
        previousTargetIndex = activeTargetIndex;
        targetHitTime = 0;
        
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        Serial.println(F("No active FORWARD targets, moving to wait position"));
      }
      
      desiredPosition = activeTargetPosition;
      moveStartTime = millis();
      arrivalTime = millis();
      targetReached = false;
      currentState = MOVE_TO_TARGET;
      break;
    
    case MOVE_TO_TARGET:
      desiredPosition = activeTargetPosition;
      
      long currentPos = encoder.read();
      long error = desiredPosition - currentPos;
      
      // Safety check - approaching right limit
      if (currentPos < UPPER_BOUND - 50) {
        Serial.println(F("⚠️  Approaching right limit, returning to safe zone"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
      // IMPROVED: Check if target zombie changed direction or stopped
      if (activeTargetIndex >= 0 && !WAIT_POS) {
        int targetDirection = ProxSensors[activeTargetIndex].direction;
        int prevDirection = ProxSensors[activeTargetIndex].prevDirection;
        
        // If zombie was FORWARD and now is BACKWARD or STOPPED, we hit it!
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
          
          if (targetHitTime == 0) {
            targetHitTime = millis();
          }
          
          // Confirm the hit for MIN_HIT_TIME before switching
          if (millis() - targetHitTime >= MIN_HIT_TIME) {
            Serial.println(F("✓ Target HIT! Zombie moving backward/stopped, choosing next"));
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        } else {
          targetHitTime = 0;  // Reset if not consistently backward
        }
        
        // IMPROVED: Also switch if zombie distance is increasing (moving away)
        // This means we successfully pushed it back
        if (targetDirection == BACKWARD && 
            zombieDistances[activeTargetIndex] < 0.15) {
          Serial.println(F("✓ Target retreating far enough, choosing next"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }
      }
      
      // Standard arrival check
      if (abs(error) <= TARGET_BAND) {
        if (millis() - arrivalTime > targetActivateTime || WAIT_POS) {
          Serial.println(F("✓ Target activated (timeout), choosing next"));
          currentState = CHOOSE_ACTIVE_TARGET;
        }
      } else {
        arrivalTime = millis();
      }
      break;
  }
}

// ============================================
// MOTION CONTROL
// ============================================
void runMotionControl() {
  long currentPosition = encoder.read();
  float error = desiredPosition - currentPosition;
  
  if (currentState == MOVE_TO_TARGET && autoMode) {
    if (currentPosition < UPPER_BOUND + 30) {
      stopMotor();
      Serial.println(F("⚠️  Too close to right limit!"));
      desiredPosition = WAIT_POSITION;
      currentState = CHOOSE_ACTIVE_TARGET;
      return;
    }
    
    if (currentPosition > 50) {
      Serial.println(F("⚠️  Position drift detected - need recalibration"));
      stopMotor();
      return;
    }
  }
  
  if (dynamicCalibrationActive) {
    desiredPosition = LOWER_BOUND;
    error = desiredPosition - currentPosition;
    
    if (abs(error) > 10) {
      float correctionVoltage = constrain(error * 0.05, -2.0, 2.0);
      setMotor(correctionVoltage);
    } else {
      stopMotor();
    }
    return;
  }
  
  if (abs(error) <= TARGET_BAND) {
    stopMotor();
    errorIntegral = 0;
    adaptiveLearning = false;
    
    if (!targetReached) {
      targetReached = true;
      unsigned long settleTime = millis() - moveStartTime;
      Serial.print(F("✓ Reached "));
      Serial.print(currentPosition);
      Serial.print(F(" in "));
      Serial.print(settleTime / 1000.0, 2);
      Serial.println(F("s"));
    }
    return;
  }
  
  targetReached = false;
  
  if (abs(error) > 500 && !adaptiveLearning && !adaptiveLearned) {
    adaptiveLearning = true;
    adaptiveFrictionVoltage = 3.0;
    lastAdaptivePosition = currentPosition;
    adaptiveStartTime = millis();
  }
  
  if (adaptiveLearning) {
    if (abs(currentPosition - lastAdaptivePosition) > 10) {
      Serial.print(F("  ✓ Learned friction: "));
      Serial.print(adaptiveFrictionVoltage, 2);
      Serial.println(F("V"));
      adaptiveLearning = false;
      adaptiveLearned = true;
    } else if (millis() - adaptiveStartTime > 200) {
      adaptiveFrictionVoltage += 0.5;
      adaptiveStartTime = millis();
      lastAdaptivePosition = currentPosition;
      
      if (adaptiveFrictionVoltage > 5.5) {
        adaptiveLearning = false;
        adaptiveLearned = true;
        adaptiveFrictionVoltage = 3.5;
      }
    }
    
    float voltage = (error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage;
    setMotor(voltage);
    return;
  }
  
  float dt = CONTROL_PERIOD / 1000.0;
  
  if (abs(error) > 300) {
    KP_active = KP * 1.8;
    KI_active = 0;
    KD_active = KD * 0.5;
    errorIntegral = 0;
  } else if (abs(error) > 50) {
    KP_active = KP * 1.3;
    KI_active = KI * 0.5;
    KD_active = KD;
  } else {
    KP_active = KP;
    KI_active = KI;
    KD_active = KD;
    
    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  }
  
  float errorDerivative = (error - lastError) / dt;
  
  float pidVoltage = (KP_active * error) + 
                     (KI_active * errorIntegral) + 
                     (KD_active * errorDerivative);
  
  float frictionComp = 0;
  if (abs(error) > TARGET_BAND) {
    float frictionScale = 1.0;
    
    if (abs(error) < 3) {
      frictionScale = 0.1;
    } else if (abs(error) < 10) {
      frictionScale = 0.3;
    } else if (abs(error) < 30) {
      frictionScale = 0.6;
    } else if (abs(error) < 100) {
      frictionScale = 0.85;
    }
    
    if (abs(motorVelocity) < 5) {
      frictionScale *= 0.5;
    }
    
    float baseFriction = (adaptiveFrictionVoltage > 0) ? 
                         adaptiveFrictionVoltage : FRICTION_COMP_VOLTAGE;
    
    if (error < 0) {
      frictionComp = -(baseFriction + FRICTION_BIAS) * frictionScale;
    } else {
      frictionComp = baseFriction * frictionScale;
    }
  }
  
  float velocityFF = 0;
  if (abs(error) > 50) {
    float desiredVelocity = constrain(error / 0.15, -400, 400);
    velocityFF = 0.008 * desiredVelocity;
  }
  
  float totalVoltage = pidVoltage + frictionComp + velocityFF;
  totalVoltage = constrain(totalVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);
  
  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
  }
  
  lastError = error;
}

// ============================================
// MOTOR CONTROL
// ============================================
void setMotor(float voltage) {
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

// ============================================
// LIMIT SWITCH SAFETY
// ============================================
void checkLimitSwitches() {
  if (digitalRead(LIMIT_LEFT) == HIGH && 
      currentState != CALIBRATE && 
      currentState != FIND_RANGE &&
      !dynamicCalibrationActive &&
      abs(motorVelocity) < 10) {
    
    delay(50);
    encoder.write(0);
    delay(30);
    
    if (encoder.read() != 0) {
      encoder.write(0);
      delay(30);
    }
    
    if (encoder.read() != 0) {
      encoder.write(0);
    }
    
    errorIntegral = 0;
    Serial.println(F("⚠️  Recalibrated at left limit"));
  }
  
  if (digitalRead(LIMIT_RIGHT) == HIGH) {
    stopMotor();
    Serial.println(F("⚠️  RIGHT LIMIT HIT - EMERGENCY STOP!"));
    
    if (autoMode && currentState != FIND_RANGE) {
      Serial.println(F("⚠️  Unexpected right limit hit, returning to safe zone"));
      delay(500);
      
      setMotor(2.0);
      delay(500);
      stopMotor();
      
      desiredPosition = LOWER_BOUND;
      currentState = CHOOSE_ACTIVE_TARGET;
    }
  }
}

bool leftPressed() {
  return digitalRead(LIMIT_LEFT) == HIGH;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == HIGH;
}

// ============================================
// HOMING
// ============================================
bool homeToLeftLimit() {
  Serial.println(F("\n🏠 HOMING..."));
  
  if (leftPressed()) {
    Serial.println(F("Already at limit, ensuring contact..."));
    setMotor(HOMING_VOLTAGE);
    delay(400);
    stopMotor();
    delay(100);
    
    encoder.write(0);
    delay(50);
    
    if (encoder.read() != 0) {
      encoder.write(0);
      delay(50);
    }
    
    if (encoder.read() != 0) {
      encoder.write(0);
      delay(50);
    }
    
    Serial.print(F("Zeroed at: "));
    Serial.println(encoder.read());
    Serial.println(F("✓ Homed\n"));
    return true;
  }
  
  unsigned long startTime = millis();
  setMotor(HOMING_VOLTAGE);
  
  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }
  
  if (leftPressed()) {
    Serial.println(F("Contact made, holding..."));
    delay(300);
    stopMotor();
    delay(100);
    
    encoder.write(0);
    delay(50);
    
    if (encoder.read() != 0) {
      Serial.println(F("  Re-zeroing..."));
      encoder.write(0);
      delay(50);
    }
    
    if (encoder.read() != 0) {
      Serial.println(F("  Re-zeroing again..."));
      encoder.write(0);
      delay(50);
    }
    
    Serial.print(F("Final encoder value: "));
    Serial.println(encoder.read());
    Serial.println(F("✓ Homed\n"));
    return true;
  } else {
    stopMotor();
    Serial.println(F("✗ Timeout\n"));
    return false;
  }
}

// ============================================
// MANUAL TARGET POSITION CALIBRATION
// ============================================
void manualCalibration() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   MANUAL TARGET POSITION CALIBRATION       ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
  Serial.println(F("Controls:"));
  Serial.println(F("  R = Move Right"));
  Serial.println(F("  L = Move Left"));
  Serial.println(F("  S = Stop"));
  Serial.println(F("  1-4 = Save current position as target"));
  Serial.println(F("  Q = Quit\n"));
  
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();
      
      cmd = toupper(cmd);
      
      long pos = encoder.read();
      
      switch (cmd) {
        case 'R':
          setMotor(-3.5);
          Serial.println(F("Moving RIGHT..."));
          break;
          
        case 'L':
          setMotor(3.5);
          Serial.println(F("Moving LEFT..."));
          break;
          
        case 'S':
          stopMotor();
          Serial.print(F("Stopped at: "));
          Serial.println(pos);
          break;
          
        case '1':
          stopMotor();
          TARGET_1_POSITION = pos;
          targetPositions[0] = pos;
          Serial.print(F("✓ Lane 1 = "));
          Serial.println(TARGET_1_POSITION);
          break;
          
        case '2':
          stopMotor();
          TARGET_2_POSITION = pos;
          targetPositions[1] = pos;
          Serial.print(F("✓ Lane 2 = "));
          Serial.println(TARGET_2_POSITION);
          break;
          
        case '3':
          stopMotor();
          TARGET_3_POSITION = pos;
          targetPositions[2] = pos;
          WAIT_POSITION = TARGET_3_POSITION;
          Serial.print(F("✓ Lane 3 = "));
          Serial.println(TARGET_3_POSITION);
          break;
          
        case '4':
          stopMotor();
          TARGET_4_POSITION = pos;
          targetPositions[3] = pos;
          Serial.print(F("✓ Lane 4 = "));
          Serial.println(TARGET_4_POSITION);
          break;
          
        case 'Q':
          stopMotor();
          Serial.println(F("\n✓ Calibration complete\n"));
          Serial.println(F("Target positions:"));
          Serial.print(F("  1: "));
          Serial.println(TARGET_1_POSITION);
          Serial.print(F("  2: "));
          Serial.println(TARGET_2_POSITION);
          Serial.print(F("  3: "));
          Serial.println(TARGET_3_POSITION);
          Serial.print(F("  4: "));
          Serial.println(TARGET_4_POSITION);
          Serial.println();
          systemEnabled = wasEnabled;
          autoMode = wasAuto;
          return;
          
        default:
          break;
      }
    }
    
    checkLimitSwitches();
    delay(50);
  }
}

// ============================================
// COMMAND PROCESSING
// ============================================
void processCommand() {
  char cmd = Serial.read();
  while (Serial.available()) Serial.read();
  
  cmd = toupper(cmd);
  
  switch (cmd) {
    case 'G':
      if (!autoMode) {
        Serial.println(F("\n🎮 STARTING AUTONOMOUS MODE"));
        Serial.println(F("Sequence: Home → Find Range → Sensor Cal → Track\n"));
        
        if (homeToLeftLimit()) {
          autoMode = true;
          systemEnabled = true;
          rangeFindingComplete = false;
          sensorCalibrated = false;
          
          currentState = CALIBRATE;
          errorIntegral = 0;
          lastPrintTime = 0;
          
          Serial.println(F("✓ HOMING COMPLETE"));
          Serial.println(F("Next: Finding encoder range...\n"));
        } else {
          Serial.println(F("✗ Homing failed\n"));
        }
      }
      break;
    
    case 'S':
      Serial.println(F("\n⏹ STOP\n"));
      autoMode = false;
      systemEnabled = false;
      dynamicCalibrationActive = false;
      stopMotor();
      errorIntegral = 0;
      break;
    
    case '1':
    case '2':
    case '3':
    case '4':
      if (!autoMode) {
        setTargetLane(cmd - '0');
        systemEnabled = true;
        lastPrintTime = 0;
      } else {
        Serial.println(F("\n⚠️  Stop autonomous mode first (press 'S')\n"));
      }
      break;
    
    case 'P':
      printStatus();
      break;
    
    case 'H':
      printHelp();
      break;
    
    case 'Z':
      homeToLeftLimit();
      break;
    
    case 'D':
      printAllSensors();
      break;
    
    case 'C':
      manualCalibration();
      break;
    
    case 'M':
      continuousMonitor();
      break;
    
    case 'R':
      adaptiveFrictionVoltage = 0;
      adaptiveLearned = false;
      rangeFindingComplete = false;
      sensorCalibrated = false;
      Serial.println(F("\n✓ Reset all calibrations\n"));
      break;
    
    default:
      break;
  }
}

void setTargetLane(int lane) {
  if (lane < 1 || lane > 4) return;
  
  desiredPosition = targetPositions[lane - 1];
  errorIntegral = 0;
  lastError = 0;
  moveStartTime = millis();
  targetReached = false;
  
  long currentPos = encoder.read();
  long error = desiredPosition - currentPos;
  
  Serial.print(F("\n→ Lane "));
  Serial.print(lane);
  Serial.print(F(" | "));
  Serial.print(currentPos);
  Serial.print(F(" → "));
  Serial.print(desiredPosition);
  Serial.print(F(" (Δ="));
  Serial.print(error);
  Serial.println(F(")"));
}

// ============================================
// DISPLAY FUNCTIONS
// ============================================
void printWelcome() {
  Serial.println(F("\n\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   PLANTS VS ZOMBIES - IMPROVED CODE        ║"));
  Serial.println(F("║   Enhanced Target Switching                ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("COMMANDS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("  C      🎯 Calibrate target positions"));
  Serial.println(F("  Z      🏠 Home to left limit"));
  Serial.println(F("  G      🎮 Start AUTO (range + sensors + track!)"));
  Serial.println(F("  S      ⏹  Stop"));
  Serial.println(F("  1-4    Manual lane control"));
  Serial.println(F("  P      Status"));
  Serial.println(F("  D      Display all sensors"));
  Serial.println(F("  M      Continuous monitor"));
  Serial.println(F("  R      Reset all calibrations"));
  Serial.println(F("  H      Help"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void printCompactStatus() {
  long pos = encoder.read();
  
  Serial.print(F("State:"));
  switch(currentState) {
    case CALIBRATE: Serial.print(F("CAL")); break;
    case FIND_RANGE: Serial.print(F("RANGE")); break;
    case CHOOSE_ACTIVE_TARGET: Serial.print(F("CHOOSE")); break;
    case MOVE_TO_TARGET: Serial.print(F("MOVE")); break;
  }
  
  Serial.print(F(" | Pos:"));
  Serial.print(pos);
  Serial.print(F("→"));
  Serial.print(desiredPosition);
  Serial.print(F(" | Active:"));
  if (activeTargetIndex >= 0) {
    Serial.print(activeTargetIndex + 1);
  } else {
    Serial.print(F("-"));
  }
  
  Serial.print(F(" | Zombies: "));
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction == FORWARD) {
      Serial.print(F("▶"));
    } else if (ProxSensors[i].direction == BACKWARD) {
      Serial.print(F("◀"));
    } else {
      Serial.print(F("■"));
    }
    
    int percentToPhoto = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(percentToPhoto);
    Serial.print(F("% "));
  }
  Serial.println();
}

void printStatus() {
  long currentPos = encoder.read();
  float error = desiredPosition - currentPos;
  
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("SYSTEM STATUS"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  Serial.print(F("Mode:      "));
  Serial.println(autoMode ? F("AUTONOMOUS") : F("Manual"));
  
  Serial.print(F("State:     "));
  switch(currentState) {
    case CALIBRATE: Serial.println(F("CALIBRATE")); break;
    case FIND_RANGE: Serial.println(F("FIND_RANGE")); break;
    case CHOOSE_ACTIVE_TARGET: Serial.println(F("CHOOSE_ACTIVE_TARGET")); break;
    case MOVE_TO_TARGET: Serial.println(F("MOVE_TO_TARGET")); break;
  }
  
  Serial.print(F("Position:  "));
  Serial.println(currentPos);
  Serial.print(F("Target:    "));
  Serial.println(desiredPosition);
  Serial.print(F("Error:     "));
  Serial.println((int)error);
  Serial.print(F("Velocity:  "));
  Serial.print((int)motorVelocity);
  Serial.println(F(" counts/s"));
  
  Serial.println(F("\n━━ ENCODER RANGE ━━"));
  Serial.print(F("Lower Bound: "));
  Serial.println(LOWER_BOUND);
  Serial.print(F("Upper Bound: "));
  Serial.println(UPPER_BOUND);
  Serial.print(F("Total Travel: "));
  Serial.println(abs(UPPER_BOUND - LOWER_BOUND));
  Serial.print(F("Range Found: "));
  Serial.println(rangeFindingComplete ? "YES" : "NO");
  
  Serial.println(F("\n━━ ACTIVE TARGET ━━"));
  Serial.print(F("Lane:      "));
  if (activeTargetIndex >= 0) {
    Serial.println(activeTargetIndex + 1);
  } else {
    Serial.println(F("None"));
  }
  
  Serial.println(F("\n━━ ZOMBIE STATUS ━━"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("Lane "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print((int)ProxSensors[i].currVal);
    Serial.print(F(" | "));
    
    int percentToPhoto = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(percentToPhoto);
    Serial.print(F("% to photo | "));
    
    switch(ProxSensors[i].direction) {
      case FORWARD: Serial.println(F("FORWARD ⚠️")); break;
      case BACKWARD: Serial.println(F("BACKWARD")); break;
      case STOPPED: Serial.println(F("STOPPED")); break;
    }
  }
  
  Serial.println(F("\n━━ SENSOR CALIBRATION ━━"));
  Serial.print(F("Calibrated: "));
  Serial.println(sensorCalibrated ? F("YES") : F("NO"));
  
  if (sensorCalibrated) {
    Serial.println(F("Ranges:"));
    for (int i = 0; i < 4; i++) {
      Serial.print(F("  Lane "));
      Serial.print(i + 1);
      Serial.print(F(": ["));
      Serial.print(ProxRange[i][1]);
      Serial.print(F("-"));
      Serial.print(ProxRange[i][0]);
      Serial.println(F("]"));
    }
  }
  
  Serial.println(F("\n━━ PID GAINS ━━"));
  Serial.print(F("Kp = "));
  Serial.println(KP, 4);
  Serial.print(F("Ki = "));
  Serial.println(KI, 4);
  Serial.print(F("Kd = "));
  Serial.println(KD, 4);
  
  if (adaptiveFrictionVoltage > 0) {
    Serial.print(F("Learned Friction = "));
    Serial.print(adaptiveFrictionVoltage, 2);
    Serial.println(F("V"));
  }
  
  Serial.println(F("\n━━ TARGET POSITIONS ━━"));
  Serial.print(F("Lane 1: "));
  Serial.println(TARGET_1_POSITION);
  Serial.print(F("Lane 2: "));
  Serial.println(TARGET_2_POSITION);
  Serial.print(F("Lane 3: "));
  Serial.println(TARGET_3_POSITION);
  Serial.print(F("Lane 4: "));
  Serial.println(TARGET_4_POSITION);
  
  Serial.println(F("\n━━ LIMIT SWITCHES ━━"));
  Serial.print(F("Left:  "));
  Serial.println(leftPressed() ? "PRESSED" : "open");
  Serial.print(F("Right: "));
  Serial.println(rightPressed() ? "PRESSED" : "open");
  
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void printAllSensors() {
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("ALL SENSOR DATA"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  long pos = encoder.read();
  Serial.print(F("Encoder:    "));
  Serial.println(pos);
  
  Serial.println(F("\nLimit Switches:"));
  Serial.print(F("  Left:     "));
  Serial.println(leftPressed() ? "PRESSED" : "open");
  Serial.print(F("  Right:    "));
  Serial.println(rightPressed() ? "PRESSED" : "open");
  
  Serial.println(F("\nProximity Sensors (raw):"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Sensor "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(analogRead(ProxSensors[i].pin));
  }
  
  Serial.println(F("\nProximity Sensors (filtered):"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Sensor "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println((int)ProxSensors[i].currVal);
  }
  
  Serial.println(F("\nZombie Distances:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Lane "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    
    int percentToPhoto = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(percentToPhoto);
    Serial.println(F("% to photosensor"));
  }
  
  Serial.println(F("\nSensor Ranges:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Lane "));
    Serial.print(i + 1);
    Serial.print(F(": ["));
    Serial.print(ProxRange[i][1]);
    Serial.print(F(" - "));
    Serial.print(ProxRange[i][0]);
    Serial.print(F("] ("));
    Serial.print(sensorCalibrated ? "calibrated" : "default");
    Serial.println(F(")"));
  }
  
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void continuousMonitor() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   CONTINUOUS MONITOR                       ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.println(F("Press any key to stop\n"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
  while (!Serial.available()) {
    updateAllSensors();
    
    long pos = encoder.read();
    
    Serial.print(F("Enc:"));
    Serial.print(pos);
    Serial.print(F(" | L:"));
    Serial.print(leftPressed() ? "1" : "0");
    Serial.print(F(" R:"));
    Serial.print(rightPressed() ? "1" : "0");
    Serial.print(F(" | Prox: "));
    
    for (int i = 0; i < 4; i++) {
      Serial.print((int)ProxSensors[i].currVal);
      Serial.print(F(" "));
    }
    
    Serial.print(F("| Dir: "));
    for (int i = 0; i < 4; i++) {
      if (ProxSensors[i].direction == FORWARD) Serial.print(F("▶"));
      else if (ProxSensors[i].direction == BACKWARD) Serial.print(F("◀"));
      else Serial.print(F("■"));
      Serial.print(F(" "));
    }
    
    Serial.println();
    delay(100);
  }
  
  while (Serial.available()) Serial.read();
  Serial.println(F("\n✓ Monitor stopped\n"));
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
