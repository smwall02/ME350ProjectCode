// ME 350 - Plants vs Zombies - COMPETITION CODE

#include <Encoder.h>
#include <EEPROM.h>
#include <math.h>

// PIN DEFINITIONS
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_ENA 11
#define MOTOR_IN2 12
#define MOTOR_IN3 13
#define LIMIT_LEFT 8
#define LIMIT_RIGHT 9
#define ON_OFF_SWITCH_PIN 5  // Flip switch for motor enable/disable

#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

Encoder encoder(ENCODER_A, ENCODER_B);

// EEPROM LAYOUT
const int EEPROM_FLAG = 0;
const int EEPROM_KP = 1;
const int EEPROM_KI = 5;
const int EEPROM_KD = 9;
const int EEPROM_FRICTION_LEFT = 13;
const int EEPROM_FRICTION_RIGHT = 17;
const int EEPROM_LEFT_LIMIT = 21;
const int EEPROM_RIGHT_LIMIT = 25;
const int EEPROM_LANES_BASE = 29;  // 4 lanes * 4 bytes each

// STATE MACHINE
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

// TARGET POSITIONS
long TARGET_1_POSITION = -74;
long TARGET_2_POSITION = -307;
long TARGET_3_POSITION = -547;
long TARGET_4_POSITION = -1080;

const long WAIT_POSITION_OFFSET = 2;
long WAIT_POSITION = TARGET_3_POSITION;
long LOWER_BOUND = 0;      // Will be set during range finding
long UPPER_BOUND = -1384;  // Default: Right limit position (will be set during range finding)

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
const unsigned long DYNAMIC_CALIBRATION_TIME = 5000;  // 5 seconds

int dynamicMin[4];
int dynamicMax[4];

// PROXIMITY SENSORS
struct ProximitySensor {
  float currVal;
  float prevVal;  // Used for direction detection
  float prevValForVelocity;  // NEW: Previous value for velocity calculation
  unsigned long prevChangeTime;
  unsigned long lastVelocityUpdate;  // NEW: Separate timestamp for velocity calculation
  int pin;
  int direction;
  int prevDirection;  // Track previous direction
  int forwardCount;
  int backwardCount;
  float velocity;  // NEW: Rate of change (sensor value change per second)
  float prevVelocity;  // NEW: Previous velocity for detecting reversals
  bool hitDetected;  // NEW: Flag for confirmed hit
  unsigned long hitTime;  // NEW: Time when hit was detected
};

ProximitySensor ProxSensors[4];

// Sensor filtering
const float alpha = 0.925;
const int stopTimeout = 150;
const int lowerNoiseLimit = 5;
const int upperNoiseLimit = 8;
const int noiseThreshold = 225;
int noiseLimit = 8;

// Target selection
int activeTargetIndex = -1;
int previousTargetIndex = -1;
long activeTargetPosition = WAIT_POSITION;
float closestZombieDist = 2.0;
float zombieDistances[4];
bool WAIT_POS = true;
unsigned long targetCommitTime = 0;  // Time when target was committed to
const unsigned long MIN_TARGET_COMMIT_TIME = 800;  // Minimum time to stay at target (ms)
const float TARGET_SWITCH_HYSTERESIS = 0.15;  // Must be this much closer to switch
const float MIN_COMMIT_DISTANCE = 0.25;  // If closer than this, commit to target
const float CLOSER_THREAT_THRESHOLD = 0.25;  // Must be this much closer to interrupt

// Fine positioning
long previousMoveStartPosition = 0;
bool fineAdjustmentActive = false;
long fineAdjustmentTarget = 0;
const int FINE_ADJUSTMENT_AMOUNT = 2;
float previousZombieDistance = 1.0;
float lastRetreatCheckDistance = 0.0;
unsigned long lastRetreatCheckTime = 0;
const unsigned long MIN_WAIT_AT_TARGET = 500;  // Minimum time to wait at target before checking retreat
const float RETREAT_DISTANCE_THRESHOLD = 0.20;  // Distance must increase by this much to confirm retreat (increased for stricter detection)
int fineAdjustmentCount = 0;
const int MAX_FINE_ADJUSTMENTS = 5;
unsigned long lastFineAdjustmentTime = 0;
const unsigned long MIN_FINE_ADJUSTMENT_INTERVAL = 50;

// Lane 4 limit switch
bool lane4LimitSwitchMode = false;
bool lane4AtLimit = false;
unsigned long lane4LimitTime = 0;
const unsigned long LANE4_LIMIT_HOLD_TIME = 200;
const int LANE4_BACKOFF_DISTANCE = 20;

// FRICTION COMPENSATION
float FRICTION_LEFT = 0.25;
float FRICTION_RIGHT = 0.25;
float adaptiveFrictionLeft = FRICTION_LEFT;
float adaptiveFrictionRight = FRICTION_RIGHT;
const float FRICTION_BOOST_AMOUNT = 0.2;

float adaptiveFrictionVoltage = 0;
long lastAdaptivePosition = 0;
unsigned long adaptiveStartTime = 0;
bool adaptiveLearning = false;
bool adaptiveLearned = false;

// PID PARAMETERS
float KP = 0.015;
float KI = 0.003;
float KD = 0.020;

float KP_active = KP;
float KI_active = KI;
float KD_active = KD;

const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 0.8;
const int TARGET_BAND = 2;
const float MAX_INTEGRAL = 1200.0;
const unsigned long CONTROL_PERIOD = 10;

// Filtering constants (like PIDAutoTune)
const float DERIVATIVE_FILTER_ALPHA = 0.7;  // Low-pass filter for derivative
const float POSITION_FILTER_ALPHA = 0.85;   // Low-pass filter for position
const long DEADBAND = 5;  // Encoder counts (like PIDAutoTune)

// Anti-windup parameters (like PIDAutoTune)
const float INTEGRAL_DECAY_FAR = 0.95;  // Decay rate when far from target
const float INTEGRAL_DECAY_CROSS = 0.5;  // Decay on zero crossing

// MOTION CONTROL
long desiredPosition = 0;
float errorIntegral = 0;
float lastError = 0;
float motorVelocity = 0;
float filteredPosition = 0;
bool positionFilterInitialized = false;
float lastDerivative = 0;  // For derivative filtering (internal to filteredDerivative function)
int previousMotorPosition = 0;
long previousVelCompTime = 0;
long lastStuckCheckPos = 0;
unsigned long lastStuckCheckTime = 0;
int stuckCounter = 0;
unsigned long stuckStartTime = 0;
bool voltageRamping = false;
int positionRetryCount = 0;
const int MAX_POSITION_RETRIES = 1;
const int RETRY_ERROR_THRESHOLD = 3;
const int MIN_VEL_COMP_COUNT = 2;
const long MIN_VEL_COMP_TIME = 10000;

unsigned long lastControlTime = 0;
unsigned long lastSensorTime = 0;
unsigned long arrivalTime = 0;
const int targetActivateTime = 100;

// SYSTEM STATE
bool systemEnabled = false;
bool autoMode = false;
unsigned long lastPrintTime = 0;
unsigned long moveStartTime = 0;
bool targetReached = false;
bool voltageRampedForRetry = false;
unsigned long retryStartTime = 0;
const unsigned long RETRY_TIME_THRESHOLD = 2000;  // 2 seconds
const unsigned long TARGET_SWITCH_TIME = 4000;  // 4 seconds
const long RETRY_LARGE_ERROR_THRESHOLD = 50;  // Error threshold for retry
unsigned long lastDriftCheckTime = 0;
const unsigned long DRIFT_CHECK_INTERVAL = 2000;  // Check drift every 2 seconds (more frequent)
const unsigned long DRIFT_CHECK_DURING_MOVE = 1000;  // Check drift every 1 second during movement
long lastDriftCheckPosition = 0;
const long MAX_DRIFT_THRESHOLD = 15;  // Reduced threshold for earlier detection
long lastKnownGoodPosition = 0;
unsigned long lastPositionValidationTime = 0;
const unsigned long POSITION_VALIDATION_INTERVAL = 500;  // Validate position every 500ms

const float CALIBRATION_VOLTAGE = 4.0;
const float HOMING_VOLTAGE = 4.0;

// Homing parameters
const float CALIBRATE_EXTRA_VOLTAGE = 0.6;
const float CALIBRATE_MIN_VOLTAGE = 3.5;
const unsigned long CALIBRATE_HOLD_TIME = 300;
const int CALIBRATE_STABLE_TICKS = 3;
const float VEL_STOP_THRESH = 2.0;

unsigned long targetHitTime = 0;
const unsigned long MIN_HIT_TIME = 100;

// SETUP
void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

  // Flip switch for motor enable/disable
  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);

  // Limit switches configured for active HIGH logic
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
    ProxSensors[i].prevValForVelocity = ProxSensors[i].currVal;
    ProxSensors[i].prevChangeTime = millis();
    ProxSensors[i].lastVelocityUpdate = millis();
    ProxSensors[i].direction = STOPPED;
    ProxSensors[i].prevDirection = STOPPED;
    ProxSensors[i].forwardCount = 0;
    ProxSensors[i].backwardCount = 0;
    ProxSensors[i].velocity = 0.0;
    ProxSensors[i].prevVelocity = 0.0;
    ProxSensors[i].hitDetected = false;
    ProxSensors[i].hitTime = 0;
  }
  
  stopMotor();
  delay(500);

  loadCalibrationFromEEPROM();
  if (EEPROM.read(EEPROM_FLAG) == 0xAA) {
    Serial.println(F("EEPROM OK"));
  } else {
    Serial.println(F("EEPROM empty"));
  }

  printWelcome();
  printHelp();
}

// MAIN LOOP
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
    
    if (autoMode && (currentTime - lastPrintTime >= 200)) {
      lastPrintTime = currentTime;

      if (dynamicCalibrationActive) {
        printCalibrationProgress();
      } else {
        printCompactStatus();
      }
    }
    
    // Drift mitigation - check more frequently during auto mode
    bool isMoving = (currentState == MOVE_TO_TARGET && abs(motorVelocity) > 5);
    unsigned long driftInterval = isMoving ? DRIFT_CHECK_DURING_MOVE : DRIFT_CHECK_INTERVAL;
    
    if (autoMode && !dynamicCalibrationActive && (currentTime - lastDriftCheckTime >= driftInterval)) {
      mitigateDrift();
      lastDriftCheckTime = currentTime;
    }
    
    // Continuous position validation - runs very frequently
    if (autoMode && !dynamicCalibrationActive && (currentTime - lastPositionValidationTime >= POSITION_VALIDATION_INTERVAL)) {
      validatePosition();
      lastPositionValidationTime = currentTime;
    }
  }
  
  checkLimitSwitches();
}

// SENSOR UPDATE
void updateAllSensors() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 4; i++) {
    ProxSensors[i].prevDirection = ProxSensors[i].direction;
    ProxSensors[i].prevVelocity = ProxSensors[i].velocity;

    float rawReading = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal + (1.0 - alpha) * rawReading;

    // Calculate velocity
    unsigned long timeDelta = currentTime - ProxSensors[i].lastVelocityUpdate;
    if (timeDelta > 0) {
      float valueDelta = ProxSensors[i].currVal - ProxSensors[i].prevValForVelocity;
      ProxSensors[i].velocity = (valueDelta * 1000.0) / timeDelta;
      ProxSensors[i].lastVelocityUpdate = currentTime;
      ProxSensors[i].prevValForVelocity = ProxSensors[i].currVal;
    } else {
      ProxSensors[i].velocity = 0.0;
    }
    noiseLimit = (ProxSensors[i].currVal >= noiseThreshold) ? upperNoiseLimit : lowerNoiseLimit;

    // Direction detection
    if (abs(ProxSensors[i].currVal - ProxSensors[i].prevVal) < noiseLimit) {
      if (timeDelta >= stopTimeout) {
        ProxSensors[i].direction = STOPPED;
      }
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].backwardCount = 0;

    } else if (ProxSensors[i].currVal - ProxSensors[i].prevVal < 0) {
      // Approaching
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;

      if (ProxSensors[i].forwardCount > 3) {
        ProxSensors[i].direction = FORWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = currentTime;
        // Reset hit
        if (ProxSensors[i].hitDetected) {
          ProxSensors[i].hitDetected = false;
          ProxSensors[i].hitTime = 0;
        }
      }

    } else {
      // Retreating
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;

      if (ProxSensors[i].backwardCount > 3) {
        ProxSensors[i].direction = BACKWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = currentTime;
      }
    }

    // Hit detection: velocity reversal
    const float VELOCITY_THRESHOLD = 5.0;
    const float HIT_VELOCITY_THRESHOLD = 10.0;
    if (!ProxSensors[i].hitDetected) {
      bool wasApproaching = (ProxSensors[i].prevVelocity < -VELOCITY_THRESHOLD);
      bool nowRetreating = (ProxSensors[i].velocity > HIT_VELOCITY_THRESHOLD);
      bool directionReversed = (ProxSensors[i].prevDirection == FORWARD && 
                                 ProxSensors[i].direction == BACKWARD);
      if ((wasApproaching && nowRetreating) || directionReversed) {
        ProxSensors[i].hitDetected = true;
        ProxSensors[i].hitTime = currentTime;
      }
    } else {
      if (ProxSensors[i].direction == FORWARD && 
          ProxSensors[i].velocity < -VELOCITY_THRESHOLD) {
        ProxSensors[i].hitDetected = false;
        ProxSensors[i].hitTime = 0;
      }
    }

    // Update distances
    if (sensorCalibrated) {
      zombieDistances[i] = (ProxSensors[i].currVal - ProxRange[i][1]) /
                           (float)(ProxRange[i][0] - ProxRange[i][1]);
      zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);
    } else {
      zombieDistances[i] = 1.0;
    }
  }
}

// VELOCITY
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
// DYNAMIC SENSOR CALIBRATION
// ============================================
void startDynamicCalibration() {
  
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
  
  bool allValid = true;
  for (int i = 0; i < 4; i++) {
    int range = dynamicMax[i] - dynamicMin[i];
    if (range < 50) allValid = false;
    ProxRange[i][0] = dynamicMax[i];
    ProxRange[i][1] = dynamicMin[i];
  }
  sensorCalibrated = true;
  if (homeToLeftLimit()) {
    findRangeAndSetBounds();
  }
}

void printCalibrationProgress() {
  Serial.print((DYNAMIC_CALIBRATION_TIME - (millis() - calibrationStartTime)) / 1000);
  for (int i = 0; i < 4; i++) {
    Serial.print(F(" "));
    Serial.print(dynamicMax[i] - dynamicMin[i]);
  }
  Serial.println();
}

void findRangeAndSetBounds() {
  // CRITICAL: Lane positions are NEVER modified during range finding
  // They remain as loaded from EEPROM
  
  if (!homeToLeftLimit()) return;
  LOWER_BOUND = encoder.read();
  unsigned long startTime = millis();
  setMotor(-max(FRICTION_LEFT + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE));
  while (digitalRead(LIMIT_RIGHT) == LOW && (millis() - startTime) < 15000) delay(10);
  if (digitalRead(LIMIT_RIGHT) == HIGH) {
    long currentPos = encoder.read();
    long lastPos = currentPos;
    unsigned long holdStart = millis();
    int stableTicks = 0;
    float holdVoltage = max(FRICTION_LEFT, CALIBRATE_MIN_VOLTAGE - 0.5);
    while (millis() - holdStart < CALIBRATE_HOLD_TIME || stableTicks < CALIBRATE_STABLE_TICKS) {
      setMotor(-holdVoltage);
      delay(10);
      long pos = encoder.read();
      if (abs(pos - lastPos) <= 1) stableTicks++;
      else { stableTicks = 0; lastPos = pos; }
    }
    stopMotor();
    delay(100);
    UPPER_BOUND = encoder.read();
    rangeFindingComplete = true;
    
    // CRITICAL: Reload lane positions from EEPROM after range finding
    // This ensures they're never modified
    loadCalibrationFromEEPROM();
  } else {
    stopMotor();
  }
}

// STATE MACHINE
void runStateMachine() {
  switch (currentState) {
    
    case CALIBRATE:
      desiredPosition = LOWER_BOUND;

      if (!dynamicCalibrationActive && sensorCalibrated) {
        currentState = CHOOSE_ACTIVE_TARGET;
        systemEnabled = true;
      }
      else if (!dynamicCalibrationActive && !sensorCalibrated) {
        // Go directly to sensor calibration (range will be found after calibration)
        startDynamicCalibration();
        desiredPosition = LOWER_BOUND;
        systemEnabled = true;
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      if (autoMode) {
        long pos = encoder.read();
        if (pos < UPPER_BOUND) {
          encoder.write(UPPER_BOUND);
          pos = UPPER_BOUND;
        }
        if (pos > LOWER_BOUND) {
          encoder.write(LOWER_BOUND);
          pos = LOWER_BOUND;
        }
        lastKnownGoodPosition = pos;
      }
      
      // Check if we should stay committed to current target
      bool shouldStayCommitted = false;
      if (activeTargetIndex >= 0 && 
          ProxSensors[activeTargetIndex].direction == FORWARD &&
          zombieDistances[activeTargetIndex] < MIN_COMMIT_DISTANCE &&
          (millis() - targetCommitTime) < MIN_TARGET_COMMIT_TIME) {
        // Very close to target and within commit time - stay committed
        shouldStayCommitted = true;
      }
      
      // If committed, only switch if target is clearly retreating or hit
      if (shouldStayCommitted) {
        if (ProxSensors[activeTargetIndex].hitDetected ||
            (ProxSensors[activeTargetIndex].direction == BACKWARD && 
             zombieDistances[activeTargetIndex] > 0.70)) {
          // Target hit or retreated far - allow switch
          shouldStayCommitted = false;
        } else {
          // Stay with current target
          activeTargetPosition = targetPositions[activeTargetIndex];
          WAIT_POS = false;
          if (lane4LimitSwitchMode) {
            desiredPosition = UPPER_BOUND;
          } else {
            desiredPosition = activeTargetPosition;
          }
          moveStartTime = millis();
          arrivalTime = millis();
          currentState = MOVE_TO_TARGET;
          break;
        }
      }
      
      activeTargetIndex = -1;
      closestZombieDist = 2.0;

      // Track position
      previousMoveStartPosition = encoder.read();

      // Find closest forward zombie
      for (int i = 0; i < 4; i++) {
        // Only target forward zombies
        if (ProxSensors[i].direction == FORWARD &&
            zombieDistances[i] < closestZombieDist) {
          closestZombieDist = zombieDistances[i];
          activeTargetIndex = i;
        }
      }
      
      // If we had a previous target, apply hysteresis
      if (previousTargetIndex >= 0 && 
          previousTargetIndex < 4 &&
          ProxSensors[previousTargetIndex].direction == FORWARD &&
          zombieDistances[previousTargetIndex] < MIN_COMMIT_DISTANCE) {
        // Previous target is still close - only switch if new target is significantly closer
        if (activeTargetIndex >= 0 && 
            zombieDistances[activeTargetIndex] < (zombieDistances[previousTargetIndex] - CLOSER_THREAT_THRESHOLD)) {
          // New target is significantly closer - switch
        } else {
          // Stay with previous target
          activeTargetIndex = previousTargetIndex;
          closestZombieDist = zombieDistances[activeTargetIndex];
        }
      }
      
      if (activeTargetIndex >= 0) {
        activeTargetPosition = targetPositions[activeTargetIndex];
        WAIT_POS = false;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        previousZombieDistance = zombieDistances[activeTargetIndex];
        
        if (activeTargetIndex == 3) {
          lane4LimitSwitchMode = true;
          lane4AtLimit = false;
          lane4LimitTime = 0;
        } else {
          lane4LimitSwitchMode = false;
          lane4AtLimit = false;
        }
        
        int percentToPhoto = (int)((1.0 - zombieDistances[activeTargetIndex]) * 100);
        
        if (activeTargetIndex != 3) {
        }
        
        // Only update commit time if target actually changed
        if (previousTargetIndex != activeTargetIndex) {
          targetCommitTime = millis();
        }
        
        previousTargetIndex = activeTargetIndex;
        targetHitTime = 0;
        // Initialize retreat check when starting to move to target
        // Reset to 0 so it gets initialized when we arrive at target
        lastRetreatCheckDistance = 0.0;
        lastRetreatCheckTime = 0;
        
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        lane4LimitSwitchMode = false;
        lane4AtLimit = false;
        targetCommitTime = 0;
      }
      
      if (lane4LimitSwitchMode) {
        desiredPosition = UPPER_BOUND;  // Go to right limit switch first
      } else {
        desiredPosition = activeTargetPosition;
      }
      
      moveStartTime = millis();
      arrivalTime = millis();
      targetReached = false;
      stuckCounter = 0;
      positionRetryCount = 0;
      voltageRampedForRetry = false;
      retryStartTime = 0;
      currentState = MOVE_TO_TARGET;
      break;
    
    case MOVE_TO_TARGET:
      long currentPos = encoder.read();
      
      // Check for large jumps before movement - only if truly stationary
      // Don't over-correct during leftward movement preparation
      if (autoMode && lastKnownGoodPosition != 0 && abs(motorVelocity) < 3) {
        long jump = abs(currentPos - lastKnownGoodPosition);
        // Only correct very large jumps when stationary
        if (jump > 100) {
          encoder.write(lastKnownGoodPosition);
          currentPos = lastKnownGoodPosition;
        }
      }
      
      if (currentPos < UPPER_BOUND) {
        encoder.write(UPPER_BOUND);
        currentPos = UPPER_BOUND;
        lastKnownGoodPosition = UPPER_BOUND;
      }
      if (currentPos > LOWER_BOUND) {
        encoder.write(LOWER_BOUND);
        currentPos = LOWER_BOUND;
        lastKnownGoodPosition = LOWER_BOUND;
      }
      
      if (lane4LimitSwitchMode && activeTargetIndex == 3) {
        if (!lane4AtLimit) {
          if (rightPressed()) {
            lane4AtLimit = true;
            lane4LimitTime = millis();
            encoder.write(UPPER_BOUND);
            currentPos = UPPER_BOUND;
            lastKnownGoodPosition = UPPER_BOUND;
          }
          // Force movement to right limit
          desiredPosition = UPPER_BOUND;
        }
        else if (millis() - lane4LimitTime < LANE4_LIMIT_HOLD_TIME) {
          desiredPosition = UPPER_BOUND;
          // Continuously clamp encoder while at limit
          if (rightPressed() || currentPos < UPPER_BOUND) {
            encoder.write(UPPER_BOUND);
            currentPos = UPPER_BOUND;
            lastKnownGoodPosition = UPPER_BOUND;
          }
        }
        else if (currentPos < (UPPER_BOUND + LANE4_BACKOFF_DISTANCE)) {
          // Back off from limit switch
          desiredPosition = UPPER_BOUND + LANE4_BACKOFF_DISTANCE;
        }
        else {
          // Now move to actual target position
          desiredPosition = activeTargetPosition;
          // Only disable limit switch mode once we're close to target
          long errorToTarget = abs(activeTargetPosition - currentPos);
          if (errorToTarget <= TARGET_BAND * 2) {
            lane4LimitSwitchMode = false;
          }
        }
      }
      // Normal mode
      else {
        if (fineAdjustmentActive) {
          desiredPosition = fineAdjustmentTarget;
        } else {
          desiredPosition = activeTargetPosition;
        }
      }
      
      long error = desiredPosition - currentPos;
      
      if (rightPressed() && error < 0) {
        stopMotor();
        encoder.write(UPPER_BOUND);
        return;
      }
      if (activeTargetIndex == 3 && currentPos < UPPER_BOUND) {
        stopMotor();
        encoder.write(UPPER_BOUND);
        return;
      }
      
      if (!autoMode && !lane4LimitSwitchMode && currentPos < UPPER_BOUND - 50) {
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
      if (activeTargetIndex >= 0 && !WAIT_POS && !lane4LimitSwitchMode) {
        int targetDirection = ProxSensors[activeTargetIndex].direction;
        bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
        unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
        
        // Velocity-based hit detection
        if (hitDetected && hitTime > 0) {
          // Confirm hit
          if (millis() - hitTime >= MIN_HIT_TIME) {
            // Reset hit detection for this sensor
            ProxSensors[activeTargetIndex].hitDetected = false;
            ProxSensors[activeTargetIndex].hitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        }
        
        // Backup: direction change
        int prevDirection = ProxSensors[activeTargetIndex].prevDirection;
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
          
          if (targetHitTime == 0) {
            targetHitTime = millis();
          }
          
          // Confirm hit
          if (millis() - targetHitTime >= MIN_HIT_TIME) {
            targetHitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        } else {
          targetHitTime = 0;  // Reset if not consistently backward
        }
        
        // Wait for retreat confirmation - don't switch immediately
        // Only switch if target has moved away significantly
        if (targetDirection == BACKWARD) {
          float currentDistance = zombieDistances[activeTargetIndex];
          
          // Initialize retreat check if not set
          if (lastRetreatCheckDistance == 0.0) {
            lastRetreatCheckDistance = currentDistance;
            lastRetreatCheckTime = millis();
          }
          
          float distanceIncrease = currentDistance - lastRetreatCheckDistance;
          
          // Update retreat check if distance is increasing
          if (distanceIncrease > 0) {
            lastRetreatCheckDistance = currentDistance;
            lastRetreatCheckTime = millis();
          }
          
          // Only switch if distance has increased significantly (target moving away)
          // AND we've been at target for minimum time
          if (distanceIncrease > RETREAT_DISTANCE_THRESHOLD && 
              millis() - arrivalTime > MIN_WAIT_AT_TARGET &&
              (millis() - targetCommitTime) >= MIN_TARGET_COMMIT_TIME) {
            lastRetreatCheckDistance = 0.0;  // Reset for next target
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
          
          // Also switch if retreated very far (safety check) - but still require commit time
          if (currentDistance > 0.85 && 
              (millis() - targetCommitTime) >= MIN_TARGET_COMMIT_TIME) {
            lastRetreatCheckDistance = 0.0;  // Reset for next target
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        }

        // Check for closer threat - only if significantly closer and we're not too close to current target
        if (zombieDistances[activeTargetIndex] > MIN_COMMIT_DISTANCE) {
          // Only check for closer threats if we're not very close to current target
          for (int i = 0; i < 4; i++) {
            if (i != activeTargetIndex &&
                ProxSensors[i].direction == FORWARD &&
                zombieDistances[i] < (zombieDistances[activeTargetIndex] - CLOSER_THREAT_THRESHOLD)) {
              // Much closer threat - switch
              currentState = CHOOSE_ACTIVE_TARGET;
              break;
            }
          }
        }
      }
      
      // Check for forward zombies
      bool hasForwardZombie = false;
      for (int i = 0; i < 4; i++) {
        if (ProxSensors[i].direction == FORWARD) {
          hasForwardZombie = true;
          break;
        }
      }

      // No forward zombies
      if (!hasForwardZombie && millis() - moveStartTime > 1000) {
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }

      // Check arrival against current desired position (may be fine-adjusted)
      long errorToCurrentTarget = desiredPosition - currentPos;
      // Also check against original target for fine adjustment logic
      long errorToOriginalTarget = activeTargetPosition - currentPos;
      
      // Retry logic: After 2 seconds, increase voltage if error is too large
      unsigned long moveDuration = millis() - moveStartTime;
      if (moveDuration >= RETRY_TIME_THRESHOLD && abs(errorToOriginalTarget) > RETRY_LARGE_ERROR_THRESHOLD) {
        if (!voltageRampedForRetry) {
          voltageRampedForRetry = true;
          retryStartTime = millis();
        }
      } else if (abs(errorToOriginalTarget) <= RETRY_LARGE_ERROR_THRESHOLD) {
        voltageRampedForRetry = false;
        retryStartTime = 0;
      }
      
      // After 4 seconds, switch to next closest target if still too far
      if (moveDuration >= TARGET_SWITCH_TIME && abs(errorToOriginalTarget) > RETRY_LARGE_ERROR_THRESHOLD) {
        // Find next closest forward zombie
        int nextTarget = -1;
        float nextClosestDist = 2.0;
        for (int i = 0; i < 4; i++) {
          if (i != activeTargetIndex && 
              ProxSensors[i].direction == FORWARD &&
              zombieDistances[i] < nextClosestDist) {
            nextClosestDist = zombieDistances[i];
            nextTarget = i;
          }
        }
        if (nextTarget >= 0) {
          activeTargetIndex = nextTarget;
          activeTargetPosition = targetPositions[nextTarget];
          moveStartTime = millis();
          voltageRampedForRetry = false;
          retryStartTime = 0;
          fineAdjustmentActive = false;
          fineAdjustmentCount = 0;
          if (activeTargetIndex == 3) {
            lane4LimitSwitchMode = true;
            lane4AtLimit = false;
          } else {
            lane4LimitSwitchMode = false;
          }
        } else {
          currentState = CHOOSE_ACTIVE_TARGET;
        }
        break;
      }
      
      static long lastFineAdjustPosition = 0;
      static unsigned long lastFineAdjustPositionTime = 0;
      bool positionChanged = false;
      if (abs(currentPos - lastFineAdjustPosition) > 2) {
        lastFineAdjustPosition = currentPos;
        lastFineAdjustPositionTime = millis();
        positionChanged = true;
      } else if (millis() - lastFineAdjustPositionTime < 500) {
        positionChanged = true;
      }
      bool notStuckAtLane4 = !(currentPos < -1200 && !positionChanged && abs(errorToOriginalTarget) > 3);
      if (activeTargetIndex >= 0 && activeTargetIndex != 3 &&
          !WAIT_POS && !fineAdjustmentActive &&
          fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
          (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
          abs(motorVelocity) < 30 &&
          abs(errorToOriginalTarget) <= 8 &&
          abs(errorToOriginalTarget) > 5 &&
          ProxSensors[activeTargetIndex].direction == FORWARD &&
          !ProxSensors[activeTargetIndex].hitDetected &&
          zombieDistances[activeTargetIndex] < 0.30 &&
          notStuckAtLane4) {
        
        fineAdjustmentTarget = activeTargetPosition;
        
        fineAdjustmentCount++;
        lastFineAdjustmentTime = millis();
        Serial.print(F("FA:"));
        Serial.print(errorToOriginalTarget);
        Serial.print(F("/"));
        Serial.println(fineAdjustmentCount);
        
        fineAdjustmentActive = true;
        desiredPosition = fineAdjustmentTarget;
        arrivalTime = millis();
      }
      
      if (abs(errorToCurrentTarget) <= TARGET_BAND) {
        if (WAIT_POS) {
          // At wait position
          if (hasForwardZombie) {
            currentState = CHOOSE_ACTIVE_TARGET;
          }
          // Otherwise stay put at wait position
        } else if (millis() - arrivalTime > targetActivateTime) {
          // Check hit
          if (activeTargetIndex >= 0) {
            bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
            unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
            
            // Check hit
            if (hitDetected && hitTime > 0 && millis() - hitTime >= MIN_HIT_TIME) {
              ProxSensors[activeTargetIndex].hitDetected = false;
              ProxSensors[activeTargetIndex].hitTime = 0;
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            // Check retreat - wait until target starts moving away (distance increasing)
            else if (ProxSensors[activeTargetIndex].direction == BACKWARD) {
              // Only switch if we've waited long enough and distance is clearly increasing
              unsigned long timeAtTarget = millis() - arrivalTime;
              float currentDistance = zombieDistances[activeTargetIndex];
              
              // Initialize retreat check if not set
              if (lastRetreatCheckDistance == 0.0) {
                lastRetreatCheckDistance = currentDistance;
                lastRetreatCheckTime = millis();
              }
              
              float distanceIncrease = currentDistance - lastRetreatCheckDistance;
              
              // Update retreat check distance if enough time has passed
              // Also require minimum commit time before allowing switch
              if (timeAtTarget > MIN_WAIT_AT_TARGET && 
                  (millis() - targetCommitTime) >= MIN_TARGET_COMMIT_TIME) {
                if (distanceIncrease > RETREAT_DISTANCE_THRESHOLD) {
                  // Target is clearly moving away - switch to next target
                  lastRetreatCheckDistance = 0.0;  // Reset for next target
                  currentState = CHOOSE_ACTIVE_TARGET;
                } else if (currentDistance > lastRetreatCheckDistance) {
                  // Distance is increasing but not enough yet - update check point
                  lastRetreatCheckDistance = currentDistance;
                  lastRetreatCheckTime = millis();
                } else if (millis() - lastRetreatCheckTime > 200) {
                  // Update check point periodically even if distance not increasing
                  lastRetreatCheckDistance = currentDistance;
                  lastRetreatCheckTime = millis();
                }
              }
            }
            // Fine positioning - DISABLED for Lane 4 to prevent oscillation
            else if (activeTargetIndex != 3 &&
                     ProxSensors[activeTargetIndex].direction == FORWARD && 
                     !fineAdjustmentActive &&
                     fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
                     (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
                     abs(motorVelocity) < 30 &&
                     abs(errorToOriginalTarget) > 5 &&
                     abs(errorToOriginalTarget) <= 7 &&
                     !ProxSensors[activeTargetIndex].hitDetected) {
              
              if (abs(errorToOriginalTarget) > 0) {
                fineAdjustmentTarget = activeTargetPosition;
                
                fineAdjustmentCount++;
                lastFineAdjustmentTime = millis();
                
                fineAdjustmentActive = true;
                desiredPosition = fineAdjustmentTarget;
                arrivalTime = millis();
              }
            }
            
            else if (activeTargetIndex != 3 &&
                     ProxSensors[activeTargetIndex].direction == FORWARD && 
                     fineAdjustmentActive &&
                     fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
                     (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
                     abs(motorVelocity) < 25 &&
                     abs(errorToCurrentTarget) > 5 &&
                     !ProxSensors[activeTargetIndex].hitDetected &&
                     (millis() - arrivalTime) >= 100 &&
                     (millis() - arrivalTime) < 2000) {  // Timeout after 2 seconds
              
              fineAdjustmentTarget = activeTargetPosition;
              
              fineAdjustmentCount++;
              lastFineAdjustmentTime = millis();
              
              desiredPosition = fineAdjustmentTarget;
              arrivalTime = millis();
            }
            else if (fineAdjustmentActive && (millis() - arrivalTime) >= 2000) {
              fineAdjustmentActive = false;
              fineAdjustmentCount = 0;
            }
            if (activeTargetIndex >= 0) {
              previousZombieDistance = zombieDistances[activeTargetIndex];
              // Update retreat check distance when at target
              if (abs(errorToCurrentTarget) <= TARGET_BAND) {
                float currentDistance = zombieDistances[activeTargetIndex];
                if (currentDistance != lastRetreatCheckDistance) {
                  lastRetreatCheckDistance = currentDistance;
                  lastRetreatCheckTime = millis();
                }
              }
            }
          }
        }
      } else {
        arrivalTime = millis();
        if (fineAdjustmentActive && abs(errorToCurrentTarget) > TARGET_BAND * 2) {
          fineAdjustmentActive = false;
          fineAdjustmentCount = 0;
          lastFineAdjustmentTime = 0;
        }
      }
      break;
  }
}

// FILTERING (like PIDAutoTune)
float filteredDerivative(float rawDerivative) {
  lastDerivative = DERIVATIVE_FILTER_ALPHA * rawDerivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * lastDerivative;
  return lastDerivative;
}

// SIMPLIFIED PID UPDATE (like PIDAutoTune)
float updatePID(long targetPosition) {
  // Read and filter position
  long rawPosition = encoder.read();
  if (!positionFilterInitialized) {
    filteredPosition = (float)rawPosition;
    positionFilterInitialized = true;
  } else {
    filteredPosition = POSITION_FILTER_ALPHA * (float)rawPosition + (1.0 - POSITION_FILTER_ALPHA) * filteredPosition;
  }
  long currentPosition = (long)filteredPosition;
  
  // Calculate error
  float error = targetPosition - currentPosition;
  
  // Apply deadband (like PIDAutoTune) - if within deadband, set error to 0
  if (abs(error) < DEADBAND) {
    error = 0;
    // Reset integral when in deadband to prevent windup
    errorIntegral = 0;
  } else {
    // Calculate integral with anti-windup
    float dt = CONTROL_PERIOD / 1000.0;
    float integralTerm = error * dt;
    
    // Conditional integration - don't accumulate if output would saturate
    float testIntegral = errorIntegral + integralTerm;
    if (abs(testIntegral) < MAX_INTEGRAL) {
      errorIntegral = testIntegral;
    }
  }
  
  // Calculate derivative with filtering
  float dt = CONTROL_PERIOD / 1000.0;
  float rawDerivative = (error - lastError) / dt;
  float derivative = filteredDerivative(rawDerivative);
  
  // Calculate PID output
  float pidOutput = KP * error + KI * errorIntegral + KD * derivative;
  
  // Add friction compensation (simple, like PIDAutoTune)
  float frictionComp = 0;
  if (error < -DEADBAND) {
    // Moving RIGHT (negative direction)
    frictionComp = -FRICTION_LEFT;
  } else if (error > DEADBAND) {
    // Moving LEFT (positive direction)
    frictionComp = FRICTION_RIGHT;
  }
  
  // Prevent crossing bounds - don't apply voltage that would cross limits
  if (currentPosition > LOWER_BOUND) {
    // Past lower bound - block all leftward movement
    frictionComp = 0;
    if (pidOutput > 0) pidOutput = 0;
  } else if (currentPosition == LOWER_BOUND && error > DEADBAND && targetPosition > LOWER_BOUND) {
    // At lower bound, error is significant, and target is beyond it - block leftward movement
    frictionComp = 0;
    if (pidOutput > 0) pidOutput = 0;
  }
  if (currentPosition < UPPER_BOUND) {
    // Past upper bound - block all rightward movement
    frictionComp = 0;
    if (pidOutput < 0) pidOutput = 0;
  } else if (currentPosition == UPPER_BOUND && error < -DEADBAND && targetPosition < UPPER_BOUND) {
    // At upper bound, error is significant, and target is beyond it - block rightward movement
    frictionComp = 0;
    if (pidOutput < 0) pidOutput = 0;
  }
  
  // Calculate total voltage
  float voltage = pidOutput + frictionComp;
  
  // Store for next iteration
  lastError = error;
  
  return voltage;
}

// VOLTAGE CAPPING (like PIDAutoTune) - significantly reduced speeds for better target registration
float cappedVoltageForError(float voltage, long error) {
  long absErr = abs(error);
  float cap;
  // Much lower voltage caps to slow down movement significantly
  if (absErr > 1000) cap = 5.5f;  // Much slower for large moves
  else if (absErr > 800) cap = 5.0f;
  else if (absErr > 500) cap = 4.5f;
  else if (absErr > 300) cap = 4.0f;
  else if (absErr > 100) cap = 3.5f;
  else if (absErr > 50) cap = 3.0f;
  else cap = 2.5f;  // Very slow for fine positioning
  cap = min(cap, 6.0f);  // Absolute max reduced significantly
  return constrain(voltage, -cap, cap);
}

// MOTION CONTROL
void runMotionControl() {
  long rawPosition = encoder.read();
  
  // Bounds checking and drift detection (before filtering)
  if (rawPosition < UPPER_BOUND) {
    encoder.write(UPPER_BOUND);
    rawPosition = UPPER_BOUND;
    lastKnownGoodPosition = UPPER_BOUND;
    positionFilterInitialized = false;  // Reset filter
  }
  if (rawPosition > LOWER_BOUND) {
    encoder.write(LOWER_BOUND);
    rawPosition = LOWER_BOUND;
    lastKnownGoodPosition = LOWER_BOUND;
    positionFilterInitialized = false;  // Reset filter
  }
  
  // Quick drift check - only when truly stationary
  if (autoMode && lastKnownGoodPosition != 0 && abs(motorVelocity) < 3 && currentState != MOVE_TO_TARGET) {
    long positionChange = abs(rawPosition - lastKnownGoodPosition);
    if (positionChange > 100) {
      encoder.write(lastKnownGoodPosition);
      rawPosition = lastKnownGoodPosition;
      positionFilterInitialized = false;  // Reset filter
    }
  }
  
  if (dynamicCalibrationActive) {
    desiredPosition = LOWER_BOUND;
    float error = desiredPosition - rawPosition;
    if (abs(error) > 10) {
      setMotor(constrain(error * 0.05, -2.0, 2.0));
    } else {
      stopMotor();
    }
    return;
  }
  
  // Use simplified PID update (like PIDAutoTune) - this handles filtering internally
  float voltage = updatePID(desiredPosition);
  
  // Get current error and position from filtered values
  long currentPos = (long)filteredPosition;
  float error = desiredPosition - currentPos;
  long absErr = abs((long)error);
  
  // Prevent crossing limits
  if (rightPressed() && error < 0) {
    stopMotor();
    encoder.write(UPPER_BOUND);
    lastError = 0;
    return;
  }
  if (leftPressed() && error > 0) {
    stopMotor();
    encoder.write(LOWER_BOUND);
    lastError = 0;
    return;
  }
  // Prevent crossing bounds - block if past bounds
  if (currentPos > LOWER_BOUND) {
    stopMotor();
    encoder.write(LOWER_BOUND);
    lastError = 0;
    return;
  }
  if (currentPos < UPPER_BOUND) {
    stopMotor();
    encoder.write(UPPER_BOUND);
    lastError = 0;
    return;
  }
  
  // Deadband is now handled in updatePID(), but check if we're at target
  // If within deadband, stop regardless of position
  if (abs(error) <= DEADBAND) {
    stopMotor();
    errorIntegral = 0;
    lastError = 0;
    if (!targetReached) {
      targetReached = true;
    }
    return;
  }
  
  // If at a bound and trying to move past it, block (unless error is very small, handled by deadband above)
  if (currentPos == LOWER_BOUND && error > DEADBAND && desiredPosition > LOWER_BOUND) {
    stopMotor();
    lastError = 0;
    return;
  }
  if (currentPos == UPPER_BOUND && error < -DEADBAND && desiredPosition < UPPER_BOUND) {
    stopMotor();
    lastError = 0;
    return;
  }
  
  targetReached = false;

  if (abs(error) > 3 && abs(error) > RETRY_ERROR_THRESHOLD && abs(error) < 50) {
    if (millis() - moveStartTime > 300 && positionRetryCount < MAX_POSITION_RETRIES) {
      positionRetryCount++;

      errorIntegral = 0;
      lastError = 0;
      moveStartTime = millis();
      stuckCounter = 0;
      stuckStartTime = 0;
      voltageRamping = false;
      delay(100);
      return;
    }
  }
  
  // Adaptive friction learning
  if (abs(error) > 100 && abs(error) < 800 && !adaptiveLearning && !adaptiveLearned) {
    adaptiveLearning = true;
          adaptiveFrictionVoltage = 1.5;
    lastAdaptivePosition = currentPos;
    adaptiveStartTime = millis();
    
    // Direction
    bool movingRight = (error < 0);
  }
  
  if (adaptiveLearning) {
    // Check movement
    long positionChange = abs(currentPos - lastAdaptivePosition);
    unsigned long elapsed = millis() - adaptiveStartTime;
    
      if (positionChange >= 5) {
      bool movingRight = (error < 0);
      
      if (movingRight) {
        adaptiveFrictionLeft = adaptiveFrictionVoltage;
        FRICTION_LEFT = adaptiveFrictionVoltage;
      } else {
        adaptiveFrictionRight = adaptiveFrictionVoltage;
        FRICTION_RIGHT = adaptiveFrictionVoltage;
      }
      
      
      adaptiveLearning = false;
      adaptiveLearned = true;
      
      // Apply friction
      float voltage = (error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage;
      setMotor(voltage);
      return;
    }
    
      if (elapsed >= 150) {
        adaptiveFrictionVoltage += 0.3;
        adaptiveStartTime = millis();
        lastAdaptivePosition = currentPos;
      if (adaptiveFrictionVoltage > 4.5) {
        adaptiveFrictionVoltage = 2.5;
        adaptiveLearning = false;
        adaptiveLearned = true;
        bool movingRight = (error < 0);
        if (movingRight) {
          adaptiveFrictionLeft = 2.5;
          FRICTION_LEFT = 2.5;
        } else {
          adaptiveFrictionRight = 2.5;
          FRICTION_RIGHT = 2.5;
        }
      }
    }
    if (elapsed > 2000) {
      adaptiveLearning = false;
      adaptiveLearned = true;
      adaptiveFrictionVoltage = 2.0;
      bool movingRight = (error < 0);
      if (movingRight) {
        adaptiveFrictionLeft = 2.0;
        FRICTION_LEFT = 2.0;
      } else {
        adaptiveFrictionRight = 2.0;
        FRICTION_RIGHT = 2.0;
      }
    }
    setMotor((error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage);
    return;
  }
  
  // Apply integral decay on zero crossing (like PIDAutoTune)
  if ((error != 0) && (error * lastError < 0)) {
    errorIntegral *= INTEGRAL_DECAY_CROSS;
  }
  if (abs(error) > 600) {
    errorIntegral *= INTEGRAL_DECAY_FAR;
  }
  
  // Apply voltage capping based on error (like PIDAutoTune)
  float totalVoltage = cappedVoltageForError(voltage, (long)error);
  
  // Retry multiplier for stuck conditions - but keep speeds reasonable
  float retryMultiplier = 1.0;
  if (voltageRampedForRetry && retryStartTime > 0) {
    unsigned long retryDuration = millis() - retryStartTime;
    if (retryDuration > 1000) retryMultiplier = 1.2;  // Reduced from 1.3
    else retryMultiplier = 1.1;  // Reduced from 1.15
    totalVoltage *= retryMultiplier;
    totalVoltage = constrain(totalVoltage, -6.0, 6.0);  // Reduced max from 9.5
  }
  if (currentState == MOVE_TO_TARGET && autoMode) {
    bool isLargeMove = (absErr > 1000);
    int leftSlowdownDistance = isLargeMove ? 200 : 100;
    // Prevent crossing lower bound (zero) - stop before reaching it
    if (error > 0 && currentPos > (LOWER_BOUND - leftSlowdownDistance)) {
      float distanceFromLimit = LOWER_BOUND - currentPos;
      float proximityFactor = (distanceFromLimit + leftSlowdownDistance) / leftSlowdownDistance;
      proximityFactor = constrain(proximityFactor, 0.0, 1.0);
      float minVoltage = isLargeMove ? 0.2 : 0.3;
      float maxVoltage = isLargeMove ? 0.5 : 0.7;
      totalVoltage *= (minVoltage + (proximityFactor * (maxVoltage - minVoltage)));
      if (currentPos >= LOWER_BOUND) {
        stopMotor();
        encoder.write(LOWER_BOUND);
        return;
      }
    }
    if (!lane4LimitSwitchMode) {
      int rightSlowdownDistance = isLargeMove ? 200 : 100;
      if (error < 0 && currentPos < (UPPER_BOUND + rightSlowdownDistance)) {
        float distanceFromLimit = currentPos - UPPER_BOUND;
        float proximityFactor = (distanceFromLimit + rightSlowdownDistance) / rightSlowdownDistance;
        proximityFactor = constrain(proximityFactor, 0.0, 1.0);
        float minVoltage = isLargeMove ? 0.2 : 0.3;
        float maxVoltage = isLargeMove ? 0.5 : 0.7;
        totalVoltage *= (minVoltage + (proximityFactor * (maxVoltage - minVoltage)));
      }
    }
  }


  unsigned long currentTime = millis();
  if (abs(error) > 3) {
    if (currentTime - lastStuckCheckTime >= 150) {
      if (abs(currentPos - lastStuckCheckPos) < 2) {
        stuckCounter++;
        if (stuckCounter == 1) {
          stuckStartTime = currentTime;
          voltageRamping = false;
        }
        if (stuckCounter >= 2) {
          voltageRamping = true;
          bool movingRight = (error < 0);
          float baseFrictionVoltage = movingRight ? FRICTION_LEFT : FRICTION_RIGHT;
          float minFrictionVoltage = max(baseFrictionVoltage, 2.0f);
          if (currentPos < -1200) {
            minFrictionVoltage = max(minFrictionVoltage, 3.5f);
          }
          unsigned long stuckDuration = currentTime - stuckStartTime;
          float rampVoltage = minFrictionVoltage;
          if (stuckDuration > 1200) rampVoltage = minFrictionVoltage + 1.0f;  // Reduced from 1.5
          else if (stuckDuration > 800) rampVoltage = minFrictionVoltage + 0.7f;  // Reduced from 1.0
          else if (stuckDuration > 400) rampVoltage = minFrictionVoltage + 0.4f;  // Reduced from 0.5
          rampVoltage = min(rampVoltage, 6.0f);  // Reduced max from 9.5
          if (abs(totalVoltage) < rampVoltage) {
            float pidMagnitude = abs(totalVoltage);
            float finalVoltage = max(pidMagnitude, rampVoltage);
            totalVoltage = (error < 0) ? -finalVoltage : finalVoltage;
          }
        }
      } else {
        stuckCounter = 0;
        stuckStartTime = 0;
        voltageRamping = false;
      }
      lastStuckCheckPos = currentPos;
      lastStuckCheckTime = currentTime;
    }
  } else {
    stuckCounter = 0;
    stuckStartTime = 0;
    voltageRamping = false;
  }

  // Minimum voltage to overcome friction (only if PID output is too low)
  if (abs(error) > 3 && !voltageRamping) {
    bool movingRight = (error < 0);
    float baseFrictionVoltage = movingRight ? FRICTION_LEFT : FRICTION_RIGHT;
    float minFrictionVoltage = max(baseFrictionVoltage, 1.5f);
    if (currentPos < -1200) {
      minFrictionVoltage = max(minFrictionVoltage, 3.0f);
    }
    if (abs(totalVoltage) < minFrictionVoltage) {
      float pidMagnitude = abs(totalVoltage);
      float finalVoltage = max(pidMagnitude, minFrictionVoltage);
      totalVoltage = (error < 0) ? -finalVoltage : finalVoltage;
    }
  }

  // Prevent crossing limits before applying voltage
  if (rightPressed() && totalVoltage < 0) {
    stopMotor();
    encoder.write(UPPER_BOUND);
    lastError = 0;
    return;
  }
  if (leftPressed() && totalVoltage > 0) {
    stopMotor();
    encoder.write(LOWER_BOUND);
    lastError = 0;
    return;
  }
  // Block if past bounds
  if (currentPos < UPPER_BOUND) {
    stopMotor();
    encoder.write(UPPER_BOUND);
    lastError = 0;
    return;
  }
  if (currentPos > LOWER_BOUND) {
    stopMotor();
    encoder.write(LOWER_BOUND);
    lastError = 0;
    return;
  }
  // If at bounds and error is significant, block movement past bound
  // Small errors within deadband are already handled above
  if (currentPos == LOWER_BOUND && totalVoltage > 0 && abs(error) > DEADBAND && desiredPosition > LOWER_BOUND) {
    stopMotor();
    lastError = 0;
    return;
  }
  if (currentPos == UPPER_BOUND && totalVoltage < 0 && abs(error) > DEADBAND && desiredPosition < UPPER_BOUND) {
    stopMotor();
    lastError = 0;
    return;
  }
  
  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
  }

  lastError = error;
}

// MOTOR CONTROL
bool isSwitchEnabled() {
  return digitalRead(ON_OFF_SWITCH_PIN) == HIGH;
}

void setMotor(float voltage) {
  // Master override from flip switch
  static bool lastSwitchState = true;
  bool enabled = isSwitchEnabled();
  if (!enabled) {
    if (lastSwitchState != enabled) {
    }
    lastSwitchState = enabled;
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, 0);
    return;
  }
  if (lastSwitchState != enabled) {
  }
  lastSwitchState = enabled;

  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;
  // Prevent crossing left limit (lower bound/zero)
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwm = 0;
  }
  // Also check encoder position to prevent crossing LOWER_BOUND
  long currentPos = encoder.read();
  // Only block if past the bound, not if at it (allows reaching target at bound)
  if (currentPos > LOWER_BOUND && voltage > 0) {
    voltage = 0;
    pwm = 0;
  }
  // Right limit switch disabled during auto mode (needed for Lane 4 positioning)
  if (!autoMode && digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
    voltage = 0;
    pwm = 0;
  }
  // Also check encoder position to prevent crossing UPPER_BOUND
  // Only block if past the bound, not if at it
  if (currentPos < UPPER_BOUND && voltage < 0) {
    voltage = 0;
    pwm = 0;
  }
  if (voltage > 0) {
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, pwm);
  } else if (voltage < 0) {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
    analogWrite(MOTOR_ENA, pwm);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, 0);
  }
}

void stopMotor() {
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  analogWrite(MOTOR_ENA, 0);
}

// DRIFT MITIGATION
void mitigateDrift() {
  if (!autoMode || dynamicCalibrationActive) return;
  
  long currentPos = encoder.read();
  
  // Immediate bounds check - catch extreme encoder glitches
  if (currentPos < UPPER_BOUND) {
    encoder.write(UPPER_BOUND);
    lastDriftCheckPosition = UPPER_BOUND;
    lastKnownGoodPosition = UPPER_BOUND;
    return;
  }
  if (currentPos > LOWER_BOUND) {
    encoder.write(LOWER_BOUND);
    lastDriftCheckPosition = LOWER_BOUND;
    lastKnownGoodPosition = LOWER_BOUND;
    return;
  }
  
  // Check for large jumps (encoder glitches) - only when truly stationary
  // During leftward movement, don't over-correct small variations
  if (lastKnownGoodPosition != 0) {
    long jump = abs(currentPos - lastKnownGoodPosition);
    // Only correct large jumps when truly stationary (not during movement)
    if (jump > 150 && abs(motorVelocity) < 5) {
      encoder.write(lastKnownGoodPosition);
      currentPos = lastKnownGoodPosition;
      return;
    }
  }
  
  // Drift detection - only check when stationary to avoid interfering with movement
  bool isStationary = (currentState == CHOOSE_ACTIVE_TARGET || abs(motorVelocity) < 3);
  if (isStationary && lastDriftCheckPosition != 0) {
    long driftAmount = abs(currentPos - lastDriftCheckPosition);
    // Check direction of drift - leftward movement should increase encoder value
    long driftDirection = currentPos - lastDriftCheckPosition;
    
    // Only correct if drift is significant and in wrong direction when stationary
    if (driftAmount > MAX_DRIFT_THRESHOLD) {
      // Only rehome for extreme cases
      if (driftAmount > 500 || currentPos > 100 || currentPos < UPPER_BOUND - 100) {
        if (homeToLeftLimit()) {
          loadCalibrationFromEEPROM();
          lastDriftCheckPosition = 0;
          lastKnownGoodPosition = 0;
          return;
        }
      } else {
        // Small drift when stationary - correct it
        encoder.write(lastDriftCheckPosition);
        currentPos = lastDriftCheckPosition;
      }
    }
  }
  
  // Only update reference positions when truly stationary
  // This prevents locking in incorrect positions during movement
  if (isStationary) {
    lastDriftCheckPosition = currentPos;
    lastKnownGoodPosition = currentPos;
  }
  // Don't update during movement - wait until stationary to establish new reference
}

// POSITION VALIDATION
void validatePosition() {
  if (!autoMode || dynamicCalibrationActive) return;
  
  long currentPos = encoder.read();
  
  // Always enforce bounds
  if (currentPos < UPPER_BOUND) {
    encoder.write(UPPER_BOUND);
    currentPos = UPPER_BOUND;
    lastKnownGoodPosition = UPPER_BOUND;
    return;
  }
  if (currentPos > LOWER_BOUND) {
    encoder.write(LOWER_BOUND);
    currentPos = LOWER_BOUND;
    lastKnownGoodPosition = LOWER_BOUND;
    return;
  }
  
  // Detect large jumps - only when truly stationary
  // During leftward movement, encoder readings can vary - don't over-correct
  if (lastKnownGoodPosition != 0) {
    long positionJump = abs(currentPos - lastKnownGoodPosition);
    // Only correct large jumps when stationary (not during active movement)
    if (positionJump > 100 && abs(motorVelocity) < 5) {
      encoder.write(lastKnownGoodPosition);
      currentPos = lastKnownGoodPosition;
    }
  }
  
  // Only update reference position when stationary
  // This prevents incorrect positions from being locked in during movement
  if (abs(motorVelocity) < 5) {
    lastKnownGoodPosition = currentPos;
  }
}

// LIMIT SWITCHES
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
  }
  
  if (digitalRead(LIMIT_RIGHT) == HIGH && 
      !lane4LimitSwitchMode && 
      !lane4AtLimit &&
      currentState == MOVE_TO_TARGET &&
      abs(motorVelocity) < 10) {
    long currentPos = encoder.read();
    if (currentPos < UPPER_BOUND) {
      encoder.write(UPPER_BOUND);
      errorIntegral = 0;
    }
  }
  if (digitalRead(LIMIT_LEFT) == HIGH && 
      currentState == MOVE_TO_TARGET &&
      abs(motorVelocity) < 10) {
    long currentPos = encoder.read();
    if (currentPos > LOWER_BOUND) {
      encoder.write(LOWER_BOUND);
      errorIntegral = 0;
    }
  }
}

bool leftPressed() {
  return digitalRead(LIMIT_LEFT) == HIGH;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == HIGH;
}

// HOMING
const float HOMING_EXTRA_VOLTAGE = 0.6;     // Added on top of friction during homing
const unsigned long HOMING_HOLD_TIME = 500; // ms to hold on switch before zeroing
const int HOMING_STABLE_TICKS = 5;          // Require this many consecutive stable readings

bool homeToLeftLimit() {
  // CRITICAL: Lane positions are NEVER modified during homing
  // They remain as loaded from EEPROM
  
  if (leftPressed()) {
    // Already at limit - hold and debounce
    long lastPos = encoder.read();
    unsigned long holdStart = millis();
    int stableTicks = 0;
    float holdVoltage = max(FRICTION_RIGHT + 0.1, 2.5);  // Hold against switch

    // Hold on the switch with debounce - ensure it settles
    while (millis() - holdStart < HOMING_HOLD_TIME || stableTicks < HOMING_STABLE_TICKS) {
      setMotor(holdVoltage);  // Keep holding against switch
      delay(10);
      long pos = encoder.read();
      if (abs(pos - lastPos) <= 1) {
        stableTicks++;
      } else {
        stableTicks = 0;
        lastPos = pos;
      }
    }

    stopMotor();
    delay(200);  // Brief pause before zeroing

    // Multiple zeroing attempts to ensure encoder is properly reset
    encoder.write(0);
    delay(50);
    if (encoder.read() != 0) {
      encoder.write(0);
      delay(50);
    }
    encoder.write(0);
    delay(50);
    
    // Verify encoder is actually zeroed
    long finalPos = encoder.read();
    if (abs(finalPos) > 2) {
      encoder.write(0);
      delay(50);
    }

    // CRITICAL: Reload lane positions from EEPROM after homing
    // This ensures they're never modified
    loadCalibrationFromEEPROM();
    
    return true;
  }

  // Approach limit switch
  unsigned long startTime = millis();
  long lastPosition = encoder.read();
  unsigned long lastMoveTime = millis();
  float driveVoltage = max(FRICTION_RIGHT + HOMING_EXTRA_VOLTAGE + 0.1, 3.0);
  setMotor(driveVoltage);

  // Move toward limit switch with stuck detection
  while (!leftPressed() && (millis() - startTime) < 12000) {
    delay(10);
    
    // Check for stuck condition
    long currentPos = encoder.read();
    if (abs(currentPos - lastPosition) > 2) {
      lastMoveTime = millis();
      lastPosition = currentPos;
    } else if (millis() - lastMoveTime > 3000) {
      // Stuck - increase voltage
      driveVoltage = min(driveVoltage + 0.5, 8.0);
      setMotor(driveVoltage);
      lastMoveTime = millis();
    }
  }
  
  if (!leftPressed()) {
    // Timeout - didn't reach limit switch
    stopMotor();
    return false;
  }
  
  // Reached limit switch - now hold and debounce (same as above)
  long lastPos = encoder.read();
  unsigned long holdStart = millis();
  int stableTicks = 0;
  float holdVoltage = max(FRICTION_RIGHT + 0.1, 2.5);  // Hold against switch

  // Hold on the switch with debounce - ensure it settles
  while (millis() - holdStart < HOMING_HOLD_TIME || stableTicks < HOMING_STABLE_TICKS) {
    setMotor(holdVoltage);  // Keep holding against switch
    delay(10);
    long pos = encoder.read();
    if (abs(pos - lastPos) <= 1) {
      stableTicks++;
    } else {
      stableTicks = 0;
      lastPos = pos;
    }
  }

  stopMotor();
  delay(200);  // Brief pause before zeroing

  // Multiple zeroing attempts to ensure encoder is properly reset
  encoder.write(0);
  delay(50);
  if (encoder.read() != 0) {
    encoder.write(0);
    delay(50);
  }
  encoder.write(0);
  delay(50);
  
  // Verify encoder is actually zeroed
  long finalPos = encoder.read();
  if (abs(finalPos) > 2) {
    encoder.write(0);
    delay(50);
  }

  // CRITICAL: Reload lane positions from EEPROM after homing
  // This ensures they're never modified
  loadCalibrationFromEEPROM();
  
  return true;
}


void characterizeFriction() {
  
  stopMotor();
  delay(500);
  
  // Home to left limit first
  if (!homeToLeftLimit()) {
    return;
  }
  
  delay(500);
  
  float measurements[3];
  int measurementCount = 0;
  
  for (int attempt = 0; attempt < 3; attempt++) {
    float testVoltage = 0.5;
    bool motionDetected = false;
    
    while (!motionDetected && testVoltage < 8.0) {
      long startPos = encoder.read();
      setMotor(-testVoltage);  // Negative = RIGHT
      delay(400);
      long endPos = encoder.read();
      setMotor(0);
      delay(300);
      
      long movement = abs(endPos - startPos);
      
      if (movement > 15) {
        motionDetected = true;
        measurements[measurementCount] = testVoltage - 0.15;  // Safety margin
        measurementCount++;
        
        if (!homeToLeftLimit()) return;
        delay(1000);
        break;
      } else {
        testVoltage += 0.2;
      }
    }
  }
  
  if (measurementCount < 2) {
    return;
  }
  
  // Calculate average, rejecting outliers
  float sum = 0;
  for (int i = 0; i < measurementCount; i++) {
    sum += measurements[i];
  }
  float avg = sum / measurementCount;
  
  // Calculate standard deviation
  float variance = 0;
  for (int i = 0; i < measurementCount; i++) {
    variance += (measurements[i] - avg) * (measurements[i] - avg);
  }
  float stdDev = sqrt(variance / measurementCount);
  
  // Use median if high variance (outlier rejection)
  if (stdDev > 0.3) {
    // Simple bubble sort for median
    for (int i = 0; i < measurementCount - 1; i++) {
      for (int j = 0; j < measurementCount - i - 1; j++) {
        if (measurements[j] > measurements[j + 1]) {
          float temp = measurements[j];
          measurements[j] = measurements[j + 1];
          measurements[j + 1] = temp;
        }
      }
    }
    FRICTION_LEFT = measurements[measurementCount / 2];  // Median
  } else {
    FRICTION_LEFT = avg;
  }
  
  float driveRight = -max(FRICTION_LEFT + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
  setMotor(driveRight);
  unsigned long driveStart = millis();
  while (digitalRead(LIMIT_RIGHT) == LOW) {
    if (millis() - driveStart > 15000) {
      setMotor(0);
      return;
    }
    delay(10);
  }
  // Hold briefly on right limit
  unsigned long holdStart = millis();
  float holdRight = -max(FRICTION_LEFT, CALIBRATE_MIN_VOLTAGE - 0.5);
  while (millis() - holdStart < CALIBRATE_HOLD_TIME) {
    setMotor(holdRight);
    delay(10);
  }
  setMotor(0);
  delay(500);
  
  
  measurementCount = 0;
  for (int attempt = 0; attempt < 3; attempt++) {
    float testVoltage = 0.5;
    bool motionDetected = false;
    
    while (!motionDetected && testVoltage < 8.0) {
      long startPos = encoder.read();
      setMotor(testVoltage);  // Positive = LEFT
      delay(400);
      long endPos = encoder.read();
      setMotor(0);
      delay(300);
      
      long movement = abs(endPos - startPos);
      
      if (movement > 15) {
        motionDetected = true;
        measurements[measurementCount] = testVoltage - 0.15;
        measurementCount++;
        
        // Return to right limit
        driveRight = -max(FRICTION_LEFT + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
        setMotor(driveRight);
        driveStart = millis();
        while (digitalRead(LIMIT_RIGHT) == LOW && millis() - driveStart < 15000) {
          delay(10);
        }
        setMotor(0);
        delay(1000);
        break;
      } else {
        testVoltage += 0.2;
      }
    }
  }
  
  if (measurementCount < 2) {
    return;
  }
  
  // Calculate average with outlier rejection
  sum = 0;
  for (int i = 0; i < measurementCount; i++) {
    sum += measurements[i];
  }
  avg = sum / measurementCount;
  
  variance = 0;
  for (int i = 0; i < measurementCount; i++) {
    variance += (measurements[i] - avg) * (measurements[i] - avg);
  }
  stdDev = sqrt(variance / measurementCount);
  
  if (stdDev > 0.3) {
    // Median
    for (int i = 0; i < measurementCount - 1; i++) {
      for (int j = 0; j < measurementCount - i - 1; j++) {
        if (measurements[j] > measurements[j + 1]) {
          float temp = measurements[j];
          measurements[j] = measurements[j + 1];
          measurements[j + 1] = temp;
        }
      }
    }
    FRICTION_RIGHT = measurements[measurementCount / 2];
  } else {
    FRICTION_RIGHT = avg;
  }
  
  // Update adaptive friction values
  adaptiveFrictionLeft = FRICTION_LEFT;
  adaptiveFrictionRight = FRICTION_RIGHT;
  
  // Save to EEPROM
  saveFrictionToEEPROM();
  
  Serial.print(FRICTION_LEFT, 1);
  Serial.print(F(" "));
  Serial.println(FRICTION_RIGHT, 1);
  
  // Return home - lane positions preserved via loadCalibrationFromEEPROM in homeToLeftLimit
  homeToLeftLimit();
}

// EEPROM
void loadCalibrationFromEEPROM() {
  // CRITICAL: This function preserves lane positions from EEPROM
  // It should be called after any homing or range finding operation
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) {
    return;  // Don't print during auto operations
  }

  EEPROM.get(EEPROM_KP, KP);
  EEPROM.get(EEPROM_KI, KI);
  EEPROM.get(EEPROM_KD, KD);
  EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);

  adaptiveFrictionLeft = FRICTION_LEFT;
  adaptiveFrictionRight = FRICTION_RIGHT;

  // CRITICAL: Always reload lane positions from EEPROM
  // This ensures they're never modified by homing or range finding
  for (int i = 0; i < 4; i++) {
    long v;
    EEPROM.get(EEPROM_LANES_BASE + i * sizeof(long), v);
    targetPositions[i] = v;
  }

  TARGET_1_POSITION = targetPositions[0];
  TARGET_2_POSITION = targetPositions[1];
  TARGET_3_POSITION = targetPositions[2];
  TARGET_4_POSITION = targetPositions[3];
  WAIT_POSITION = TARGET_3_POSITION;
}

void saveTargetsToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * sizeof(long), targetPositions[i]);
  }
}

void savePIDToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
}

void saveFrictionToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
}

// MANUAL CALIBRATION
void manualCalibration() {
  Serial.println(F("\nMAN CAL"));
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  Serial.println(F("R/L/S 1-4=Save Q=Quit\n"));
  
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();
      
      cmd = toupper(cmd);
      
      long pos = encoder.read();
      
      switch (cmd) {
        case 'R':
          setMotor(-3.5);
          break;
        case 'L':
          setMotor(3.5);
          break;
        case 'S':
          stopMotor();
          Serial.println(pos);
          break;
          
        case '1':
          stopMotor();
          TARGET_1_POSITION = pos;
          targetPositions[0] = pos;
          Serial.println(TARGET_1_POSITION);
          break;
          
        case '2':
          stopMotor();
          TARGET_2_POSITION = pos;
          targetPositions[1] = pos;
          Serial.println(TARGET_2_POSITION);
          break;
          
        case '3':
          stopMotor();
          TARGET_3_POSITION = pos;
          targetPositions[2] = pos;
          WAIT_POSITION = TARGET_3_POSITION;
          Serial.println(TARGET_3_POSITION);
          break;
          
        case '4':
          stopMotor();
          TARGET_4_POSITION = pos;
          targetPositions[3] = pos;
          Serial.println(TARGET_4_POSITION);
          break;
          
        case 'Q':
          stopMotor();
          Serial.print(TARGET_1_POSITION);
          Serial.print(F(" "));
          Serial.print(TARGET_2_POSITION);
          Serial.print(F(" "));
          Serial.print(TARGET_3_POSITION);
          Serial.print(F(" "));
          Serial.println(TARGET_4_POSITION);

          saveTargetsToEEPROM();

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

// COMMANDS
void processCommand() {
  char cmd = Serial.read();
  while (Serial.available()) Serial.read();
  
  cmd = toupper(cmd);
  
  switch (cmd) {
    case 'G':
      if (!autoMode) {
        Serial.println(F("\nAUTO"));

          stopMotor();
          delay(200);
          if (homeToLeftLimit()) {
          encoder.write(0);
          delay(50);
          if (encoder.read() != 0) {
            encoder.write(0);
            delay(50);
            encoder.write(0);
          }
          autoMode = true;
          systemEnabled = true;
          rangeFindingComplete = true;
          sensorCalibrated = false;
          dynamicCalibrationActive = false;
          // Initialize drift check
          lastDriftCheckPosition = encoder.read();
          lastKnownGoodPosition = encoder.read();
          lastDriftCheckTime = millis();
          lastPositionValidationTime = millis();
          
          currentState = CALIBRATE;
          errorIntegral = 0;
          lastError = 0;
          lastPrintTime = 0;
          moveStartTime = 0;
          targetReached = false;
          stuckCounter = 0;
          positionRetryCount = 0;
          fineAdjustmentActive = false;
          fineAdjustmentCount = 0;  // Reset counter
          lastFineAdjustmentTime = 0;  // Reset timestamp
          stuckStartTime = 0;
          voltageRamping = false;
          lane4LimitSwitchMode = false;
          lane4AtLimit = false;
          lane4LimitTime = 0;
          
          activeTargetIndex = -1;
          previousTargetIndex = -1;
          WAIT_POS = true;
          previousZombieDistance = 1.0;
          targetCommitTime = 0;
          for (int i = 0; i < 4; i++) {
            ProxSensors[i].hitDetected = false;
            ProxSensors[i].hitTime = 0;
          }
          
          targetHitTime = 0;

        } else {
        }
      }
      break;
    
    case 'S':
      Serial.println(F("\nSTOP"));
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
    
    case 'F':
      if (!autoMode) {
        characterizeFriction();
      } else {
      }
      break;

    case 'R':
      adaptiveFrictionVoltage = 0;
      adaptiveLearning = false;
      adaptiveLearned = false;
      rangeFindingComplete = false;
      sensorCalibrated = false;
      adaptiveFrictionLeft = FRICTION_LEFT;
      adaptiveFrictionRight = FRICTION_RIGHT;
      Serial.println(F("Reset"));
      break;

    case 'L':
      loadCalibrationFromEEPROM();
      break;

    case 'W':
      saveTargetsToEEPROM();
      savePIDToEEPROM();
      saveFrictionToEEPROM();
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
  stuckCounter = 0;  // Reset stuck detection for new movement
  positionRetryCount = 0;  // Reset retry counter for new movement
  stuckStartTime = 0;  // Reset stuck tracking
  voltageRamping = false;  // Reset voltage ramping
  
  long currentPos = encoder.read();
  long error = desiredPosition - currentPos;
  
  Serial.print(currentPos);
  Serial.print(F(" "));
  Serial.print(desiredPosition);
  Serial.print(F(" "));
  Serial.println(error);
}

// DISPLAY
void printWelcome() {
}

void printHelp() {
  Serial.println(F("\nCmds: C Z G S 1-4 P D M R L W H F"));
}

void printCompactStatus() {
  Serial.print(encoder.read());
  Serial.print(F(" "));
  Serial.print(desiredPosition);
  Serial.print(F(" "));
  Serial.print(activeTargetIndex >= 0 ? activeTargetIndex + 1 : 0);
  for (int i = 0; i < 4; i++) {
    Serial.print(F(" "));
    Serial.print((int)((1.0 - zombieDistances[i]) * 100));
  }
  Serial.println();
}

void printStatus() {
  long p = encoder.read();
  Serial.print(autoMode ? F("A") : F("M"));
  Serial.print(F(" "));
  Serial.print(p);
  Serial.print(F(" "));
  Serial.print(desiredPosition);
  Serial.print(F(" "));
  Serial.print((int)(desiredPosition - p));
  Serial.print(F(" "));
  Serial.print((int)motorVelocity);
  Serial.print(F(" "));
  Serial.print(activeTargetIndex >= 0 ? activeTargetIndex + 1 : 0);
  Serial.println();
}

void printAllSensors() {
  Serial.print(encoder.read());
  Serial.print(F(" "));
  Serial.print(leftPressed() ? 1 : 0);
  Serial.print(rightPressed() ? 1 : 0);
  for (int i = 0; i < 4; i++) {
    Serial.print(F(" "));
    Serial.print(analogRead(ProxSensors[i].pin));
  }
  Serial.println();
}

void continuousMonitor() {
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  while (!Serial.available()) {
    updateAllSensors();
    Serial.print(encoder.read());
    Serial.print(F(" "));
    Serial.print(leftPressed() ? 1 : 0);
    Serial.print(rightPressed() ? 1 : 0);
    for (int i = 0; i < 4; i++) {
      Serial.print(F(" "));
      Serial.print((int)ProxSensors[i].currVal);
    }
    Serial.println();
    delay(100);
  }
  while (Serial.available()) Serial.read();
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
