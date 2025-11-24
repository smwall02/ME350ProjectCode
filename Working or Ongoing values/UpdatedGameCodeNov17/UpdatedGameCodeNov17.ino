// ME 350 - Plants vs Zombies - COMPETITION CODE

#include <Encoder.h>
#include <EEPROM.h>

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
long LOWER_BOUND = 0;      // Fixed: Left limit position (home)
long UPPER_BOUND = -1424;  // Fixed: Right limit position (from calibration)

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

// Fine positioning
long previousMoveStartPosition = 0;
bool fineAdjustmentActive = false;
long fineAdjustmentTarget = 0;
const int FINE_ADJUSTMENT_AMOUNT = 2;
float previousZombieDistance = 1.0;
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

  // Momentum compensation
  const int RIGHTWARD_DRIFT_OFFSET = 2;
  const int LEFTWARD_DRIFT_OFFSET = 2;

// MOTION CONTROL
long desiredPosition = 0;
float errorIntegral = 0;
float lastError = 0;
float motorVelocity = 0;
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
  Serial.println(F("\nSENSOR CALIBRATION"));
  Serial.println(F("10 seconds...\n"));
  
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
  
  Serial.println(F("\nCALIBRATION COMPLETE"));
  
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
        Serial.println(F(" WARN"));
        allValid = false;
      } else {
        Serial.println(F(" OK"));
      }
    
    ProxRange[i][0] = dynamicMax[i];
    ProxRange[i][1] = dynamicMin[i];
  }
  
  if (allValid) {
    sensorCalibrated = true;
    Serial.println(F("\nSensors OK"));
    Serial.println(F("Rehoming...\n"));
    if (homeToLeftLimit()) {
      Serial.println(F("Rehomed\n"));
    } else {
      Serial.println(F("Rehome failed\n"));
    }
  } else {
    Serial.println(F("\nSome sensors small range"));
    Serial.println(F("Rehoming...\n"));
    sensorCalibrated = true;
    if (homeToLeftLimit()) {
      Serial.println(F("Rehomed\n"));
    } else {
      Serial.println(F("Rehome failed\n"));
    }
  }
}

void printCalibrationProgress() {
  unsigned long elapsed = millis() - calibrationStartTime;
  unsigned long remaining = DYNAMIC_CALIBRATION_TIME - elapsed;
  
  Serial.print(F("Cal: "));
  Serial.print(remaining / 1000);
  Serial.print(F("s | "));
  
  for (int i = 0; i < 4; i++) {
    int range = dynamicMax[i] - dynamicMin[i];
    Serial.print(range);
    Serial.print(F(" "));
  }
  Serial.println();
}

// STATE MACHINE
void runStateMachine() {
  switch (currentState) {
    
    case CALIBRATE:
      desiredPosition = LOWER_BOUND;

      if (!dynamicCalibrationActive && sensorCalibrated) {
        Serial.println(F("State: TRACKING\n"));
        currentState = CHOOSE_ACTIVE_TARGET;
        systemEnabled = true;
      }
      else if (!dynamicCalibrationActive && !sensorCalibrated) {
        // Skip range finding - use fixed bounds, go directly to sensor calibration
        Serial.println(F("State: SENSOR CAL\n"));
        rangeFindingComplete = true;  // Mark as complete since we're using fixed values
        startDynamicCalibration();
        desiredPosition = LOWER_BOUND;
        systemEnabled = true;
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      activeTargetIndex = -1;
      closestZombieDist = 2.0;

      // Track position
      previousMoveStartPosition = encoder.read();

      for (int i = 0; i < 4; i++) {
        // Only target forward zombies
        if (ProxSensors[i].direction == FORWARD &&
            zombieDistances[i] < closestZombieDist) {
          closestZombieDist = zombieDistances[i];
          activeTargetIndex = i;
        }
      }
      
      if (activeTargetIndex >= 0) {
        activeTargetPosition = targetPositions[activeTargetIndex];
        WAIT_POS = false;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        previousZombieDistance = zombieDistances[activeTargetIndex];
        
        // Lane 4: limit switch mode
        if (activeTargetIndex == 3) {
          lane4LimitSwitchMode = true;
          lane4AtLimit = false;
          lane4LimitTime = 0;
          Serial.println(F("Lane 4: limit switch mode"));
        } else {
          lane4LimitSwitchMode = false;
          lane4AtLimit = false;
        }
        
        int percentToPhoto = (int)((1.0 - zombieDistances[activeTargetIndex]) * 100);
        
        if (activeTargetIndex != 3) {
        Serial.print(F("Target: L"));
        Serial.print(activeTargetIndex + 1);
        Serial.print(F(" ("));
        Serial.print(percentToPhoto);
        Serial.println(F("%)"));
        }
        
        previousTargetIndex = activeTargetIndex;
        targetHitTime = 0;
        
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        lane4LimitSwitchMode = false;
        lane4AtLimit = false;
        Serial.println(F("No targets, wait pos"));
      }
      
      // Lane 4: go to limit first
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
      currentState = MOVE_TO_TARGET;
      break;
    
    case MOVE_TO_TARGET:
      long currentPos = encoder.read();
      
      // Clamp encoder at right limit - prevent reading beyond UPPER_BOUND
      // Always clamp if reading beyond UPPER_BOUND to prevent drift
      if (currentPos < UPPER_BOUND) {
        // Always clamp if beyond limit - prevents encoder drift
        encoder.write(UPPER_BOUND);
        currentPos = UPPER_BOUND;
      }
      
      // Lane 4: limit switch approach
      if (lane4LimitSwitchMode && activeTargetIndex == 3) {
        if (!lane4AtLimit) {
          if (rightPressed()) {
            lane4AtLimit = true;
            lane4LimitTime = millis();
            Serial.println(F("Lane 4: at limit"));
            encoder.write(UPPER_BOUND);
            currentPos = UPPER_BOUND;
          }
          desiredPosition = UPPER_BOUND;
        }
        else if (millis() - lane4LimitTime < LANE4_LIMIT_HOLD_TIME) {
          desiredPosition = UPPER_BOUND;
          // Continuously clamp encoder while at limit
          if (rightPressed() || currentPos < UPPER_BOUND) {
            encoder.write(UPPER_BOUND);
            currentPos = UPPER_BOUND;
          }
        }
        else if (currentPos < (UPPER_BOUND + LANE4_BACKOFF_DISTANCE)) {
          desiredPosition = UPPER_BOUND + LANE4_BACKOFF_DISTANCE;
        }
        else {
          desiredPosition = activeTargetPosition;
          lane4LimitSwitchMode = false;
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
      
      // Prevent corrections beyond right limit - if at limit and trying to move right, stop
      if (rightPressed() && error < 0) {
        stopMotor();
        encoder.write(UPPER_BOUND);
        return;
      }
      
      // Safety check - disabled during auto mode (right limit needed for Lane 4)
      if (!autoMode && !lane4LimitSwitchMode && currentPos < UPPER_BOUND - 50) {
        Serial.println(F("Near right limit"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
  // Hit detection - skip during lane 4 limit approach
      if (activeTargetIndex >= 0 && !WAIT_POS && !lane4LimitSwitchMode) {
        int targetDirection = ProxSensors[activeTargetIndex].direction;
        bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
        unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
        
        // Velocity-based hit detection
        if (hitDetected && hitTime > 0) {
          // Confirm hit
          if (millis() - hitTime >= MIN_HIT_TIME) {
            Serial.println(F("Target HIT"));
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
            Serial.println(F("Target HIT (dir)"));
            targetHitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        } else {
          targetHitTime = 0;  // Reset if not consistently backward
        }
        
        // Switch on velocity change
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
          // Switch immediately
          Serial.println(F("Target stopped"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }
        
        // Switch if retreated far
        if (targetDirection == BACKWARD &&
            zombieDistances[activeTargetIndex] > 0.80) {
          Serial.println(F("Target retreated"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }

        // Check for closer threat
        for (int i = 0; i < 4; i++) {
          if (i != activeTargetIndex &&
              ProxSensors[i].direction == FORWARD &&
              zombieDistances[i] < zombieDistances[activeTargetIndex] - 0.20) {
            Serial.print(F("Closer: L"));
            Serial.println(i + 1);
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
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
        Serial.println(F("No threats"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }

      // Check arrival against current desired position (may be fine-adjusted)
      long errorToCurrentTarget = desiredPosition - currentPos;
      // Also check against original target for fine adjustment logic
      long errorToOriginalTarget = activeTargetPosition - currentPos;
      
      // Early fine adjustment - prevent loops when stuck at lane 4
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
      
      // Early fine adjustment - DISABLED for Lane 4 to prevent oscillation
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
        Serial.print(F("Fine adj: err="));
        Serial.print(errorToOriginalTarget);
        Serial.print(F(" ("));
        Serial.print(fineAdjustmentCount);
        Serial.println(F("/3)"));
        
        fineAdjustmentActive = true;
        desiredPosition = fineAdjustmentTarget;
        arrivalTime = millis();
      }
      
      if (abs(errorToCurrentTarget) <= TARGET_BAND) {
        if (WAIT_POS) {
          // At wait position
          if (hasForwardZombie) {
            Serial.println(F("New threat"));
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
              Serial.println(F("Target HIT"));
              ProxSensors[activeTargetIndex].hitDetected = false;
              ProxSensors[activeTargetIndex].hitTime = 0;
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            // Check retreat
            else if (ProxSensors[activeTargetIndex].direction == BACKWARD) {
              Serial.println(F("Target retreating"));
              currentState = CHOOSE_ACTIVE_TARGET;
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
                Serial.print(F("Fine adj: err="));
                Serial.print(errorToOriginalTarget);
                Serial.print(F(" ("));
                Serial.print(fineAdjustmentCount);
                Serial.println(F("/3)"));
                
                fineAdjustmentActive = true;
                desiredPosition = fineAdjustmentTarget;
                arrivalTime = millis();
              }
            }
            
            // Continuous fine adjustment - DISABLED for Lane 4 to prevent oscillation
            else if (activeTargetIndex != 3 &&
                     ProxSensors[activeTargetIndex].direction == FORWARD && 
                     fineAdjustmentActive &&
                     fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
                     (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
                     abs(motorVelocity) < 25 &&
                     abs(errorToCurrentTarget) > 5 &&
                     !ProxSensors[activeTargetIndex].hitDetected &&
                     (millis() - arrivalTime) >= 100) {
              
              fineAdjustmentTarget = activeTargetPosition;
              
              fineAdjustmentCount++;
              lastFineAdjustmentTime = millis();
              Serial.print(F("Fine adj cont: err="));
              Serial.print(errorToCurrentTarget);
              Serial.print(F(" ("));
              Serial.print(fineAdjustmentCount);
              Serial.println(F("/3)"));
              
              desiredPosition = fineAdjustmentTarget;
              arrivalTime = millis();
            }
            if (activeTargetIndex >= 0) {
              previousZombieDistance = zombieDistances[activeTargetIndex];
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

// MOTION CONTROL
void runMotionControl() {
  long currentPosition = encoder.read();
  
  // Clamp encoder reading at right limit switch - prevent reading beyond UPPER_BOUND
  // This prevents the encoder from counting beyond the physical limit
  // Always clamp if reading beyond UPPER_BOUND (encoder can drift due to mechanical play)
  if (currentPosition < UPPER_BOUND) {
    // Always clamp if beyond limit - prevents encoder drift
    encoder.write(UPPER_BOUND);
    currentPosition = UPPER_BOUND;
  }
  
  long adjustedDesiredPosition = desiredPosition;
  // Disable momentum compensation when very close to target to prevent oscillation
  long errorToTarget = abs(desiredPosition - currentPosition);
  if (!fineAdjustmentActive && errorToTarget > 10) {
    if (currentPosition > desiredPosition) {
      adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
    } else if (currentPosition < desiredPosition) {
      adjustedDesiredPosition = desiredPosition - LEFTWARD_DRIFT_OFFSET;
    }
  }
  
  float error = adjustedDesiredPosition - currentPosition;
  
  // Prevent any corrections that would try to move right (more negative) when at right limit
  if (rightPressed() && error < 0) {
    stopMotor();
    encoder.write(UPPER_BOUND);
    return;
  }
  
  if (currentState == MOVE_TO_TARGET && autoMode) {
    // Check drift
    if (currentPosition > 50) {
      Serial.println(F("Position drift"));
      stopMotor();
      return;
    }
  }
  
  if (dynamicCalibrationActive) {
    desiredPosition = LOWER_BOUND;
    adjustedDesiredPosition = LOWER_BOUND;
    error = desiredPosition - currentPosition;
    if (abs(error) > 10) {
      setMotor(constrain(error * 0.05, -2.0, 2.0));
    } else {
      stopMotor();
    }
    return;
  }
  
  float originalError = desiredPosition - currentPosition;
  
  // Enhanced deadband for Lane 4 to prevent oscillation
  // Lane 4 is more prone to oscillation, so use larger deadband
  bool isLane4 = (activeTargetIndex == 3);
  int deadbandSize = isLane4 ? 5 : 3;  // Larger deadband for Lane 4
  int hysteresisThreshold = isLane4 ? 8 : 5;  // Larger hysteresis for Lane 4
  
  // For Lane 4, if we're at the target position, completely stop all corrections
  if (isLane4 && abs(originalError) <= deadbandSize) {
    stopMotor();
    errorIntegral = 0;
    adaptiveLearning = false;
    lastError = 0;
    stuckCounter = 0;
    stuckStartTime = 0;
    voltageRamping = false;
    if (!targetReached) {
      targetReached = true;
    }
    return;
  }
  
  static bool inDeadband = false;
  static unsigned long lastMoveStart = 0;
  
  // Reset deadband flag when starting a new move
  if (moveStartTime != lastMoveStart) {
    inDeadband = false;
    lastMoveStart = moveStartTime;
  }
  
  if (abs(originalError) <= deadbandSize) {
    inDeadband = true;
    stopMotor();
    errorIntegral = 0;
    adaptiveLearning = false;
    lastError = 0;  // Reset lastError to prevent derivative spikes
    if (!targetReached) {
      targetReached = true;
    }
    return;
  }
  // Hysteresis: only resume correction if error exceeds threshold
  if (inDeadband && abs(originalError) > hysteresisThreshold) {
    inDeadband = false;
  }
  if (inDeadband) {
    stopMotor();
    return;
  }
  
  bool atTarget = fineAdjustmentActive ? (abs(originalError) <= 1) : (abs(originalError) <= TARGET_BAND);
  
  if (atTarget) {
    stopMotor();
    errorIntegral = 0;
    adaptiveLearning = false;
    if (!targetReached) {
      targetReached = true;
      Serial.print(F("At "));
      Serial.print(currentPosition);
      Serial.print(F(" err="));
      Serial.println(originalError);
    }
    return;
  }

  targetReached = false;

  // Retry logic - skip if within deadband (larger for Lane 4)
  int deadbandSize = isLane4 ? 5 : 3;
  if (abs(originalError) > deadbandSize && abs(originalError) > RETRY_ERROR_THRESHOLD && abs(originalError) < 50) {
    // Check if stuck
    if (millis() - moveStartTime > 300 && positionRetryCount < MAX_POSITION_RETRIES) {
      positionRetryCount++;
      Serial.print(F("Stuck err="));
      Serial.println(abs(originalError));

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
    lastAdaptivePosition = currentPosition;
    adaptiveStartTime = millis();
    
    // Direction
    bool movingRight = (error < 0);
    Serial.print(F("Learning friction "));
    Serial.println(movingRight ? F("R") : F("L"));
  }
  
  if (adaptiveLearning) {
    // Check movement
    long positionChange = abs(currentPosition - lastAdaptivePosition);
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
      
      Serial.print(F("Friction: "));
      Serial.print(adaptiveFrictionVoltage, 2);
      Serial.println(F("V"));
      
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
        lastAdaptivePosition = currentPosition;
      if (adaptiveFrictionVoltage > 4.5) {
        Serial.println(F("Max friction"));
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
      Serial.println(F("Friction timeout"));
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
  
  // Skip PID correction if error is within deadband (larger for Lane 4)
  int deadbandSize = isLane4 ? 5 : 3;
  if (abs(error) <= deadbandSize) {
    stopMotor();
    lastError = 0;  // Reset to prevent derivative spikes
    return;
  }
  
  float dt = CONTROL_PERIOD / 1000.0;
  
  // PID CONTROL - uses EEPROM values
  
  // Adaptive PID gains - reduce aggressiveness when close to target, especially for Lane 4
  if (abs(error) > 1000) {
    KP_active = KP * 3.5;
    KI_active = 0;
    KD_active = KD * 0.8;
    errorIntegral = 0;
  } else if (abs(error) > 500) {
    KP_active = KP * 3.0;
    KI_active = 0;
    KD_active = KD * 0.75;
    errorIntegral = 0;
  } else if (abs(error) > 300) {
    KP_active = KP * 2.5;
    KI_active = 0;
    KD_active = KD * 0.7;
    errorIntegral = 0;
  } else if (abs(error) > 50) {
    KP_active = KP * 2.0;
    KI_active = KI * 0.8;
    KD_active = KD * 1.2;
  } else if (abs(error) > 10) {
    // Reduced gains when close to target to prevent oscillation
    if (isLane4) {
      // Lane 4: even more conservative gains
      KP_active = KP * 0.5;
      KI_active = KI * 0.2;
      KD_active = KD * 2.0;  // Very high damping for Lane 4
    } else {
      KP_active = KP * 1.0;
      KI_active = KI * 0.5;
      KD_active = KD * 1.5;
    }
    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  } else {
    // Very close - minimal gains, especially for Lane 4
    if (isLane4) {
      KP_active = KP * 0.2;  // Very low for Lane 4
      KI_active = 0;
      KD_active = KD * 3.0;  // Very high damping
    } else {
      KP_active = KP * 0.5;
      KI_active = 0;
      KD_active = KD * 2.0;
    }
    errorIntegral = 0;  // Reset integral
  }
  
  float errorDerivative = (error - lastError) / dt;
  
  float pidVoltage = (KP_active * error) +
                     (KI_active * errorIntegral) +
                     (KD_active * errorDerivative);
  
  // Momentum compensation - disabled when very close to target, especially for Lane 4
  float momentumCompensation = 1.0;
  if (isLane4 && abs(error) <= 20) {
    // Lane 4: disable momentum compensation when close to prevent oscillation
    momentumCompensation = 1.0;
  } else if (abs(error) <= 10) {
    // Disable momentum compensation when very close to prevent oscillation
    momentumCompensation = 1.0;
  } else if (abs(error) > 200) {
    momentumCompensation = 1.0;
  } else if (abs(error) < 100 && abs(motorVelocity) > 50) {
    float velocityFactor = constrain(abs(motorVelocity) / 200.0, 0.0, 1.0);
    momentumCompensation = 1.0 - (velocityFactor * 0.4);
    momentumCompensation = max(momentumCompensation, 0.6);
  } else if (abs(error) < 50 && abs(motorVelocity) > 30) {
    float velocityFactor = constrain(abs(motorVelocity) / 100.0, 0.0, 1.0);
    momentumCompensation = 1.0 - (velocityFactor * 0.5);
    momentumCompensation = max(momentumCompensation, 0.5);
  }
  
  pidVoltage *= momentumCompensation;

  // FRICTION COMPENSATION - higher on right side (lane 4)
  // Skip friction compensation if within deadband (larger for Lane 4)
  int deadbandSize = isLane4 ? 5 : 3;
  float frictionComp = 0;
  if (abs(error) > deadbandSize) {
    bool movingTowardMoreNegative = (error < 0);
    float baseFriction = movingTowardMoreNegative ? adaptiveFrictionLeft : adaptiveFrictionRight;
    float positionFrictionBoost = 1.0;
    if (currentPosition < -1200) {
      float lane4Factor = (currentPosition + 1200) / -224.0;
      lane4Factor = constrain(lane4Factor, 0.0, 1.0);
      positionFrictionBoost = 2.0 + (lane4Factor * 2.0);
    } else if (currentPosition < -1000) {
      float rightSideFactor = (currentPosition + 1000) / -200.0;
      rightSideFactor = constrain(rightSideFactor, 0.0, 1.0);
      positionFrictionBoost = 1.5 + (rightSideFactor * 0.5);
    } else if (currentPosition < -500) {
      float mediumRightFactor = (currentPosition + 500) / -500.0;
      mediumRightFactor = constrain(mediumRightFactor, 0.0, 1.0);
      positionFrictionBoost = 1.0 + (mediumRightFactor * 0.5);
    }
    
    float frictionScale = 1.0;
    float absError = abs(error);
    if (currentPosition < -1200) {
      if (absError < 3) frictionScale = 0.3;
      else if (absError < 10) frictionScale = 0.6;
      else if (absError < 30) frictionScale = 0.85;
      else if (absError < 100) frictionScale = 0.95;
    } else {
      if (absError < 3) frictionScale = 0.1;
      else if (absError < 10) frictionScale = 0.3;
      else if (absError < 30) frictionScale = 0.6;
      else if (absError < 100) frictionScale = 0.85;
    }
    
    if (abs(motorVelocity) > 10) {
      float velocityFactor = constrain(abs(motorVelocity) / 100.0, 0.0, 1.0);
      if (currentPosition < -1200) {
        frictionScale *= (1.0 - velocityFactor * 0.2);
      } else {
        frictionScale *= (1.0 - velocityFactor * 0.4);
      }
    }
    
    float totalFriction = baseFriction * positionFrictionBoost * frictionScale;
    frictionComp = (error < 0) ? -totalFriction : totalFriction;
  }

  float velocityFF = 0;
  // Fine adjustment boost - disabled when very close, especially for Lane 4
  float fineAdjustmentBoost = 0;
  if (fineAdjustmentActive && !isLane4 && abs(error) > 3 && abs(error) <= 5 && abs(motorVelocity) < 15) {
    float boostMultiplier = 1.2;
    fineAdjustmentBoost = error * boostMultiplier;
    fineAdjustmentBoost = constrain(fineAdjustmentBoost, -2.0, 2.0);
  }

  // Total voltage
  float totalVoltage = pidVoltage + frictionComp + velocityFF + fineAdjustmentBoost;

  // Voltage capping - 9V nominal, limit large moves to prevent slamming
  float voltageLimit = 9.0;
  long absErr = abs(error);
  if (absErr > 1000) voltageLimit = 7.0;
  else if (absErr > 800) voltageLimit = 7.5;
  else if (absErr > 500) voltageLimit = 8.0;
  else if (absErr > 300) voltageLimit = 7.5;
  else if (absErr > 100) voltageLimit = 7.0;
  else if (absErr > 50) voltageLimit = 6.0;
  else voltageLimit = 5.0;

  totalVoltage = constrain(totalVoltage, -voltageLimit, voltageLimit);

  // Limit switch protection - slow down before hitting limits
  if (currentState == MOVE_TO_TARGET && autoMode) {
    bool isLargeMove = (absErr > 1000);
    int leftSlowdownDistance = isLargeMove ? 200 : 100;
    if (error > 0 && currentPosition > -leftSlowdownDistance) {
      float proximityFactor = (currentPosition + leftSlowdownDistance) / leftSlowdownDistance;
      proximityFactor = constrain(proximityFactor, 0.0, 1.0);
      float minVoltage = isLargeMove ? 0.2 : 0.3;
      float maxVoltage = isLargeMove ? 0.5 : 0.7;
      totalVoltage *= (minVoltage + (proximityFactor * (maxVoltage - minVoltage)));
      if (currentPosition > 0) {
        stopMotor();
        return;
      }
    }
    // Right limit slowdown disabled during Lane 4 limit switch mode (need to reach limit)
    if (!lane4LimitSwitchMode) {
      int rightSlowdownDistance = isLargeMove ? 200 : 100;
      if (error < 0 && currentPosition < (UPPER_BOUND + rightSlowdownDistance)) {
        float distanceFromLimit = currentPosition - UPPER_BOUND;
        float proximityFactor = (distanceFromLimit + rightSlowdownDistance) / rightSlowdownDistance;
        proximityFactor = constrain(proximityFactor, 0.0, 1.0);
        float minVoltage = isLargeMove ? 0.2 : 0.3;
        float maxVoltage = isLargeMove ? 0.5 : 0.7;
        totalVoltage *= (minVoltage + (proximityFactor * (maxVoltage - minVoltage)));
      }
    }
  }

  // Anti-windup on zero crossing
  if ((error != 0) && (error * lastError < 0)) {
    errorIntegral *= 0.5;
  }

  // Stuck detection with voltage ramping
  // Skip stuck detection if within deadband (larger for Lane 4)
  int deadbandSize = isLane4 ? 5 : 3;
  unsigned long currentTime = millis();
  if (abs(originalError) > deadbandSize) {
    if (currentTime - lastStuckCheckTime >= 150) {
      if (abs(currentPosition - lastStuckCheckPos) < 2) {
        stuckCounter++;
        if (stuckCounter == 1) {
          stuckStartTime = currentTime;
          voltageRamping = false;
        }
        if (stuckCounter >= 2) {
          voltageRamping = true;
          bool movingRight = (error < 0);
          float baseFrictionVoltage = movingRight ? adaptiveFrictionLeft : adaptiveFrictionRight;
          float minFrictionVoltage = max(baseFrictionVoltage, 2.0f);
          if (currentPosition < -1200) {
            minFrictionVoltage = max(minFrictionVoltage, 3.5f);
          }
          unsigned long stuckDuration = currentTime - stuckStartTime;
          float rampVoltage = minFrictionVoltage;
          if (stuckDuration > 1200) rampVoltage = minFrictionVoltage + 1.5f;
          else if (stuckDuration > 800) rampVoltage = minFrictionVoltage + 1.0f;
          else if (stuckDuration > 400) rampVoltage = minFrictionVoltage + 0.5f;
          rampVoltage = min(rampVoltage, voltageLimit);
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
      lastStuckCheckPos = currentPosition;
      lastStuckCheckTime = currentTime;
    }
  } else {
    stuckCounter = 0;
    stuckStartTime = 0;
    voltageRamping = false;
  }

  // Ensure minimum voltage for error correction
  // Skip if within deadband (larger for Lane 4)
  int deadbandSize = isLane4 ? 5 : 3;
  if (abs(originalError) > deadbandSize && !voltageRamping) {
    bool movingRight = (error < 0);
    float baseFrictionVoltage = movingRight ? adaptiveFrictionLeft : adaptiveFrictionRight;
    float minFrictionVoltage = max(baseFrictionVoltage, 1.5f);
    if (currentPosition < -1200) {
      minFrictionVoltage = max(minFrictionVoltage, 3.0f);
    }
    if (abs(totalVoltage) < minFrictionVoltage) {
      float pidMagnitude = abs(totalVoltage);
      float finalVoltage = max(pidMagnitude, minFrictionVoltage);
      totalVoltage = (error < 0) ? -finalVoltage : finalVoltage;
    }
  }

  // Final check: prevent moving right (more negative) when at right limit
  if (rightPressed() && totalVoltage < 0) {
    stopMotor();
    encoder.write(UPPER_BOUND);
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
      Serial.println(F("Flip switch OFF - motor disabled"));
    }
    lastSwitchState = enabled;
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, 0);
    return;
  }
  if (lastSwitchState != enabled) {
    Serial.println(F("Flip switch ON - motor enabled"));
  }
  lastSwitchState = enabled;

  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwm = 0;
  }
  // Right limit switch disabled during auto mode (needed for Lane 4 positioning)
  if (!autoMode && digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
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
    Serial.println(F("Recal at left"));
  }
  
  // Right limit switch: clamp encoder if accidentally hit (not during Lane 4)
  // During Lane 4, the limit switch is used intentionally, so don't interfere
  if (digitalRead(LIMIT_RIGHT) == HIGH && 
      !lane4LimitSwitchMode && 
      !lane4AtLimit &&
      currentState == MOVE_TO_TARGET &&
      abs(motorVelocity) < 10) {
    // Accidentally hit right limit - clamp encoder to prevent drift
    long currentPos = encoder.read();
    if (currentPos < UPPER_BOUND) {
      encoder.write(UPPER_BOUND);
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
bool homeToLeftLimit() {
  Serial.println(F("\nHoming..."));

  if (leftPressed()) {
    Serial.println(F("At limit"));

    // Hold at limit
    long lastPos = encoder.read();
    unsigned long holdStart = millis();
    int stableTicks = 0;
    float holdVoltage = max(FRICTION_RIGHT, CALIBRATE_MIN_VOLTAGE - 0.5);

    while (millis() - holdStart < CALIBRATE_HOLD_TIME || stableTicks < CALIBRATE_STABLE_TICKS) {
      setMotor(holdVoltage);
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
    delay(100);

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
      Serial.print(F("⚠️  Warning: Encoder not zeroed, reading: "));
      Serial.println(finalPos);
      encoder.write(0);  // Try one more time
      delay(50);
    }

  Serial.println(F("Homed"));
    return true;
  }

  // Approach limit
  unsigned long startTime = millis();
  float driveVoltage = max(FRICTION_RIGHT + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
  setMotor(driveVoltage);

  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }

  if (leftPressed()) {
    Serial.println(F("Contact"));

    // Hold at limit
    long currentPos = encoder.read();
    long lastPos = currentPos;
    unsigned long holdStart = millis();
    int stableTicks = 0;
    float holdVoltage = max(FRICTION_RIGHT, CALIBRATE_MIN_VOLTAGE - 0.5);

    while (millis() - holdStart < CALIBRATE_HOLD_TIME || stableTicks < CALIBRATE_STABLE_TICKS) {
      setMotor(holdVoltage);
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
    delay(100);

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
      Serial.print(F("⚠️  Warning: Encoder not zeroed, reading: "));
      Serial.println(finalPos);
      encoder.write(0);  // Try one more time
      delay(50);
    }

  Serial.println(F("Homed"));
    return true;
  } else {
    stopMotor();
    Serial.println(F("Timeout"));
    return false;
  }
}


// EEPROM
void loadCalibrationFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) {
    Serial.println(F("EEPROM empty"));
    return;
  }

  EEPROM.get(EEPROM_KP, KP);
  EEPROM.get(EEPROM_KI, KI);
  EEPROM.get(EEPROM_KD, KD);
  EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);

  adaptiveFrictionLeft = FRICTION_LEFT;
  adaptiveFrictionRight = FRICTION_RIGHT;

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

  Serial.println(F("EEPROM OK"));
}

void saveTargetsToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * sizeof(long), targetPositions[i]);
  }
  Serial.println(F("OK"));
}

void savePIDToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  Serial.println(F("OK"));
}

void saveFrictionToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  Serial.println(F("OK"));
}

// MANUAL CALIBRATION
void manualCalibration() {
  Serial.println(F("\nMANUAL CAL"));
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  Serial.println(F("R=Right L=Left S=Stop 1-4=Save Q=Quit\n"));
  
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();
      
      cmd = toupper(cmd);
      
      long pos = encoder.read();
      
      switch (cmd) {
        case 'R':
          setMotor(-3.5);
          Serial.println(F("Right"));
          break;
        case 'L':
          setMotor(3.5);
          Serial.println(F("Left"));
          break;
        case 'S':
          stopMotor();
          Serial.print(F("Stop: "));
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
          Serial.println(F("\nCalibration done\n"));
          Serial.print(F("T1="));
          Serial.print(TARGET_1_POSITION);
          Serial.print(F(" T2="));
          Serial.print(TARGET_2_POSITION);
          Serial.print(F(" T3="));
          Serial.print(TARGET_3_POSITION);
          Serial.print(F(" T4="));
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
        Serial.println(F("\nAUTO MODE"));
        Serial.println(F("Bounds: 0 to -1424\n"));

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
          for (int i = 0; i < 4; i++) {
            ProxSensors[i].hitDetected = false;
            ProxSensors[i].hitTime = 0;
          }
          
          targetHitTime = 0;

          Serial.println(F("Homed"));
        } else {
          Serial.println(F("Homing failed\n"));
        }
      }
      break;
    
    case 'S':
      Serial.println(F("\nSTOP\n"));
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
        Serial.println(F("\nStop auto mode first\n"));
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
      Serial.println(F("OK"));
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
  
  Serial.print(F("\nL"));
  Serial.print(lane);
  Serial.print(F(": "));
  Serial.print(currentPos);
  Serial.print(F("->"));
  Serial.print(desiredPosition);
  Serial.print(F(" err="));
  Serial.println(error);
}

// DISPLAY
void printWelcome() {
  Serial.println(F("\nPLANTS VS ZOMBIES"));
}

void printHelp() {
  Serial.println(F("\nCmds: C Z G S 1-4 P D M R L W H"));
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
  Serial.println(F("\nSTATUS"));
  Serial.print(F("Mode: "));
  Serial.println(autoMode ? F("AUTO") : F("MAN"));
  Serial.print(F("State: "));
  switch(currentState) {
    case CALIBRATE: Serial.println(F("CAL")); break;
    case FIND_RANGE: Serial.println(F("RNG")); break;
    case CHOOSE_ACTIVE_TARGET: Serial.println(F("CHS")); break;
    case MOVE_TO_TARGET: Serial.println(F("MOV")); break;
  }
  Serial.print(F("Pos: "));
  Serial.print(currentPos);
  Serial.print(F(" Tgt: "));
  Serial.print(desiredPosition);
  Serial.print(F(" Err: "));
  Serial.println((int)error);
  Serial.print(F("Vel: "));
  Serial.print((int)motorVelocity);
  Serial.println(F(" cnt/s"));
  Serial.print(F("Range: "));
  Serial.print(LOWER_BOUND);
  Serial.print(F(" to "));
  Serial.println(UPPER_BOUND);
  Serial.print(F("Target: "));
  Serial.println(activeTargetIndex >= 0 ? activeTargetIndex + 1 : 0);
  Serial.print(F("PID: Kp="));
  Serial.print(KP, 4);
  Serial.print(F(" Ki="));
  Serial.print(KI, 4);
  Serial.print(F(" Kd="));
  Serial.println(KD, 4);
  Serial.print(F("Friction: L="));
  Serial.print(FRICTION_LEFT, 2);
  Serial.print(F(" R="));
  Serial.println(FRICTION_RIGHT, 2);
  Serial.println();
}

void printAllSensors() {
  Serial.println(F("\nSENSORS"));
  Serial.print(F("Enc: "));
  Serial.println(encoder.read());
  Serial.print(F("Limits: L="));
  Serial.print(leftPressed() ? 1 : 0);
  Serial.print(F(" R="));
  Serial.println(rightPressed() ? 1 : 0);
  Serial.print(F("Raw: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(analogRead(ProxSensors[i].pin));
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.print(F("Filt: "));
  for (int i = 0; i < 4; i++) {
    Serial.print((int)ProxSensors[i].currVal);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.print(F("Dist: "));
  for (int i = 0; i < 4; i++) {
    Serial.print((int)((1.0 - zombieDistances[i]) * 100));
    Serial.print(F("% "));
  }
  Serial.println();
}

void continuousMonitor() {
  Serial.println(F("\nMONITOR - key to stop\n"));
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  while (!Serial.available()) {
    updateAllSensors();
    Serial.print(F("E:"));
    Serial.print(encoder.read());
    Serial.print(F(" L:"));
    Serial.print(leftPressed() ? 1 : 0);
    Serial.print(F(" R:"));
    Serial.print(rightPressed() ? 1 : 0);
    Serial.print(F(" P:"));
    for (int i = 0; i < 4; i++) {
      Serial.print((int)ProxSensors[i].currVal);
      Serial.print(F(" "));
    }
    Serial.print(F(" D:"));
    for (int i = 0; i < 4; i++) {
      Serial.print(ProxSensors[i].direction == FORWARD ? F(">") : 
                   ProxSensors[i].direction == BACKWARD ? F("<") : F("-"));
    }
    Serial.println();
    delay(100);
  }
  while (Serial.available()) Serial.read();
  Serial.println(F("\nStopped\n"));
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
