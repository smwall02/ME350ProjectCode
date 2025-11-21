// ============================================================================
// ME 350 - Plants vs Zombies - COMPETITION CODE - IMPROVED
// Enhanced target switching and zombie tracking
// Updated with improved limit switch handling and PID control
// ============================================================================

#include <Encoder.h>
#include <EEPROM.h>

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
#define ON_OFF_SWITCH_PIN 5  // Flip switch for motor enable/disable

#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

Encoder encoder(ENCODER_A, ENCODER_B);

// ============================================
// EEPROM LAYOUT (Compatible with PID auto-tune sketch)
// ============================================
const int EEPROM_FLAG = 0;
const int EEPROM_KP = 1;
const int EEPROM_KI = 5;
const int EEPROM_KD = 9;
const int EEPROM_FRICTION_LEFT = 13;
const int EEPROM_FRICTION_RIGHT = 17;
const int EEPROM_LEFT_LIMIT = 21;
const int EEPROM_RIGHT_LIMIT = 25;
const int EEPROM_LANES_BASE = 29;  // 4 lanes * 4 bytes each

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
long LOWER_BOUND = 0;      // Fixed: Left limit position (home)
long UPPER_BOUND = -1424;  // Fixed: Right limit position (from calibration)

long targetPositions[4] = {
  TARGET_1_POSITION,
  TARGET_2_POSITION,
  TARGET_3_POSITION,
  TARGET_4_POSITION
};

// ============================================
// SENSOR CALIBRATION (Permanent values from calibration)
// ============================================
// Permanent sensor ranges based on calibration:
// Lane 1: MIN=86, MAX=585, Range=499
// Lane 2: MIN=115, MAX=594, Range=479
// Lane 3: MIN=132, MAX=628, Range=496
// Lane 4: MIN=84, MAX=601, Range=517
int ProxRange[4][2] = {
  {585, 86},   // Lane 1: [MAX, MIN]
  {594, 115},  // Lane 2: [MAX, MIN]
  {628, 132},  // Lane 3: [MAX, MIN]
  {601, 84}    // Lane 4: [MAX, MIN]
};

bool sensorCalibrated = true;  // Use permanent values, no calibration needed
bool dynamicCalibrationActive = false;
bool rangeFindingComplete = true;  // Using fixed bounds
unsigned long calibrationStartTime = 0;
const unsigned long DYNAMIC_CALIBRATION_TIME = 10000;

int dynamicMin[4];
int dynamicMax[4];

// ============================================
// PROXIMITY SENSOR PROCESSING
// ============================================
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

// Sensor filtering parameters
const float alpha = 0.925;
const int stopTimeout = 150;  // Faster stopped detection
const int lowerNoiseLimit = 5;
const int upperNoiseLimit = 8;
const int noiseThreshold = 225;
int noiseLimit = 8;

// Target selection
int activeTargetIndex = -1;
int previousTargetIndex = -1;  // Track previous target
long activeTargetPosition = WAIT_POSITION;
float closestZombieDist = 2.0;
float zombieDistances[4];
bool WAIT_POS = true;

// Fine positioning adjustment
long previousMoveStartPosition = 0;  // Position before last move
bool fineAdjustmentActive = false;   // Flag for fine adjustment mode
long fineAdjustmentTarget = 0;       // Fine-tuned target position
const int FINE_ADJUSTMENT_AMOUNT = 2;  // 1-2 counts adjustment
float previousZombieDistance = 1.0;  // Track previous distance to detect if getting closer

// ============================================
// FRICTION COMPENSATION (Improved)
// ============================================
// NOTE: These values will be loaded from EEPROM if available
// FRICTION_LEFT: voltage needed when moving TO MORE NEGATIVE positions (away from home)
// FRICTION_RIGHT: voltage needed when moving TO LESS NEGATIVE positions (toward home)
float FRICTION_LEFT = 0.25;  // Reduced from 2.2 to prevent overshoot to right
float FRICTION_RIGHT = 0.25; // For moving toward less negative (left/toward home)

// Adaptive friction boost (increases if target not reached)
float adaptiveFrictionLeft = FRICTION_LEFT;
float adaptiveFrictionRight = FRICTION_RIGHT;
const float FRICTION_BOOST_AMOUNT = 0.2;

// Legacy adaptive variables (kept for compatibility)
float adaptiveFrictionVoltage = 0;
long lastAdaptivePosition = 0;
unsigned long adaptiveStartTime = 0;
bool adaptiveLearning = false;
bool adaptiveLearned = false;

// ============================================
// PID CONTROL PARAMETERS
// ============================================
float KP = 0.015;  // Reduced from 0.020 to be less aggressive
float KI = 0.003;  // Reduced from 0.005 to prevent integral windup
float KD = 0.020;  // Increased from 0.004 for much stronger damping to prevent overshoot

float KP_active = KP;
float KI_active = KI;
float KD_active = KD;

const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 0.8;
const int TARGET_BAND = 2;  // Position tolerance: +/- 2 counts
const float MAX_INTEGRAL = 1200.0;
const unsigned long CONTROL_PERIOD = 10;

// Rightward drift compensation (accounts for momentum when moving right)
// When moving right (toward more negative positions), the system overshoots by 1-3 counts
// due to momentum. This offset adjusts the effective target slightly left during control,
// while arrival detection still uses the original target position.
const int RIGHTWARD_DRIFT_OFFSET = 2;  // Compensate for 2-count overshoot to the right

// ============================================
// MOTION CONTROL STATE
// ============================================
long desiredPosition = 0;
float errorIntegral = 0;
float lastError = 0;
float motorVelocity = 0;
int previousMotorPosition = 0;
long previousVelCompTime = 0;

// Stuck detection for final positioning
long lastStuckCheckPos = 0;
unsigned long lastStuckCheckTime = 0;
int stuckCounter = 0;

// Retry logic for positioning accuracy
int positionRetryCount = 0;
const int MAX_POSITION_RETRIES = 1;
const int RETRY_ERROR_THRESHOLD = 3;  // Retry if stuck at error > 3

const int MIN_VEL_COMP_COUNT = 2;
const long MIN_VEL_COMP_TIME = 10000;

unsigned long lastControlTime = 0;
unsigned long lastSensorTime = 0;
unsigned long arrivalTime = 0;
const int targetActivateTime = 200;  // Time at position before choosing next target

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

// Improved homing softness parameters
const float CALIBRATE_EXTRA_VOLTAGE = 0.6;      // Added to overcome friction during homing
const float CALIBRATE_MIN_VOLTAGE = 3.5;        // Minimum drive voltage during homing
const unsigned long CALIBRATE_HOLD_TIME = 300;  // ms to hold on limit before zeroing
const int CALIBRATE_STABLE_TICKS = 3;           // Stable readings required before zeroing
const float VEL_STOP_THRESH = 2.0;              // counts/sec considered stopped

// NEW: Target success detection
unsigned long targetHitTime = 0;
const unsigned long MIN_HIT_TIME = 100;  // Minimum time to confirm hit

// PID tuning removed - done in separate sketch

// ============================================
// SETUP
// ============================================
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

  // Load PID, friction, and lane calibration from EEPROM (if available)
  loadCalibrationFromEEPROM();

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

// ============================================
// SENSOR UPDATE
// ============================================
void updateAllSensors() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 4; i++) {
    // Store previous state
    ProxSensors[i].prevDirection = ProxSensors[i].direction;
    ProxSensors[i].prevVelocity = ProxSensors[i].velocity;

    // Read and filter sensor value
    float rawReading = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal +
                             (1.0 - alpha) * rawReading;

    // Calculate velocity (rate of change of sensor value)
    // Negative velocity = approaching (sensor value decreasing)
    // Positive velocity = retreating (sensor value increasing)
    unsigned long timeDelta = currentTime - ProxSensors[i].lastVelocityUpdate;
    if (timeDelta > 0) {
      float valueDelta = ProxSensors[i].currVal - ProxSensors[i].prevValForVelocity;
      ProxSensors[i].velocity = (valueDelta * 1000.0) / timeDelta;  // Change per second
      ProxSensors[i].lastVelocityUpdate = currentTime;
      ProxSensors[i].prevValForVelocity = ProxSensors[i].currVal;  // Update for next iteration
    } else {
      ProxSensors[i].velocity = 0.0;
    }

    if (ProxSensors[i].currVal >= noiseThreshold) {
      noiseLimit = upperNoiseLimit;
    } else {
      noiseLimit = lowerNoiseLimit;
    }

    // Direction detection based on filtered value change
    if (abs(ProxSensors[i].currVal - ProxSensors[i].prevVal) < noiseLimit) {
      if (timeDelta >= stopTimeout) {
        ProxSensors[i].direction = STOPPED;
      }
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].backwardCount = 0;

    } else if (ProxSensors[i].currVal - ProxSensors[i].prevVal < 0) {
      // Sensor value decreasing = zombie approaching
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;

      if (ProxSensors[i].forwardCount > 3) {
        ProxSensors[i].direction = FORWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = currentTime;
        // Reset hit detection when zombie starts moving forward again
        if (ProxSensors[i].hitDetected) {
          ProxSensors[i].hitDetected = false;
          ProxSensors[i].hitTime = 0;
        }
      }

    } else {
      // Sensor value increasing = zombie retreating
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;

      if (ProxSensors[i].backwardCount > 3) {
        ProxSensors[i].direction = BACKWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = currentTime;
      }
    }

    // IMPROVED HIT DETECTION: Detect velocity reversal
    // When velocity switches from negative (approaching) to positive (retreating),
    // that indicates the zombie hit the target and bounced back
    const float VELOCITY_THRESHOLD = 5.0;  // Minimum velocity change to detect hit
    const float HIT_VELOCITY_THRESHOLD = 10.0;  // Minimum retreat velocity to confirm hit
    
    if (!ProxSensors[i].hitDetected) {
      // Check for velocity reversal: was approaching (negative vel) and now retreating (positive vel)
      bool wasApproaching = (ProxSensors[i].prevVelocity < -VELOCITY_THRESHOLD);
      bool nowRetreating = (ProxSensors[i].velocity > HIT_VELOCITY_THRESHOLD);
      
      // Also check direction change as backup
      bool directionReversed = (ProxSensors[i].prevDirection == FORWARD && 
                                 ProxSensors[i].direction == BACKWARD);
      
      if ((wasApproaching && nowRetreating) || directionReversed) {
        ProxSensors[i].hitDetected = true;
        ProxSensors[i].hitTime = currentTime;
      }
    } else {
      // If already detected hit, check if zombie has moved far enough away to reset
      if (ProxSensors[i].direction == FORWARD && 
          ProxSensors[i].velocity < -VELOCITY_THRESHOLD) {
        // Zombie is approaching again, reset hit detection
        ProxSensors[i].hitDetected = false;
        ProxSensors[i].hitTime = 0;
      }
    }

    // Update zombie distances continuously (for display and decision making)
    if (sensorCalibrated) {
      zombieDistances[i] = (ProxSensors[i].currVal - ProxRange[i][1]) /
                           (float)(ProxRange[i][0] - ProxRange[i][1]);
      zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);
    } else {
      zombieDistances[i] = 1.0;  // Default to far away if not calibrated
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

// Range finding removed - using fixed bounds (0 to -1424)

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

      if (!dynamicCalibrationActive && sensorCalibrated) {
        Serial.println(F("State: CALIBRATE → CHOOSE_ACTIVE_TARGET (tracking enabled)\n"));
        currentState = CHOOSE_ACTIVE_TARGET;
        systemEnabled = true;
      }
      else if (!dynamicCalibrationActive && !sensorCalibrated) {
        // Skip range finding - use fixed bounds, go directly to sensor calibration
        Serial.println(F("State: CALIBRATE → Sensor Calibration (using fixed bounds)\n"));
        rangeFindingComplete = true;  // Mark as complete since we're using fixed values
        startDynamicCalibration();
        desiredPosition = LOWER_BOUND;
        systemEnabled = true;
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      activeTargetIndex = -1;
      closestZombieDist = 2.0;

      // Track previous move start position for fine adjustment logic
      previousMoveStartPosition = encoder.read();

      // Zombie distances are now updated continuously in updateAllSensors()
      for (int i = 0; i < 4; i++) {
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
        fineAdjustmentActive = false;  // Reset fine adjustment
        previousZombieDistance = zombieDistances[activeTargetIndex];  // Initialize distance tracking
        
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
        fineAdjustmentActive = false;
        Serial.println(F("No active FORWARD targets, moving to wait position"));
      }
      
      desiredPosition = activeTargetPosition;
      moveStartTime = millis();
      arrivalTime = millis();
      targetReached = false;
      stuckCounter = 0;
      positionRetryCount = 0;
      currentState = MOVE_TO_TARGET;
      break;
    
    case MOVE_TO_TARGET:
      // Use fine adjustment target if active, otherwise use original target
      if (fineAdjustmentActive) {
        desiredPosition = fineAdjustmentTarget;
      } else {
        desiredPosition = activeTargetPosition;
      }
      
      long currentPos = encoder.read();
      long error = desiredPosition - currentPos;
      
      // Safety check - approaching right limit
      if (currentPos < UPPER_BOUND - 50) {
        Serial.println(F("⚠️  Approaching right limit, returning to safe zone"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
      // IMPROVED: Velocity-based hit detection
      if (activeTargetIndex >= 0 && !WAIT_POS) {
        int targetDirection = ProxSensors[activeTargetIndex].direction;
        bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
        unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
        
        // PRIMARY: Use velocity-based hit detection (most reliable)
        // When velocity switches from negative (approaching) to positive (retreating),
        // that means the light hit the sensor and the target bounced back
        if (hitDetected && hitTime > 0) {
          // Confirm the hit for MIN_HIT_TIME to avoid false positives
          if (millis() - hitTime >= MIN_HIT_TIME) {
            Serial.println(F("✓ Target HIT! Velocity reversal detected (light hit sensor), choosing next"));
            // Reset hit detection for this sensor
            ProxSensors[activeTargetIndex].hitDetected = false;
            ProxSensors[activeTargetIndex].hitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        }
        
        // BACKUP: Check direction change (for cases where velocity detection might miss)
        int prevDirection = ProxSensors[activeTargetIndex].prevDirection;
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
          
          if (targetHitTime == 0) {
            targetHitTime = millis();
          }
          
          // Confirm the hit for MIN_HIT_TIME before switching
          if (millis() - targetHitTime >= MIN_HIT_TIME) {
            Serial.println(F("✓ Target HIT! Direction reversed (backup detection), choosing next"));
            targetHitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        } else {
          targetHitTime = 0;  // Reset if not consistently backward
        }
        
        // IMPROVED: Switch if zombie retreated far away (no longer a threat)
        // distance > 0.80 means percentage < 20% (far from photo sensor)
        if (targetDirection == BACKWARD &&
            zombieDistances[activeTargetIndex] > 0.80) {
          Serial.println(F("✓ Target retreated far away (safe), choosing next"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }

        // Check for more dangerous forward-moving zombie
        for (int i = 0; i < 4; i++) {
          if (i != activeTargetIndex &&
              ProxSensors[i].direction == FORWARD &&
              zombieDistances[i] < zombieDistances[activeTargetIndex] - 0.20) {
            Serial.print(F("✓ Closer threat in Lane "));
            Serial.println(i + 1);
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        }
      }
      
      // Check if there are ANY forward-moving zombies to track
      bool hasForwardZombie = false;
      for (int i = 0; i < 4; i++) {
        if (ProxSensors[i].direction == FORWARD) {
          hasForwardZombie = true;
          break;
        }
      }

      // If no forward zombies and been moving for >1s, go to wait position
      if (!hasForwardZombie && millis() - moveStartTime > 1000) {
        Serial.println(F("No forward threats detected, reconsidering"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }

      // Check arrival against current desired position (may be fine-adjusted)
      long errorToCurrentTarget = desiredPosition - currentPos;
      // Also check against original target for fine adjustment logic
      long errorToOriginalTarget = activeTargetPosition - currentPos;
      
      if (abs(errorToCurrentTarget) <= TARGET_BAND) {
        if (WAIT_POS) {
          // At wait position - only reconsider if there's a new forward target
          if (hasForwardZombie) {
            Serial.println(F("✓ New threat detected, choosing target"));
            currentState = CHOOSE_ACTIVE_TARGET;
          }
          // Otherwise stay put at wait position
        } else if (millis() - arrivalTime > targetActivateTime) {
          // Check if targeted zombie was hit (velocity reversal) or is retreating
          if (activeTargetIndex >= 0) {
            bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
            unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
            
            // Primary: Check for velocity-based hit detection
            if (hitDetected && hitTime > 0 && millis() - hitTime >= MIN_HIT_TIME) {
              Serial.println(F("✓ Target HIT at position (velocity reversal), choosing next"));
              ProxSensors[activeTargetIndex].hitDetected = false;
              ProxSensors[activeTargetIndex].hitTime = 0;
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            // Backup: Check if zombie is moving backward
            else if (ProxSensors[activeTargetIndex].direction == BACKWARD) {
              Serial.println(F("✓ Target retreating, choosing next"));
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            // FINE POSITIONING: If zombie still approaching and at original target, make small adjustment
            else if (ProxSensors[activeTargetIndex].direction == FORWARD && 
                     !fineAdjustmentActive &&
                     abs(errorToOriginalTarget) <= TARGET_BAND &&  // At original target position
                     zombieDistances[activeTargetIndex] < 0.20 &&  // Close and still approaching
                     zombieDistances[activeTargetIndex] < previousZombieDistance) {  // Getting closer
              
              // Determine adjustment direction based on previous move
              // If moved RIGHT (previous position > target), likely overshot right, adjust LEFT
              // If moved LEFT (previous position < target), likely undershot, adjust RIGHT
              bool movedRight = (previousMoveStartPosition > activeTargetPosition);
              
              if (movedRight) {
                // Moved right, likely overshot, adjust left (toward less negative)
                fineAdjustmentTarget = activeTargetPosition + FINE_ADJUSTMENT_AMOUNT;
                Serial.print(F("🔧 Fine adjust LEFT (+"));
                Serial.print(FINE_ADJUSTMENT_AMOUNT);
                Serial.print(F(") - zombie at "));
                Serial.print((int)(zombieDistances[activeTargetIndex] * 100));
                Serial.println(F("% getting closer"));
              } else {
                // Moved left, likely undershot, adjust right (toward more negative)
                fineAdjustmentTarget = activeTargetPosition - FINE_ADJUSTMENT_AMOUNT;
                Serial.print(F("🔧 Fine adjust RIGHT (-"));
                Serial.print(FINE_ADJUSTMENT_AMOUNT);
                Serial.print(F(") - zombie at "));
                Serial.print((int)(zombieDistances[activeTargetIndex] * 100));
                Serial.println(F("% getting closer"));
              }
              
              fineAdjustmentActive = true;
              desiredPosition = fineAdjustmentTarget;
              arrivalTime = millis();  // Reset arrival time for fine adjustment
            }
            
            // Update previous zombie distance for next iteration
            if (activeTargetIndex >= 0) {
              previousZombieDistance = zombieDistances[activeTargetIndex];
            }
            // Otherwise stay at target position until zombie starts retreating
          }
        }
      } else {
        arrivalTime = millis();
        // Reset fine adjustment if we've moved away from target significantly
        if (fineAdjustmentActive && abs(errorToCurrentTarget) > TARGET_BAND * 2) {
          fineAdjustmentActive = false;
        }
      }
      break;
  }
}

// ============================================
// MOTION CONTROL
// ============================================
void runMotionControl() {
  long currentPosition = encoder.read();
  
  // Apply rightward drift compensation: if moving right (toward more negative),
  // adjust target slightly left to compensate for momentum overshoot
  long adjustedDesiredPosition = desiredPosition;
  if (currentPosition > desiredPosition) {
    // Moving right (current is less negative than target)
    // Adjust target left by drift offset to compensate for overshoot
    adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
  }
  
  float error = adjustedDesiredPosition - currentPosition;
  
  if (currentState == MOVE_TO_TARGET && autoMode) {
    // Check for left-side drift (position should never be > 50)
    if (currentPosition > 50) {
      Serial.println(F("⚠️  Position drift detected - need recalibration"));
      stopMotor();
      return;
    }
  }
  
  if (dynamicCalibrationActive) {
    desiredPosition = LOWER_BOUND;
    adjustedDesiredPosition = LOWER_BOUND;
    error = desiredPosition - currentPosition;
    
    if (abs(error) > 10) {
      float correctionVoltage = constrain(error * 0.05, -2.0, 2.0);
      setMotor(correctionVoltage);
    } else {
      stopMotor();
    }
    return;
  }
  
  // Check against original target position for arrival (not adjusted one)
  float originalError = desiredPosition - currentPosition;
  if (abs(originalError) <= TARGET_BAND) {
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

  // Retry logic: if stuck outside target band for too long, retry once
  // Use original error for retry detection (not adjusted)
  if (abs(originalError) > RETRY_ERROR_THRESHOLD && abs(originalError) < 50) {
    // Check if we've been stuck at this error for 500ms
    if (millis() - moveStartTime > 500 && positionRetryCount < MAX_POSITION_RETRIES) {
      positionRetryCount++;
      Serial.print(F("⚠️  Stuck at error = "));
      Serial.print(abs(originalError));
      Serial.println(F(", retrying..."));

      // Reset for retry
      errorIntegral = 0;
      lastError = 0;
      moveStartTime = millis();
      stuckCounter = 0;
      delay(100);
      return;
    }
  }
  
  // IMPROVED: Adaptive friction learning - learns direction-specific friction
  // Start learning when error is significant but not too large (better for accuracy)
  if (abs(error) > 100 && abs(error) < 800 && !adaptiveLearning && !adaptiveLearned) {
    adaptiveLearning = true;
    adaptiveFrictionVoltage = 1.5;  // Start lower for faster learning
    lastAdaptivePosition = currentPosition;
    adaptiveStartTime = millis();
    
    // Determine direction for learning
    bool movingRight = (error < 0);
    Serial.print(F("🔍 Learning friction ("));
    Serial.print(movingRight ? F("RIGHT") : F("LEFT"));
    Serial.println(F(")..."));
  }
  
  if (adaptiveLearning) {
    // Check if movement occurred (more sensitive detection)
    long positionChange = abs(currentPosition - lastAdaptivePosition);
    unsigned long elapsed = millis() - adaptiveStartTime;
    
    // Movement detected - friction learned!
    if (positionChange >= 5) {  // More sensitive: 5 counts instead of 10
      bool movingRight = (error < 0);
      
      // Store learned friction in appropriate direction variable
      if (movingRight) {
        adaptiveFrictionLeft = adaptiveFrictionVoltage;  // Moving right = need left friction
        FRICTION_LEFT = adaptiveFrictionVoltage;
      } else {
        adaptiveFrictionRight = adaptiveFrictionVoltage;  // Moving left = need right friction
        FRICTION_RIGHT = adaptiveFrictionVoltage;
      }
      
      Serial.print(F("  ✓ Learned friction: "));
      Serial.print(adaptiveFrictionVoltage, 2);
      Serial.print(F("V ("));
      Serial.print(movingRight ? F("LEFT") : F("RIGHT"));
      Serial.println(F(")"));
      
      adaptiveLearning = false;
      adaptiveLearned = true;
      
      // Apply learned friction immediately
      float voltage = (error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage;
      setMotor(voltage);
      return;
    }
    
    // No movement yet - increase voltage and try again
    // Use faster increments: 0.3V every 150ms (was 0.5V every 200ms)
    if (elapsed >= 150) {
      adaptiveFrictionVoltage += 0.3;
      adaptiveStartTime = millis();
      lastAdaptivePosition = currentPosition;  // Reset position check
      
      // Safety limit - if we exceed reasonable friction, use conservative value
      if (adaptiveFrictionVoltage > 4.5) {
        Serial.println(F("  ⚠️  Max friction reached, using 2.5V"));
        adaptiveFrictionVoltage = 2.5;
        adaptiveLearning = false;
        adaptiveLearned = true;
        
        // Store conservative value
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
    
    // Timeout protection - if no movement after 2 seconds, give up
    if (elapsed > 2000) {
      Serial.println(F("  ⚠️  Friction learning timeout"));
      adaptiveLearning = false;
      adaptiveLearned = true;
      adaptiveFrictionVoltage = 2.0;  // Use conservative default
      
      // Store default value
      bool movingRight = (error < 0);
      if (movingRight) {
        adaptiveFrictionLeft = 2.0;
        FRICTION_LEFT = 2.0;
      } else {
        adaptiveFrictionRight = 2.0;
        FRICTION_RIGHT = 2.0;
      }
    }
    
    // Apply test voltage
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

  // FRICTION COMPENSATION DISABLED - was causing overshoot
  float frictionComp = 0;
  // Disabled until proper tuning can be done
  // if (abs(error) > TARGET_BAND) {
  //   bool movingTowardMoreNegative = (error < 0);
  //   float baseFriction = movingTowardMoreNegative ? adaptiveFrictionLeft : adaptiveFrictionRight;
  //   float frictionScale = 1.0;
  //   float absError = abs(error);
  //   if (absError < 3) frictionScale = 0.05;
  //   else if (absError < 10) frictionScale = 0.15;
  //   else if (absError < 30) frictionScale = 0.4;
  //   else if (absError < 100) frictionScale = 0.7;
  //   if (abs(motorVelocity) > 5) frictionScale *= 0.5;
  //   if (error < 0) frictionComp = -baseFriction * frictionScale;
  //   else frictionComp = baseFriction * frictionScale;
  // }

  // Velocity feedforward - DISABLED to prevent overshoot
  float velocityFF = 0;
  // Disabled: was adding up to 3.2V extra, causing massive overshoot
  // if (abs(error) > 50) {
  //   float desiredVelocity = constrain(error / 0.15, -400, 400);
  //   velocityFF = 0.008 * desiredVelocity;
  // }

  // Calculate total voltage
  float totalVoltage = pidVoltage + frictionComp + velocityFF;

  // Voltage capping based on error magnitude (increased for faster movement)
  float voltageLimit = MAX_VOLTAGE;
  long absErr = abs(error);
  if (absErr > 800) {
    voltageLimit = 4.5;  // Increased from 3.0 for faster long moves
  } else if (absErr > 500) {
    voltageLimit = 4.0;  // Increased from 2.7
  } else if (absErr > 300) {
    voltageLimit = 3.5;  // Increased from 2.5
  } else if (absErr > 100) {
    voltageLimit = 3.0;  // Increased from 2.2
  } else if (absErr > 50) {
    voltageLimit = 2.5;  // Increased from 2.0
  } else {
    voltageLimit = 2.2;  // Increased from 1.8 for final approach
  }

  totalVoltage = constrain(totalVoltage, -voltageLimit, voltageLimit);

  // Anti-windup on zero crossing
  if ((error != 0) && (error * lastError < 0)) {
    errorIntegral *= 0.5;
  }

  // Stuck detection: if outside target band but not moving, apply minimum voltage
  // Use original error for stuck detection (not adjusted)
  unsigned long currentTime = millis();
  if (abs(originalError) > TARGET_BAND) {
    // Check if stuck (position hasn't changed in 300ms)
    if (currentTime - lastStuckCheckTime >= 300) {
      if (abs(currentPosition - lastStuckCheckPos) < 2) {
        stuckCounter++;

        // If stuck for 2+ consecutive checks, apply friction-overcoming voltage
        if (stuckCounter >= 2) {
          float minVoltage = 2.5;  // Conservative to prevent overshoot
          if (abs(totalVoltage) < minVoltage) {
            totalVoltage = (error < 0) ? -minVoltage : minVoltage;
          }
        }
      } else {
        stuckCounter = 0;  // Reset if moving
      }
      lastStuckCheckPos = currentPosition;
      lastStuckCheckTime = currentTime;
    }
  } else {
    stuckCounter = 0;  // Reset when within target band
  }

  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
  }

  lastError = error;
}

// ============================================
// MOTOR CONTROL (Improved with flip switch and better limit handling)
// ============================================
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

  // Constrain voltage to safe range
  voltage = constrain(voltage, -10.0, 10.0);

  // Convert voltage to PWM (0-255)
  int pwm = abs(voltage) * 25.5;

  // Safety: Check limit switches and prevent movement into limits
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwm = 0;
  }
  if (digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
    voltage = 0;
    pwm = 0;
  }

  // Apply voltage to H-bridge
  if (voltage > 0) {
    // Move LEFT (toward position 0)
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, pwm);
  } else if (voltage < 0) {
    // Move RIGHT (toward negative positions)
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
    analogWrite(MOTOR_ENA, pwm);
  } else {
    // STOP
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

  // Right limit is valid range boundary, not an e-stop
}

bool leftPressed() {
  return digitalRead(LIMIT_LEFT) == HIGH;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == HIGH;
}

// ============================================
// HOMING (Improved soft homing with stable detection)
// ============================================
bool homeToLeftLimit() {
  Serial.println(F("\nHOMING..."));

  if (leftPressed()) {
    Serial.println(F("At limit, stabilizing..."));

    // Hold gently at limit to ensure stable position
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

    Serial.print(F("Homed (encoder: "));
    Serial.print(encoder.read());
    Serial.println(F(")"));
    return true;
  }

  // Approach limit switch
  unsigned long startTime = millis();
  float driveVoltage = max(FRICTION_RIGHT + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
  setMotor(driveVoltage);

  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }

  if (leftPressed()) {
    Serial.println(F("Contact..."));

    // Hold gently at limit to remove bounce
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

    Serial.print(F("Homed (encoder: "));
    Serial.print(encoder.read());
    Serial.println(F(")"));
    return true;
  } else {
    stopMotor();
    Serial.println(F("Timeout"));
    return false;
  }
}

// PID tuning functions removed - done in separate sketch

// ============================================
// EEPROM LOADING AND SAVING
// ============================================
void loadCalibrationFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) {
    Serial.println(F("EEPROM not set, using defaults"));
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

  Serial.println(F("Loaded from EEPROM"));
}

void saveTargetsToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * sizeof(long), targetPositions[i]);
  }
  Serial.println(F("Lanes saved"));
}

void savePIDToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  Serial.println(F("PID saved"));
}

void saveFrictionToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  Serial.println(F("Friction saved"));
}

// ============================================
// MANUAL TARGET POSITION CALIBRATION
// ============================================
void manualCalibration() {
  Serial.println(F("\n=== MANUAL CALIBRATION ==="));
  
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

          // Save to EEPROM
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
        Serial.println(F("Sequence: Home → Track (using permanent sensor ranges)\n"));
        Serial.println(F("Using fixed encoder bounds: 0 to -1424\n"));

        // Full rehoming with state reset
        stopMotor();
        delay(200);
        
        if (homeToLeftLimit()) {
          // Ensure encoder is properly zeroed
          encoder.write(0);
          delay(50);
          if (encoder.read() != 0) {
            encoder.write(0);
            delay(50);
            encoder.write(0);
          }
          
          // Reset all state variables
          autoMode = true;
          systemEnabled = true;
          rangeFindingComplete = true;  // Using fixed bounds, no need to find range
          sensorCalibrated = true;  // Using permanent sensor ranges, no calibration needed
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
          
          // Reset target tracking
          activeTargetIndex = -1;
          previousTargetIndex = -1;
          WAIT_POS = true;
          previousZombieDistance = 1.0;
          
          // Reset all sensor hit detection
          for (int i = 0; i < 4; i++) {
            ProxSensors[i].hitDetected = false;
            ProxSensors[i].hitTime = 0;
          }
          
          targetHitTime = 0;

          Serial.println(F("✓ HOMING COMPLETE - Encoder zeroed"));
          Serial.print(F("Encoder position: "));
          Serial.println(encoder.read());
          Serial.println(F("Using permanent sensor ranges - starting target tracking...\n"));
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
      adaptiveLearning = false;
      adaptiveLearned = false;
      rangeFindingComplete = true;  // Using fixed bounds
      sensorCalibrated = true;  // Keep permanent sensor ranges
      adaptiveFrictionLeft = FRICTION_LEFT;
      adaptiveFrictionRight = FRICTION_RIGHT;
      Serial.println(F("Reset - friction learning will restart on next movement"));
      break;

    case 'L':
      loadCalibrationFromEEPROM();
      break;

    case 'W':
      saveTargetsToEEPROM();
      savePIDToEEPROM();
      saveFrictionToEEPROM();
      Serial.println(F("All saved"));
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
  Serial.println(F("\n=== PLANTS VS ZOMBIES - IMPROVED ==="));
  Serial.println(F("Soft homing | EEPROM | Flip switch"));
}

void printHelp() {
  Serial.println(F("\nCOMMANDS:"));
  Serial.println(F("C-Calibrate Z-Home G-Auto S-Stop"));
  Serial.println(F("1-4:Lanes P-Status D-Sensors M-Monitor"));
  Serial.println(F("R-Reset L-Load W-Save H-Help"));
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
  Serial.println(KP, 6);
  Serial.print(F("Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("Kd = "));
  Serial.println(KD, 6);
  
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
