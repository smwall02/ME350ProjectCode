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

// Safety margins to prevent hitting limit switches
const long SAFETY_MARGIN_RIGHT = 5;  // Stop 5 counts before right limit
const long SAFETY_MARGIN_LEFT = 5;    // Stop 5 counts before left limit

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
bool initialRangeFinding = false;  // Flag for initial startup range finding
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
float FRICTION_LEFT = 2.5;  // Increased for static friction overcoming
float FRICTION_RIGHT = 2.5; // For moving toward less negative (left/toward home)

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
float KP = 0.018;  // Slightly increased for faster response while maintaining stability
float KI = 0.003;  // Reduced from 0.005 to prevent integral windup
float KD = 0.022;  // Slightly increased damping to compensate for higher KP

float KP_active = KP;
float KI_active = KI;
float KD_active = KD;

const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 2.0;  // Increased to overcome static friction
const int TARGET_BAND = 2;  // Position tolerance: +/- 2 counts
const float BREAKAWAY_VOLTAGE_BOOST = 1.5;  // Extra voltage when starting from rest
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
const int MAX_POSITION_RETRIES = 2;  // Increased retries
const int RETRY_ERROR_THRESHOLD = 3;  // Retry if stuck at error > 3
const float RETRY_VOLTAGE_BOOST = 2.0;  // Extra voltage boost when retrying
unsigned long retryStartTime = 0;  // Track when retry started
bool inRetryMode = false;  // Flag for retry mode

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

const float CALIBRATION_VOLTAGE = 5.5;  // Increased for faster homing
const float HOMING_VOLTAGE = 5.5;  // Increased for faster homing

// Improved homing softness parameters
const float CALIBRATE_EXTRA_VOLTAGE = 0.8;      // Increased for faster homing
const float CALIBRATE_MIN_VOLTAGE = 4.5;        // Increased minimum drive voltage during homing
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
  
  // Periodic encoder bounds validation - ensure encoder never shows invalid positions
  if (currentTime - lastControlTime >= CONTROL_PERIOD) {
    long currentPos = encoder.read();
    
    // Validate encoder is within safe bounds (only if not in calibration/homing)
    if (currentState != CALIBRATE && currentState != FIND_RANGE && !dynamicCalibrationActive) {
      if (currentPos < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
        // Encoder shows position beyond right safety margin - correct it
        long safeRightLimit = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
        encoder.write(safeRightLimit);
        previousMotorPosition = safeRightLimit;
        previousVelCompTime = micros();
        motorVelocity = 0;
      }
      if (currentPos > SAFETY_MARGIN_LEFT && !leftPressed()) {
        // Encoder shows position beyond left safety margin - correct it
        encoder.write(SAFETY_MARGIN_LEFT);
        previousMotorPosition = SAFETY_MARGIN_LEFT;
        previousVelCompTime = micros();
        motorVelocity = 0;
      }
    }
  }
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
  
  // Safety: If at left limit, ALWAYS force encoder to 0 and reset tracking
  if (leftPressed()) {
    if (abs(currentPos) > 1) {
      // Encoder not at 0 - force reset
      encoder.write(0);
      delay(10);
      currentPos = encoder.read();
      if (abs(currentPos) > 1) {
        // Retry
        encoder.write(0);
        delay(10);
        currentPos = 0;
      } else {
        currentPos = 0;
      }
    } else {
      currentPos = 0;
    }
    previousMotorPosition = 0;
    previousVelCompTime = micros();
    motorVelocity = 0;
    return;
  }
  
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
  Serial.println(F("\nCalibrating..."));
  
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
  Serial.println(F("Cal done"));
  for (int i = 0; i < 4; i++) {
    ProxRange[i][0] = dynamicMax[i];
    ProxRange[i][1] = dynamicMin[i];
  }
  sensorCalibrated = true;
}

void printCalibrationProgress() {
  unsigned long remaining = DYNAMIC_CALIBRATION_TIME - (millis() - calibrationStartTime);
  Serial.print(F("Cal:"));
  Serial.print(remaining / 1000);
  Serial.println(F("s"));
}

// ============================================
// STATE MACHINE - IMPROVED
// ============================================
void runStateMachine() {
  switch (currentState) {
    
    case CALIBRATE:
      // Static variables for this state
      static unsigned long calibrateStartTime = 0;
      static unsigned long calibrateHoldStart = 0;
      static unsigned long lastDebugPrint = 0;
      
      // If at left limit switch, ensure encoder is at 0 and hold position
      if (leftPressed()) {
        // Reset homing timeout since we're at the limit
        calibrateStartTime = 0;
        
        long currentPos = encoder.read();
        if (abs(currentPos) > 1) {
          // Encoder not zeroed - reset it aggressively
          stopMotor();
          delay(200);
          for (int i = 0; i < 10; i++) {
            encoder.write(0);
            delay(100);
            currentPos = encoder.read();
            if (abs(currentPos) <= 1) {
              break;
            }
          }
          // Reset ALL tracking variables
          previousMotorPosition = 0;
          previousVelCompTime = micros();
          motorVelocity = 0;
          errorIntegral = 0;
          lastError = 0;
          lastStuckCheckPos = 0;
          lastStuckCheckTime = 0;
          stuckCounter = 0;
        }
        desiredPosition = LOWER_BOUND;
        // Hold at limit with gentle voltage
        float holdVoltage = max(FRICTION_RIGHT, 1.5);
        setMotor(holdVoltage);
        errorIntegral = 0;
        // Force position to be 0
        currentPos = 0;
        
        // Check if we're in initial range finding sequence
        // Debug: Print state of initialRangeFinding flag
        if (millis() - lastDebugPrint > 1000) {
          Serial.print(F("CAL: initialRangeFinding="));
          Serial.print(initialRangeFinding ? F("true") : F("false"));
          Serial.print(F(" leftPressed="));
          Serial.println(leftPressed() ? F("true") : F("false"));
          lastDebugPrint = millis();
        }
        
        if (initialRangeFinding) {
          // First time at left limit - after holding, go find right range
          if (calibrateHoldStart == 0) {
            calibrateHoldStart = millis();
            Serial.println(F("Holding..."));
          }
          if (millis() - calibrateHoldStart > 500) {  // Hold for 500ms
            calibrateHoldStart = 0;
            Serial.println(F("Finding range..."));
            currentState = FIND_RANGE;
            systemEnabled = true;
            // Reset encoder to 0 one more time before moving
            encoder.write(0);
            delay(100);
            previousMotorPosition = 0;
            previousVelCompTime = micros();
            motorVelocity = 0;
          }
        } else {
          // Not in initial range finding - ready to start operation immediately
          calibrateHoldStart = 0;
          Serial.println(F("Range found, starting operation..."));
          currentState = CHOOSE_ACTIVE_TARGET;
          systemEnabled = true;
          rangeFindingComplete = true;
          // Stop holding motor
          stopMotor();
          // Break immediately to ensure transition takes effect
          break;
        }
      } else {
        // Not at left limit - try to home
        // Add timeout check to prevent infinite homing attempts
        if (calibrateStartTime == 0) {
          calibrateStartTime = millis();
        }
        
        // If homing takes too long (30 seconds), force transition if range finding is complete
        if (millis() - calibrateStartTime > 30000) {
          if (!initialRangeFinding && rangeFindingComplete) {
            // Range finding is complete, but homing failed - try to proceed anyway
            Serial.println(F("Homing timeout, proceeding..."));
            calibrateStartTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            systemEnabled = true;
            break;
          }
          // Reset timeout if still in initial range finding
          calibrateStartTime = millis();
        }
        
        desiredPosition = LOWER_BOUND;
        systemEnabled = true;
      }
      break;
    
    case FIND_RANGE:
      // Move to right limit to find UPPER_BOUND - gentle approach
      static unsigned long rangeFindStartTime = 0;
      static bool rangeFindStarted = false;
      
      if (!rangeFindStarted) {
        rangeFindStartTime = millis();
        rangeFindStarted = true;
      }
      
      if (rightPressed()) {
        // At right limit - record position as UPPER_BOUND
        // Stop immediately and back off slightly
        stopMotor();
        delay(300);
        
        // Back off from limit by safety margin
        long rightLimitPos = encoder.read();
        if (rightLimitPos < -100) {  // Sanity check
          UPPER_BOUND = rightLimitPos + SAFETY_MARGIN_RIGHT;  // Add safety margin
        } else {
          UPPER_BOUND = rightLimitPos + SAFETY_MARGIN_RIGHT;  // Still add margin
        }
        
        // Move slightly left to get off the limit switch (back off more than safety margin)
        desiredPosition = rightLimitPos + (SAFETY_MARGIN_RIGHT * 2);  // Back off double the safety margin
        systemEnabled = true;
        
        // Wait to move off limit
        unsigned long backoffStart = millis();
        while (millis() - backoffStart < 1000 && encoder.read() < rightLimitPos + SAFETY_MARGIN_RIGHT) {
          delay(50);
        }
        
        rangeFindStarted = false;
        
        // Clear the initial range finding flag - we've found the range
        initialRangeFinding = false;
        
        // Now home back to left limit
        currentState = CALIBRATE;
        desiredPosition = LOWER_BOUND;
        systemEnabled = true;
      } else {
        // Move toward right limit - use gentle, gradual approach
        long currentPos = encoder.read();
        long estimatedLimit = -1500;  // Conservative estimate
        const long APPROACH_MARGIN = 50;  // Start slowing 50 counts before limit
        
        // Calculate how close we are
        long distanceToLimit = currentPos - estimatedLimit;
        
        if (distanceToLimit > APPROACH_MARGIN + 200) {
          // Still far away - move toward estimated limit slowly
          desiredPosition = estimatedLimit + APPROACH_MARGIN;
        } else if (distanceToLimit > APPROACH_MARGIN) {
          // Getting closer - slow down more
          desiredPosition = currentPos - 10;  // Move only 10 counts at a time
        } else {
          // Very close - move very slowly
          desiredPosition = currentPos - 5;  // Move only 5 counts at a time
        }
        
        // Safety: Never go beyond estimated limit
        if (desiredPosition < estimatedLimit) {
          desiredPosition = estimatedLimit;
        }
        
        systemEnabled = true;
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      // CRITICAL: If at left limit, verify encoder is at 0
      if (leftPressed()) {
        long checkPos = encoder.read();
        if (abs(checkPos) > 1) {
          // Force reset
          stopMotor();
          delay(200);
          for (int i = 0; i < 5; i++) {
            encoder.write(0);
            delay(100);
            checkPos = encoder.read();
            if (abs(checkPos) <= 1) {
              break;
            }
          }
          previousMotorPosition = 0;
          previousVelCompTime = micros();
          motorVelocity = 0;
        }
      }
      
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
        
        Serial.print(F("Tgt:L"));
        Serial.print(activeTargetIndex + 1);
        Serial.print(F(" D:"));
        Serial.print((int)((1.0 - zombieDistances[activeTargetIndex]) * 100));
        Serial.println();
        
        previousTargetIndex = activeTargetIndex;
        targetHitTime = 0;
        
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        fineAdjustmentActive = false;
      }
      
      // Constrain target position to safe bounds
      if (activeTargetPosition < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
        activeTargetPosition = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
      }
      if (activeTargetPosition > SAFETY_MARGIN_LEFT) {
        activeTargetPosition = SAFETY_MARGIN_LEFT;
      }
      
      desiredPosition = activeTargetPosition;
      moveStartTime = millis();
      arrivalTime = millis();
      targetReached = false;
      stuckCounter = 0;
      positionRetryCount = 0;
      inRetryMode = false;  // Reset retry mode for new target
      currentState = MOVE_TO_TARGET;
      break;
    
    case MOVE_TO_TARGET:
      // Use fine adjustment target if active, otherwise use original target
      if (fineAdjustmentActive) {
        desiredPosition = fineAdjustmentTarget;
      } else {
        desiredPosition = activeTargetPosition;
      }
      
      // Constrain desired position to safe bounds
      if (desiredPosition < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
        desiredPosition = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
      }
      if (desiredPosition > SAFETY_MARGIN_LEFT && !leftPressed()) {
        desiredPosition = SAFETY_MARGIN_LEFT;
      }
      
      long currentPos = encoder.read();
      
      // Validate encoder position is within bounds
      if (currentPos < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
        // Too far right - correct encoder
        long safeRightLimit = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
        encoder.write(safeRightLimit);
        delay(50);
        currentPos = safeRightLimit;
        previousMotorPosition = safeRightLimit;
        previousVelCompTime = micros();
        motorVelocity = 0;
      }
      if (currentPos > SAFETY_MARGIN_LEFT && !leftPressed()) {
        // Too far left (but not at limit) - correct encoder
        encoder.write(SAFETY_MARGIN_LEFT);
        delay(50);
        currentPos = SAFETY_MARGIN_LEFT;
        previousMotorPosition = SAFETY_MARGIN_LEFT;
        previousVelCompTime = micros();
        motorVelocity = 0;
      }
      
      long error = desiredPosition - currentPos;
      
      // Safety check: if beyond safe range, choose new target
      if (currentPos < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
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
          if (millis() - hitTime >= MIN_HIT_TIME) {
            Serial.println(F("HIT"));
            ProxSensors[activeTargetIndex].hitDetected = false;
            ProxSensors[activeTargetIndex].hitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        }
        
        int prevDirection = ProxSensors[activeTargetIndex].prevDirection;
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
          if (targetHitTime == 0) {
            targetHitTime = millis();
          }
          if (millis() - targetHitTime >= MIN_HIT_TIME) {
            Serial.println(F("HIT"));
            targetHitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        } else {
          targetHitTime = 0;
        }
        
        if (targetDirection == BACKWARD &&
            zombieDistances[activeTargetIndex] > 0.80) {
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }

        for (int i = 0; i < 4; i++) {
          if (i != activeTargetIndex &&
              ProxSensors[i].direction == FORWARD &&
              zombieDistances[i] < zombieDistances[activeTargetIndex] - 0.20) {
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

      if (!hasForwardZombie && millis() - moveStartTime > 1000) {
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }

      // Check arrival against current desired position (may be fine-adjusted)
      long errorToCurrentTarget = desiredPosition - currentPos;
      // Also check against original target for fine adjustment logic
      long errorToOriginalTarget = activeTargetPosition - currentPos;
      
      if (abs(errorToCurrentTarget) <= TARGET_BAND) {
        if (WAIT_POS) {
          if (hasForwardZombie) {
            currentState = CHOOSE_ACTIVE_TARGET;
          }
        } else if (millis() - arrivalTime > targetActivateTime) {
          // Check if targeted zombie was hit (velocity reversal) or is retreating
          if (activeTargetIndex >= 0) {
            bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
            unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
            
            if (hitDetected && hitTime > 0 && millis() - hitTime >= MIN_HIT_TIME) {
              Serial.println(F("HIT"));
              ProxSensors[activeTargetIndex].hitDetected = false;
              ProxSensors[activeTargetIndex].hitTime = 0;
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            else if (ProxSensors[activeTargetIndex].direction == BACKWARD) {
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
                fineAdjustmentTarget = activeTargetPosition + FINE_ADJUSTMENT_AMOUNT;
              } else {
                fineAdjustmentTarget = activeTargetPosition - FINE_ADJUSTMENT_AMOUNT;
              }
              
              // Constrain fine adjustment to safe bounds
              if (fineAdjustmentTarget < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
                fineAdjustmentTarget = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
              }
              if (fineAdjustmentTarget > SAFETY_MARGIN_LEFT && !leftPressed()) {
                fineAdjustmentTarget = SAFETY_MARGIN_LEFT;
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
  
  // CRITICAL: If at left limit, encoder MUST be at 0
  // Only reset if we're actually at the limit AND encoder is not already at 0
  if (leftPressed()) {
    // Check if encoder needs reset
    if (abs(currentPosition) > 1) {
      // Encoder not at 0 - reset it
      stopMotor();
      encoder.write(0);
      delay(50);
      // Verify and retry if needed
      currentPosition = encoder.read();
      if (abs(currentPosition) > 1) {
        for (int i = 0; i < 5; i++) {
          encoder.write(0);
          delay(100);
          currentPosition = encoder.read();
          if (abs(currentPosition) <= 1) {
            break;
          }
        }
      }
      // Force position to 0 if still not correct
      if (abs(currentPosition) > 1) {
        currentPosition = 0;
      }
      // Reset all tracking to match
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
      errorIntegral = 0;
      lastError = 0;
    } else {
      // Encoder is already at 0 - just ensure tracking variables match
      if (previousMotorPosition != 0) {
        previousMotorPosition = 0;
        previousVelCompTime = micros();
        motorVelocity = 0;
      }
      currentPosition = 0;
    }
  }
  
  // Apply rightward drift compensation: if moving right (toward more negative),
  // adjust target slightly left to compensate for momentum overshoot
  long adjustedDesiredPosition = desiredPosition;
  if (currentPosition > desiredPosition) {
    // Moving right (current is less negative than target)
    // Adjust target left by drift offset to compensate for overshoot
    adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
  }
  
  float error = adjustedDesiredPosition - currentPosition;
  
  // CRITICAL SAFETY: If at left limit, only prevent leftward movement
  if (leftPressed()) {
    // Calculate error to see which direction we're trying to move
    float originalError = desiredPosition - currentPosition;
    
    // If trying to move left (positive error), stop and reset encoder
    if (originalError > 0) {
      stopMotor();
      errorIntegral = 0;
      if (abs(currentPosition) > 2) {
        encoder.write(0);
        delay(50);
        previousMotorPosition = 0;
        previousVelCompTime = micros();
        motorVelocity = 0;
      }
      return;
    }
    
    // If trying to move right (negative error), allow it but ensure encoder is at 0
    if (abs(currentPosition) > 2) {
      encoder.write(0);
      delay(50);
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
      // Recalculate error after encoder reset
      currentPosition = encoder.read();
      adjustedDesiredPosition = desiredPosition;
      if (currentPosition > desiredPosition) {
        adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
      }
      error = adjustedDesiredPosition - currentPosition;
    }
  }
  
  // CRITICAL: Position validation - encoder should never be positive
  // If at left limit, position must be 0 or very close
  if (leftPressed()) {
    if (currentPosition > 5) {
      // Encoder drifted positive - force reset to 0
      encoder.write(0);
      delay(50);
      currentPosition = 0;
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
      errorIntegral = 0;
      // Recalculate error with corrected position
      adjustedDesiredPosition = desiredPosition;
      if (currentPosition > desiredPosition) {
        adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
      }
      error = adjustedDesiredPosition - currentPosition;
    }
  }
  
  // Safety: Never allow positive positions
  if (currentPosition > 10) {
    // Encoder has drifted significantly positive - reset to 0
    encoder.write(0);
    delay(50);
    currentPosition = 0;
    previousMotorPosition = 0;
    previousVelCompTime = micros();
    motorVelocity = 0;
    errorIntegral = 0;
    // Recalculate error with corrected position
    adjustedDesiredPosition = desiredPosition;
    if (currentPosition > desiredPosition) {
      adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
    }
    error = adjustedDesiredPosition - currentPosition;
  }
  
  // Safety: Never allow positions beyond upper bound (too far right)
  // Add safety margin to prevent hitting right limit switch
  if (currentPosition < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
    // Encoder has gone beyond safe range - cap it at safety margin
    long safeRightLimit = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
    encoder.write(safeRightLimit);
    delay(50);
    currentPosition = safeRightLimit;
    previousMotorPosition = safeRightLimit;
    previousVelCompTime = micros();
    motorVelocity = 0;
    errorIntegral = 0;
    // Recalculate error with corrected position
    adjustedDesiredPosition = desiredPosition;
    if (currentPosition > desiredPosition) {
      adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
    }
    error = adjustedDesiredPosition - currentPosition;
  }
  
  // Safety: Prevent movement beyond safe bounds
  if (currentPosition < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
    // Too close to right limit - stop and correct
    stopMotor();
    long safeRightLimit = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
    encoder.write(safeRightLimit);
    delay(50);
    currentPosition = safeRightLimit;
    previousMotorPosition = safeRightLimit;
    previousVelCompTime = micros();
    motorVelocity = 0;
    errorIntegral = 0;
    return;
  }
  
  if (currentPosition > SAFETY_MARGIN_LEFT && !leftPressed()) {
    // Too close to left limit (but not at it) - ensure we don't go further left
    if (desiredPosition > SAFETY_MARGIN_LEFT) {
      desiredPosition = SAFETY_MARGIN_LEFT;
    }
  }
  
  // Safety check: if encoder shows positive position (shouldn't happen), reset it
  if (currentState == MOVE_TO_TARGET && autoMode) {
    if (currentPosition > 10) {
      // Encoder drifted positive - reset to 0
      encoder.write(0);
      currentPosition = 0;
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
      errorIntegral = 0;
      // Recalculate error
      adjustedDesiredPosition = desiredPosition;
      if (currentPosition > desiredPosition) {
        adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
      }
      error = adjustedDesiredPosition - currentPosition;
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
  // Re-read position in case it was reset above, but only if not at left limit
  // (if at left limit, we already forced it to 0 above)
  if (!leftPressed()) {
    currentPosition = encoder.read();
  }
  
  // CRITICAL: Constrain desired position to safe bounds
  // Ensure desired position doesn't go beyond safe bounds
  if (desiredPosition < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
    desiredPosition = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
  }
  if (desiredPosition > SAFETY_MARGIN_LEFT && !leftPressed()) {
    desiredPosition = SAFETY_MARGIN_LEFT;
  }
  
  float originalError = desiredPosition - currentPosition;
  
  // CRITICAL: Prevent any movement when at limit switches (except during calibration/homing)
  if (leftPressed() && currentState != CALIBRATE && currentState != FIND_RANGE) {
    // At left limit - only allow rightward movement (negative error)
    if (originalError > 0) {
      // Trying to move left - stop immediately
      stopMotor();
      errorIntegral = 0;
      if (abs(currentPosition) > 2) {
        encoder.write(0);
        delay(50);
        previousMotorPosition = 0;
        previousVelCompTime = micros();
        motorVelocity = 0;
        currentPosition = encoder.read();
        originalError = desiredPosition - currentPosition;
      }
      return;
    }
  }
  
  if (rightPressed() && currentState != CALIBRATE && currentState != FIND_RANGE) {
    // At right limit - only allow leftward movement (positive error)
    if (originalError < 0) {
      // Trying to move right - stop immediately
      stopMotor();
      errorIntegral = 0;
      long safeRightLimit = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
      if (currentPosition < safeRightLimit) {
        encoder.write(safeRightLimit);
        delay(50);
        previousMotorPosition = safeRightLimit;
        previousVelCompTime = micros();
        motorVelocity = 0;
        currentPosition = encoder.read();
        originalError = desiredPosition - currentPosition;
      }
      return;
    }
  }
  
  // CRITICAL: Prevent leftward movement when at left limit (backup check)
  if (leftPressed() && originalError > 0) {
    // At left limit and trying to move left - stop immediately
    stopMotor();
    errorIntegral = 0;
    // Ensure encoder is at 0
    if (abs(currentPosition) > 2) {
      encoder.write(0);
      delay(50);
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
      currentPosition = encoder.read();
      originalError = desiredPosition - currentPosition;
    }
    return;
  }
  
  // CRITICAL: Prevent rightward movement beyond safe limit
  if (currentPosition <= UPPER_BOUND - SAFETY_MARGIN_RIGHT && originalError < 0) {
    // Too close to right limit and trying to move further right - stop immediately
    stopMotor();
    errorIntegral = 0;
    // Cap position at safety margin
    long safeRightLimit = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
    if (currentPosition < safeRightLimit) {
      encoder.write(safeRightLimit);
      delay(50);
      currentPosition = safeRightLimit;
      previousMotorPosition = safeRightLimit;
      previousVelCompTime = micros();
      motorVelocity = 0;
    }
    return;
  }
  
  if (abs(originalError) <= TARGET_BAND) {
    stopMotor();
    errorIntegral = 0;
    adaptiveLearning = false;

    if (!targetReached) {
      targetReached = true;
    }
    return;
  }

  targetReached = false;

  // Retry logic: if stuck outside target band for too long, retry with higher voltage
  // Use original error for retry detection (not adjusted)
  // Expanded threshold to handle larger errors (up to 100 counts)
  if (abs(originalError) > RETRY_ERROR_THRESHOLD && abs(originalError) < 100) {
    // Check if we've been stuck at this error for 500ms
    if (millis() - moveStartTime > 500 && positionRetryCount < MAX_POSITION_RETRIES) {
      if (!inRetryMode) {
        positionRetryCount++;
        Serial.print(F("Stuck:"));
        Serial.print(abs(originalError));
        Serial.print(F(" R:"));
        Serial.println(positionRetryCount);

        // Enter retry mode with voltage boost
        inRetryMode = true;
        retryStartTime = millis();
        
        // Reset for retry
        errorIntegral = 0;
        lastError = 0;
        moveStartTime = millis();
        stuckCounter = 0;
      }
    }
  }
  
  // IMPROVED: Adaptive friction learning - learns direction-specific friction
  // Start learning when error is significant but not too large (better for accuracy)
  if (abs(error) > 100 && abs(error) < 800 && !adaptiveLearning && !adaptiveLearned) {
    adaptiveLearning = true;
    adaptiveFrictionVoltage = 2.5;  // Start higher to overcome static friction
    lastAdaptivePosition = currentPosition;
    adaptiveStartTime = millis();
    
    bool movingRight = (error < 0);
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
      
      
      adaptiveLearning = false;
      adaptiveLearned = true;
      
      // Apply learned friction immediately
      float voltage = (error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage;
      setMotor(voltage);
      return;
    }
    
    // No movement yet - increase voltage and try again
    // Use faster increments: 0.5V every 100ms for quicker response
    if (elapsed >= 100) {
      adaptiveFrictionVoltage += 0.5;
      adaptiveStartTime = millis();
      lastAdaptivePosition = currentPosition;  // Reset position check
      
      if (adaptiveFrictionVoltage > 5.0) {
        adaptiveFrictionVoltage = 3.0;
        adaptiveLearning = false;
        adaptiveLearned = true;
        
        // Store learned value
        bool movingRight = (error < 0);
        if (movingRight) {
          adaptiveFrictionLeft = 3.0;
          FRICTION_LEFT = 3.0;
        } else {
          adaptiveFrictionRight = 3.0;
          FRICTION_RIGHT = 3.0;
        }
      }
    }
    
    if (elapsed > 1500) {
      adaptiveLearning = false;
      adaptiveLearned = true;
      adaptiveFrictionVoltage = 2.5;  // Use default value
      
      // Store default value
      bool movingRight = (error < 0);
      if (movingRight) {
        adaptiveFrictionLeft = 2.5;
        FRICTION_LEFT = 2.5;
      } else {
        adaptiveFrictionRight = 2.5;
        FRICTION_RIGHT = 2.5;
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

  // FRICTION COMPENSATION - re-enabled with improved scaling
  float frictionComp = 0;
  if (abs(error) > TARGET_BAND) {
    bool movingTowardMoreNegative = (error < 0);
    float baseFriction = movingTowardMoreNegative ? adaptiveFrictionLeft : adaptiveFrictionRight;
    float frictionScale = 1.0;
    float absError = abs(error);
    
    // Scale friction based on error magnitude - more aggressive for larger errors
    if (absError < 3) frictionScale = 0.1;
    else if (absError < 10) frictionScale = 0.3;
    else if (absError < 30) frictionScale = 0.6;
    else if (absError < 100) frictionScale = 0.8;
    else frictionScale = 1.0;  // Full friction for large errors
    
    // Reduce friction when already moving (dynamic friction is less than static)
    if (abs(motorVelocity) > 10) frictionScale *= 0.4;
    else if (abs(motorVelocity) > 5) frictionScale *= 0.6;
    
    // Apply friction compensation
    if (error < 0) frictionComp = -baseFriction * frictionScale;
    else frictionComp = baseFriction * frictionScale;
  }

  // Velocity feedforward - DISABLED to prevent overshoot
  float velocityFF = 0;
  // Disabled: was adding up to 3.2V extra, causing massive overshoot
  // if (abs(error) > 50) {
  //   float desiredVelocity = constrain(error / 0.15, -400, 400);
  //   velocityFF = 0.008 * desiredVelocity;
  // }

  // Calculate total voltage
  float totalVoltage = pidVoltage + frictionComp + velocityFF;
  
  // BREAKAWAY VOLTAGE BOOST: Add extra voltage when starting from rest to overcome static friction
  // If motor is not moving (or moving very slowly) and error is significant, add breakaway boost
  if (abs(motorVelocity) < 3.0 && abs(originalError) > 5) {
    float breakawayBoost = (originalError < 0) ? -BREAKAWAY_VOLTAGE_BOOST : BREAKAWAY_VOLTAGE_BOOST;
    totalVoltage += breakawayBoost;
  }
  
  // Apply voltage boost during retry mode (for first 300ms of retry)
  if (inRetryMode) {
    if (millis() - retryStartTime < 300) {
      // Apply aggressive voltage boost in direction of error
      float boostVoltage = (originalError < 0) ? -RETRY_VOLTAGE_BOOST : RETRY_VOLTAGE_BOOST;
      totalVoltage += boostVoltage;
    } else {
      // Retry boost period over, continue with normal control
      inRetryMode = false;
    }
  }

  // Voltage capping based on error magnitude (increased significantly for faster movement)
  // Also ensures minimum voltage to overcome static friction
  // Use lower voltage during range finding for gentler approach
  float voltageLimit = MAX_VOLTAGE;
  long absErr = abs(error);
  
  if (currentState == FIND_RANGE) {
    // Range finding mode - use lower, gentler voltages
    if (absErr > 500) {
      voltageLimit = 4.0;  // Gentle for long moves
    } else if (absErr > 200) {
      voltageLimit = 3.5;  // Medium speed
    } else if (absErr > 50) {
      voltageLimit = 3.0;  // Slower approach
    } else {
      voltageLimit = 2.5;  // Very gentle near limit
    }
  } else {
    // Normal operation - higher voltages
    if (absErr > 800) {
      voltageLimit = 6.5;  // Increased from 4.5 for much faster long moves
    } else if (absErr > 500) {
      voltageLimit = 6.0;  // Increased from 4.0
    } else if (absErr > 300) {
      voltageLimit = 5.5;  // Increased from 3.5
    } else if (absErr > 100) {
      voltageLimit = 5.0;  // Increased from 3.0
    } else if (absErr > 50) {
      voltageLimit = 4.5;  // Increased from 2.5
    } else if (absErr > 10) {
      voltageLimit = 4.0;  // Increased for medium errors
    } else {
      voltageLimit = 3.5;  // Minimum for small errors - enough to overcome static friction
    }
  }

  // Allow higher voltage limit during retry mode (20% boost)
  if (inRetryMode) {
    voltageLimit *= 1.2;
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

        // If stuck for 2+ consecutive checks, apply aggressive friction-overcoming voltage
        if (stuckCounter >= 2) {
          // Scale minimum voltage based on error magnitude - increased for static friction
          float minVoltage;
          long absError = abs(originalError);
          if (absError > 50) {
            minVoltage = 6.0;  // Large error - use high voltage
          } else if (absError > 20) {
            minVoltage = 5.0;  // Medium error - use medium-high voltage
          } else {
            minVoltage = 4.0;  // Small error - use higher voltage to overcome static friction
          }
          
          // Apply minimum voltage in direction of error
          if (abs(totalVoltage) < minVoltage) {
            totalVoltage = (originalError < 0) ? -minVoltage : minVoltage;
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

  // Apply motor voltage - ensure we apply at least friction compensation if error is significant
  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else if (abs(originalError) > TARGET_BAND) {
    // If error is significant but voltage is low, apply at least friction compensation
    // This helps overcome static friction for small movements
    float minFrictionVoltage = (originalError < 0) ? -adaptiveFrictionLeft : adaptiveFrictionRight;
    if (abs(minFrictionVoltage) > 0.5) {
      setMotor(minFrictionVoltage);
    } else {
      stopMotor();
    }
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
  // Also check encoder position to prevent getting too close to limits
  long currentPos = encoder.read();
  
  // Prevent leftward movement at or near left limit
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwm = 0;
  } else if (currentPos <= SAFETY_MARGIN_LEFT && voltage > 0 && !leftPressed()) {
    // Too close to left limit - prevent further leftward movement
    voltage = 0;
    pwm = 0;
  }
  
  // Prevent rightward movement at or near right limit
  if (digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
    voltage = 0;
    pwm = 0;
  } else if (currentPos <= UPPER_BOUND - SAFETY_MARGIN_RIGHT && voltage < 0) {
    // Too close to right limit - prevent further rightward movement
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
    
    // At left limit - aggressively reset encoder to 0
    long currentPos = encoder.read();
    if (abs(currentPos) > 1) {
      for (int i = 0; i < 5; i++) {
        encoder.write(0);
        delay(50);
        currentPos = encoder.read();
        if (abs(currentPos) <= 1) {
          break;
        }
      }
      // Reset all tracking variables
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
      errorIntegral = 0;
      lastError = 0;
    }
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

  // Stop motor first to ensure clean state
  stopMotor();
  delay(200);
  
  // Disable motion control temporarily during homing
  bool wasSystemEnabled = systemEnabled;
  systemEnabled = false;

  if (leftPressed()) {
    Serial.println(F("At limit"));
    
    // Stop motor immediately
    stopMotor();
    delay(200);

    // Hold gently at limit to ensure stable position
    long lastPos = encoder.read();
    unsigned long holdStart = millis();
    int stableTicks = 0;
    // Use gentle holding voltage - just enough to overcome friction and hold position
    float holdVoltage = max(FRICTION_RIGHT, 1.5);  // Gentle holding voltage (1.5V minimum)

    while (millis() - holdStart < CALIBRATE_HOLD_TIME || stableTicks < CALIBRATE_STABLE_TICKS) {
      // Only apply small holding voltage
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

    // STOP motor completely
    stopMotor();
    delay(500);  // Longer delay to ensure motor fully stops

    // Force encoder to zero - multiple aggressive attempts
    for (int i = 0; i < 10; i++) {
      encoder.write(0);
      delay(50);
      long checkPos = encoder.read();
      if (abs(checkPos) <= 1) {
        break;  // Successfully zeroed
      }
    }
    
    // Verify encoder is actually at zero
    long finalPos = encoder.read();
    int attempts = 0;
    while (abs(finalPos) > 1 && attempts < 5) {
      encoder.write(0);
      delay(100);
      finalPos = encoder.read();
      attempts++;
    }

    // Final stop
    stopMotor();
    delay(300);
    
    // CRITICAL: Reset ALL position tracking variables
    previousMotorPosition = 0;
    previousVelCompTime = micros();
    motorVelocity = 0;
    errorIntegral = 0;
    lastError = 0;
    lastStuckCheckPos = 0;
    lastStuckCheckTime = 0;
    stuckCounter = 0;
    
    // Force one final encoder read with multiple readings for stability
    delay(100);
    finalPos = encoder.read();
    delay(50);
    long pos2 = encoder.read();
    delay(50);
    long pos3 = encoder.read();
    
    // Use the value closest to 0
    if (abs(pos2) < abs(finalPos)) finalPos = pos2;
    if (abs(pos3) < abs(finalPos)) finalPos = pos3;
    
    if (abs(finalPos) > 1) {
      // Last resort - force zero one more time
      stopMotor();
      delay(300);
      encoder.write(0);
      delay(200);
      finalPos = encoder.read();
      // Reset all tracking again
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
    }
    
    Serial.print(F("Homed:"));
    Serial.print(finalPos);
    if (abs(finalPos) <= 1) {
      Serial.println(F(" OK"));
    } else {
      Serial.print(F(" WARN:"));
      Serial.println(finalPos);
    }
    
    systemEnabled = wasSystemEnabled;
    return true;
  }

  Serial.println(F("Homing..."));
  unsigned long startTime = millis();
  float initialVoltage = 2.0;  // Start with low voltage
  float currentVoltage = initialVoltage;
  float maxVoltage = 5.5;  // Maximum voltage to use
  float voltageIncrement = 0.2;  // Increase voltage by 0.2V every 200ms
  unsigned long lastVoltageIncrease = millis();
  const unsigned long VOLTAGE_RAMP_INTERVAL = 200;  // Increase voltage every 200ms
  const unsigned long HOMING_TIMEOUT = 15000;  // 15 second timeout
  const unsigned long DEBOUNCE_TIME = 100;  // Debounce time for limit switch
  bool limitDetected = false;
  unsigned long limitDetectTime = 0;

  setMotor(currentVoltage);

  // Wait for limit switch with gradual voltage increase and debouncing
  while ((millis() - startTime) < HOMING_TIMEOUT) {
    delay(10);
    
    // Check for limit switch with debouncing
    if (leftPressed()) {
      if (!limitDetected) {
        // First detection - start debounce timer
        limitDetected = true;
        limitDetectTime = millis();
      } else {
        // Already detected - check if debounce time has passed
        if ((millis() - limitDetectTime) >= DEBOUNCE_TIME) {
          // Limit switch confirmed - break out of loop
          break;
        }
      }
    } else {
      // Limit switch not pressed - reset detection
      if (limitDetected) {
        limitDetected = false;
        limitDetectTime = 0;
      }
    }
    
    // Gradually increase voltage if limit switch not reached
    if ((millis() - lastVoltageIncrease) >= VOLTAGE_RAMP_INTERVAL) {
      if (currentVoltage < maxVoltage) {
        currentVoltage += voltageIncrement;
        currentVoltage = min(currentVoltage, maxVoltage);
        setMotor(currentVoltage);
        lastVoltageIncrease = millis();
      }
    }
  }

  // STOP motor immediately when limit is detected
  stopMotor();
  delay(200);

  // Check if we successfully reached the limit switch
  if (leftPressed()) {
    Serial.println(F("Contact"));

    // Hold gently at limit to remove bounce
    long currentPos = encoder.read();
    long lastPos = currentPos;
    unsigned long holdStart = millis();
    int stableTicks = 0;
    // Use gentle holding voltage - just enough to overcome friction and hold position
    float holdVoltage = max(FRICTION_RIGHT, 1.5);  // Gentle holding voltage (1.5V minimum)

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

    // STOP motor completely
    stopMotor();
    delay(500);  // Longer delay to ensure motor fully stops

    // Force encoder to zero - multiple aggressive attempts
    for (int i = 0; i < 10; i++) {
      encoder.write(0);
      delay(50);
      long checkPos = encoder.read();
      if (abs(checkPos) <= 1) {
        break;  // Successfully zeroed
      }
    }
    
    // Verify encoder is actually at zero
    long finalPos = encoder.read();
    int attempts = 0;
    while (abs(finalPos) > 1 && attempts < 5) {
      encoder.write(0);
      delay(100);
      finalPos = encoder.read();
      attempts++;
    }

    // Final stop and verify
    stopMotor();
    delay(300);
    
    // CRITICAL: Reset ALL position tracking variables
    previousMotorPosition = 0;
    previousVelCompTime = micros();
    motorVelocity = 0;
    errorIntegral = 0;
    lastError = 0;
    lastStuckCheckPos = 0;
    lastStuckCheckTime = 0;
    stuckCounter = 0;
    
    // Force one final encoder read with multiple readings for stability
    delay(100);
    finalPos = encoder.read();
    delay(50);
    long pos2 = encoder.read();
    delay(50);
    long pos3 = encoder.read();
    
    // Use the value closest to 0
    if (abs(pos2) < abs(finalPos)) finalPos = pos2;
    if (abs(pos3) < abs(finalPos)) finalPos = pos3;
    
    if (abs(finalPos) > 1) {
      // Last resort - force zero one more time
      stopMotor();
      delay(300);
      encoder.write(0);
      delay(200);
      finalPos = encoder.read();
      // Reset all tracking again
      previousMotorPosition = 0;
      previousVelCompTime = micros();
      motorVelocity = 0;
    }
    
    Serial.print(F("Homed:"));
    Serial.print(finalPos);
    if (abs(finalPos) <= 1) {
      Serial.println(F(" OK"));
    } else {
      Serial.print(F(" WARN:"));
      Serial.println(finalPos);
    }
    
    systemEnabled = wasSystemEnabled;
    return true;
  } else {
    // Timeout - limit switch not reached
    stopMotor();
    delay(500);
    
    // Retry: check if we're close to the limit (maybe it bounced)
    // Try a few more times with lower voltage
    Serial.println(F("Timeout - retrying..."));
    for (int retry = 0; retry < 3; retry++) {
      // Try with lower voltage to avoid bouncing
      setMotor(2.0);
      delay(500);
      
      if (leftPressed()) {
        // Found it on retry - proceed with normal homing sequence
        stopMotor();
        delay(200);
        
        // Hold gently at limit
        long currentPos = encoder.read();
        long lastPos = currentPos;
        unsigned long holdStart = millis();
        int stableTicks = 0;
        float holdVoltage = max(FRICTION_RIGHT, 1.5);
        
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
        delay(500);
        
        // Reset encoder
        for (int i = 0; i < 10; i++) {
          encoder.write(0);
          delay(50);
          if (abs(encoder.read()) <= 1) {
            break;
          }
        }
        
        long finalPos = encoder.read();
        int attempts = 0;
        while (abs(finalPos) > 1 && attempts < 5) {
          encoder.write(0);
          delay(100);
          finalPos = encoder.read();
          attempts++;
        }
        
        stopMotor();
        delay(300);
        
        previousMotorPosition = 0;
        previousVelCompTime = micros();
        motorVelocity = 0;
        errorIntegral = 0;
        lastError = 0;
        lastStuckCheckPos = 0;
        lastStuckCheckTime = 0;
        stuckCounter = 0;
        
        delay(100);
        finalPos = encoder.read();
        delay(50);
        long pos2 = encoder.read();
        delay(50);
        long pos3 = encoder.read();
        
        if (abs(pos2) < abs(finalPos)) finalPos = pos2;
        if (abs(pos3) < abs(finalPos)) finalPos = pos3;
        
        if (abs(finalPos) > 1) {
          stopMotor();
          delay(300);
          encoder.write(0);
          delay(200);
          finalPos = encoder.read();
          previousMotorPosition = 0;
          previousVelCompTime = micros();
          motorVelocity = 0;
        }
        
        Serial.print(F("Homed (retry):"));
        Serial.print(finalPos);
        if (abs(finalPos) <= 1) {
          Serial.println(F(" OK"));
        } else {
          Serial.print(F(" WARN:"));
          Serial.println(finalPos);
        }
        
        systemEnabled = wasSystemEnabled;
        return true;
      }
      
      stopMotor();
      delay(300);
    }
    
    // All retries failed
    Serial.println(F("Homing failed after retries"));
    systemEnabled = wasSystemEnabled;
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

  Serial.println(F("Loaded"));
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

// ============================================
// MANUAL TARGET POSITION CALIBRATION
// ============================================
void manualCalibration() {
  Serial.println(F("\nCalibration"));
  Serial.println(F("R/L=move S=stop 1-4=save Q=quit"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
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
          Serial.print(F("Pos:"));
          Serial.println(pos);
          break;
          
        case '1':
          stopMotor();
          TARGET_1_POSITION = pos;
          targetPositions[0] = pos;
          Serial.print(F("L1:"));
          Serial.println(TARGET_1_POSITION);
          break;
          
        case '2':
          stopMotor();
          TARGET_2_POSITION = pos;
          targetPositions[1] = pos;
          Serial.print(F("L2:"));
          Serial.println(TARGET_2_POSITION);
          break;
          
        case '3':
          stopMotor();
          TARGET_3_POSITION = pos;
          targetPositions[2] = pos;
          WAIT_POSITION = TARGET_3_POSITION;
          Serial.print(F("L3:"));
          Serial.println(TARGET_3_POSITION);
          break;
          
        case '4':
          stopMotor();
          TARGET_4_POSITION = pos;
          targetPositions[3] = pos;
          Serial.print(F("L4:"));
          Serial.println(TARGET_4_POSITION);
          break;
          
        case 'Q':
          stopMotor();
          Serial.println(F("Done"));

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
// PID TUNING MODE
// ============================================
float readFloatFromSerial() {
  String inputString = "";
  unsigned long timeout = millis() + 30000;
  
  Serial.println(F("Value:"));
  
  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      
      if (c == '\n' || c == '\r') {
        if (inputString.length() > 0) {
          break;
        }
      } else if (c == 'C' || c == 'c') {
        return -999.0;
      } else if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
        inputString += c;
        Serial.print(c);  // Echo the character
      } else if (c == 8 || c == 127) {  // Backspace
        if (inputString.length() > 0) {
          inputString.remove(inputString.length() - 1);
          Serial.print(F("\b \b"));  // Erase character
        }
      }
    }
    delay(10);
  }
  
  Serial.println();
  
  if (inputString.length() == 0) {
    return -999.0;
  }
  
  float value = inputString.toFloat();
  return value;
}

void pidTuningMode() {
  Serial.println(F("\nPID Tune"));
  Serial.print(F("KP:"));
  Serial.print(KP, 3);
  Serial.print(F(" KI:"));
  Serial.print(KI, 3);
  Serial.print(F(" KD:"));
  Serial.println(KD, 3);
  Serial.println(F("P/I/D=select E=enter +/-/*/=adj R=reset S=save Q=quit"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
  char currentParam = 'P';  // Default to KP
  
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();
      
      cmd = toupper(cmd);
      
      switch (cmd) {
        case 'P':
          currentParam = 'P';
          Serial.print(F("KP:"));
          Serial.println(KP, 3);
          break;
          
        case 'I':
          currentParam = 'I';
          Serial.print(F("KI:"));
          Serial.println(KI, 3);
          break;
          
        case 'D':
          currentParam = 'D';
          Serial.print(F("KD:"));
          Serial.println(KD, 3);
          break;
          
        case 'E':
          {
            Serial.print(F("Enter "));
            if (currentParam == 'P') {
              Serial.print(F("KP:"));
            } else if (currentParam == 'I') {
              Serial.print(F("KI:"));
            } else {
              Serial.print(F("KD:"));
            }
            float newValue = readFloatFromSerial();
            if (newValue != -999.0 && newValue >= 0 && newValue <= 10.0) {
              if (currentParam == 'P') {
                KP = newValue;
                Serial.print(F("KP="));
                Serial.println(KP, 3);
              } else if (currentParam == 'I') {
                KI = newValue;
                Serial.print(F("KI="));
                Serial.println(KI, 3);
              } else if (currentParam == 'D') {
                KD = newValue;
                Serial.print(F("KD="));
                Serial.println(KD, 3);
              }
              KP_active = KP;
              KI_active = KI;
              KD_active = KD;
            }
          }
          break;
          
        case '+':
          if (currentParam == 'P') { KP *= 1.1; Serial.print(F("KP:")); Serial.println(KP, 3); }
          else if (currentParam == 'I') { KI *= 1.1; Serial.print(F("KI:")); Serial.println(KI, 3); }
          else if (currentParam == 'D') { KD *= 1.1; Serial.print(F("KD:")); Serial.println(KD, 3); }
          KP_active = KP; KI_active = KI; KD_active = KD;
          break;
          
        case '-':
          if (currentParam == 'P') { KP *= 0.9; Serial.print(F("KP:")); Serial.println(KP, 3); }
          else if (currentParam == 'I') { KI *= 0.9; Serial.print(F("KI:")); Serial.println(KI, 3); }
          else if (currentParam == 'D') { KD *= 0.9; Serial.print(F("KD:")); Serial.println(KD, 3); }
          KP_active = KP; KI_active = KI; KD_active = KD;
          break;
          
        case '*':
          if (currentParam == 'P') { KP *= 1.01; Serial.print(F("KP:")); Serial.println(KP, 3); }
          else if (currentParam == 'I') { KI *= 1.01; Serial.print(F("KI:")); Serial.println(KI, 3); }
          else if (currentParam == 'D') { KD *= 1.01; Serial.print(F("KD:")); Serial.println(KD, 3); }
          KP_active = KP; KI_active = KI; KD_active = KD;
          break;
          
        case '/':
          if (currentParam == 'P') { KP *= 0.99; Serial.print(F("KP:")); Serial.println(KP, 3); }
          else if (currentParam == 'I') { KI *= 0.99; Serial.print(F("KI:")); Serial.println(KI, 3); }
          else if (currentParam == 'D') { KD *= 0.99; Serial.print(F("KD:")); Serial.println(KD, 3); }
          KP_active = KP; KI_active = KI; KD_active = KD;
          break;
          
        case 'R':
          KP = 0.018; KI = 0.003; KD = 0.022;
          KP_active = KP; KI_active = KI; KD_active = KD;
          Serial.println(F("Reset"));
          break;
          
        case 'S':
          savePIDToEEPROM();
          Serial.println(F("Saved"));
          break;
          
        case 'Q':
          Serial.println(F("Done"));
          
          systemEnabled = wasEnabled;
          autoMode = wasAuto;
          return;
          
        default:
          break;
      }
    }
    
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
        Serial.println(F("\nAUTO MODE"));

        // Full rehoming with state reset
        stopMotor();
        delay(200);
        
        if (homeToLeftLimit()) {
          // CRITICAL: Aggressively reset encoder to zero with verification
          stopMotor();
          delay(300);
          
          // Multiple reset attempts with verification
          long checkPos = 999;
          for (int attempt = 0; attempt < 20; attempt++) {
            encoder.write(0);
            delay(100);
            checkPos = encoder.read();
            if (abs(checkPos) <= 1) {
              break;
            }
          }
          
          // If still not zero, try one more aggressive reset
          if (abs(checkPos) > 1) {
            stopMotor();
            delay(200);
            for (int i = 0; i < 10; i++) {
              encoder.write(0);
              delay(150);
              checkPos = encoder.read();
              if (abs(checkPos) <= 1) {
                break;
              }
            }
          }
          
          // Final verification - if still not zero, report error but continue
          checkPos = encoder.read();
          if (abs(checkPos) > 1) {
            Serial.print(F("ERROR: Encoder not zero after reset: "));
            Serial.println(checkPos);
            // Force one last time
            encoder.write(0);
            delay(200);
            checkPos = encoder.read();
          }
          
          // CRITICAL: Reset ALL position tracking variables to match encoder
          previousMotorPosition = 0;
          previousVelCompTime = micros();
          motorVelocity = 0;
          errorIntegral = 0;
          lastError = 0;
          lastStuckCheckPos = 0;
          lastStuckCheckTime = 0;
          stuckCounter = 0;
          
          // Reset all state variables
          autoMode = true;
          systemEnabled = true;
          sensorCalibrated = true;  // Using permanent sensor ranges, no calibration needed
          dynamicCalibrationActive = false;
          initialRangeFinding = true;  // Start with range finding sequence
          rangeFindingComplete = false;  // Will be set after range is found
          
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

          // Final verification - read encoder one more time
          long finalEncoderPos = encoder.read();
          
          // If encoder is still not zero, this is a critical error
          if (abs(finalEncoderPos) > 1) {
            Serial.print(F("CRITICAL: Encoder reset failed, reading: "));
            Serial.println(finalEncoderPos);
            // Last resort - try resetting one more time
            stopMotor();
            delay(500);
            encoder.write(0);
            delay(200);
            finalEncoderPos = encoder.read();
            // Force all tracking to match
            previousMotorPosition = 0;
            previousVelCompTime = micros();
            motorVelocity = 0;
          }
          
          Serial.print(F("Ready (enc:"));
          Serial.print(finalEncoderPos);
          if (abs(finalEncoderPos) <= 1) {
            Serial.println(F(" OK)"));
          } else {
            Serial.print(F(" FAIL:"));
            Serial.println(finalEncoderPos);
          }
        } else {
          Serial.println(F("Homing failed"));
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
        Serial.println(F("Stop auto first"));
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
    
    case 'T':
      pidTuningMode();
      break;
    
    case 'R':
      adaptiveFrictionVoltage = 0;
      adaptiveLearning = false;
      adaptiveLearned = false;
      rangeFindingComplete = true;
      sensorCalibrated = true;
      adaptiveFrictionLeft = FRICTION_LEFT;
      adaptiveFrictionRight = FRICTION_RIGHT;
      break;

    case 'L':
      loadCalibrationFromEEPROM();
      break;

    case 'W':
      saveTargetsToEEPROM();
      savePIDToEEPROM();
      saveFrictionToEEPROM();
      Serial.println(F("Saved"));
      break;

    default:
      break;
  }
}

void setTargetLane(int lane) {
  if (lane < 1 || lane > 4) return;

  desiredPosition = targetPositions[lane - 1];
  
  // Constrain desired position to safe bounds
  if (desiredPosition < UPPER_BOUND - SAFETY_MARGIN_RIGHT) {
    desiredPosition = UPPER_BOUND - SAFETY_MARGIN_RIGHT;
  }
  if (desiredPosition > SAFETY_MARGIN_LEFT && !leftPressed()) {
    desiredPosition = SAFETY_MARGIN_LEFT;
  }
  
  errorIntegral = 0;
  lastError = 0;
  moveStartTime = millis();
  targetReached = false;
  stuckCounter = 0;  // Reset stuck detection for new movement
  positionRetryCount = 0;  // Reset retry counter for new movement
  inRetryMode = false;  // Reset retry mode for new movement
  
  long currentPos = encoder.read();
  long error = desiredPosition - currentPos;
  
  Serial.print(F("L"));
  Serial.print(lane);
  Serial.print(F(":"));
  Serial.print(currentPos);
  Serial.print(F("->"));
  Serial.println(desiredPosition);
}

// ============================================
// DISPLAY FUNCTIONS
// ============================================
void printWelcome() {
  Serial.println(F("\nPVZ Robot"));
}

void printHelp() {
  Serial.println(F("\nCOMMANDS:"));
  Serial.println(F("C-Calibrate Z-Home G-Auto S-Stop"));
  Serial.println(F("1-4:Lanes P-Status D-Sensors M-Monitor"));
  Serial.println(F("T-PIDTune R-Reset L-Load W-Save H-Help"));
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
  
  Serial.print(F(" | Z:"));
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction == FORWARD) {
      Serial.print(F(">"));
    } else if (ProxSensors[i].direction == BACKWARD) {
      Serial.print(F("<"));
    } else {
      Serial.print(F("-"));
    }
    Serial.print((int)((1.0 - zombieDistances[i]) * 100));
    Serial.print(F(" "));
  }
  Serial.println();
}

void printStatus() {
  long currentPos = encoder.read();
  float error = desiredPosition - currentPos;
  
  Serial.println(F("\n=== STATUS ==="));
  Serial.print(F("Mode:"));
  Serial.println(autoMode ? F("AUTO") : F("MAN"));
  Serial.print(F("State:"));
  switch(currentState) {
    case CALIBRATE: Serial.println(F("CAL")); break;
    case FIND_RANGE: Serial.println(F("RANGE")); break;
    case CHOOSE_ACTIVE_TARGET: Serial.println(F("CHOOSE")); break;
    case MOVE_TO_TARGET: Serial.println(F("MOVE")); break;
  }
  Serial.print(F("Pos:"));
  Serial.print(currentPos);
  Serial.print(F(" Tgt:"));
  Serial.print(desiredPosition);
  Serial.print(F(" Err:"));
  Serial.print((int)error);
  Serial.print(F(" Vel:"));
  Serial.println((int)motorVelocity);
  
  Serial.print(F("Lane:"));
  Serial.println(activeTargetIndex >= 0 ? activeTargetIndex + 1 : 0);
  Serial.print(F("PID:"));
  Serial.print(KP, 3);
  Serial.print(F(","));
  Serial.print(KI, 3);
  Serial.print(F(","));
  Serial.println(KD, 3);
}

void printAllSensors() {
  long pos = encoder.read();
  Serial.print(F("\nEnc:"));
  Serial.print(pos);
  Serial.print(F(" L:"));
  Serial.print(leftPressed() ? 1 : 0);
  Serial.print(F(" R:"));
  Serial.println(rightPressed() ? 1 : 0);
  
  Serial.print(F("Raw:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(analogRead(ProxSensors[i].pin));
    Serial.print(F(" "));
  }
  Serial.print(F("Filt:"));
  for (int i = 0; i < 4; i++) {
    Serial.print((int)ProxSensors[i].currVal);
    Serial.print(F(" "));
  }
  Serial.println();
}

void continuousMonitor() {
  Serial.println(F("\nMONITOR (key to stop)"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
  while (!Serial.available()) {
    updateAllSensors();
    long pos = encoder.read();
    Serial.print(F("E:"));
    Serial.print(pos);
    Serial.print(F(" L:"));
    Serial.print(leftPressed() ? 1 : 0);
    Serial.print(F(" R:"));
    Serial.print(rightPressed() ? 1 : 0);
    Serial.print(F(" P:"));
    for (int i = 0; i < 4; i++) {
      Serial.print((int)ProxSensors[i].currVal);
      Serial.print(F(","));
    }
    Serial.println();
    delay(100);
  }
  while (Serial.available()) Serial.read();
  Serial.println(F("Done"));
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
