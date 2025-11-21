// PVZ

#include <Encoder.h>
#include <EEPROM.h>

// PINS
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

// EEPROM
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

const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

// TARGETS
long TARGET_1_POSITION = -74;
long TARGET_2_POSITION = -307;
long TARGET_3_POSITION = -547;
long TARGET_4_POSITION = -1080;

const long WAIT_POSITION_OFFSET = 2;
long WAIT_POSITION = TARGET_3_POSITION;
long LOWER_BOUND = 0;
long UPPER_BOUND = -1424;
long RANGE_MIDPOINT = (LOWER_BOUND + UPPER_BOUND) / 2;

long targetPositions[4] = {
  TARGET_1_POSITION,
  TARGET_2_POSITION,
  TARGET_3_POSITION,
  TARGET_4_POSITION
};

// SENSOR CALIBRATION
int ProxRange[4][2] = {
  {800, 100},
  {800, 100},
  {800, 100},
  {800, 100}
};

bool sensorCalibrated = false;
bool dynamicCalibrationActive = false;
bool rangeFindingComplete = false;
bool rangeFindingActive = false;
unsigned long calibrationStartTime = 0;
const unsigned long DYNAMIC_CALIBRATION_TIME = 10000;

// Range vars
enum RangeFindingState {
  RANGE_IDLE,
  RANGE_MOVE_TO_LEFT,
  RANGE_HOLD_LEFT,
  RANGE_MOVE_TO_RIGHT,
  RANGE_HOLD_RIGHT
};
RangeFindingState rangeFindingState = RANGE_IDLE;
unsigned long rangeFindingStartTime = 0;
long rangeFindingLastPosition = 0;
unsigned long rangeFindingLastMoveTime = 0;
float rangeFindingDriveVoltage = 0;
long rangeFindingLastPos = 0;
int rangeFindingStableTicks = 0;
unsigned long rangeFindingHoldStart = 0;

int dynamicMin[4];
int dynamicMax[4];

// SENSORS
struct ProximitySensor {
  float currVal;
  float prevVal;
  float prevValForVelocity;
  unsigned long prevChangeTime;
  unsigned long lastVelocityUpdate;
  int pin;
  int direction;
  int prevDirection;
  int forwardCount;
  int backwardCount;
  float velocity;
  float prevVelocity;
  bool hitDetected;
  unsigned long hitTime;
};

ProximitySensor ProxSensors[4];

// Filter
const float alpha = 0.925;
const int stopTimeout = 150;
const int lowerNoiseLimit = 5;
const int upperNoiseLimit = 8;
const int noiseThreshold = 225;
int noiseLimit = 8;

// Target
int activeTargetIndex = -1;
int previousTargetIndex = -1;
long activeTargetPosition = WAIT_POSITION;
float closestZombieDist = 2.0;
float zombieDistances[4];
bool WAIT_POS = true;

long previousMoveStartPosition = 0;
bool fineAdjustmentActive = false;
long fineAdjustmentTarget = 0;
const int FINE_ADJUSTMENT_AMOUNT = 2;
float previousZombieDistance = 1.0;
int fineAdjustmentCount = 0;
const int MAX_FINE_ADJUSTMENTS = 5;
unsigned long lastFineAdjustmentTime = 0;
const unsigned long MIN_FINE_ADJUSTMENT_INTERVAL = 50;

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

// PID
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
const int RIGHTWARD_DRIFT_OFFSET = 2;
const int LEFTWARD_DRIFT_OFFSET = 2;

// MOTION
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

// SYS
bool systemEnabled = false;
bool autoMode = false;
unsigned long lastPrintTime = 0;
unsigned long moveStartTime = 0;
bool targetReached = false;

const float CALIBRATION_VOLTAGE = 4.0;
const float HOMING_VOLTAGE = 4.0;
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

  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);
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

// LOOP
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

// UPDATE
void updateAllSensors() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 4; i++) {
    ProxSensors[i].prevDirection = ProxSensors[i].direction;
    ProxSensors[i].prevVelocity = ProxSensors[i].velocity;
    float rawReading = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal +
                             (1.0 - alpha) * rawReading;
    unsigned long timeDelta = currentTime - ProxSensors[i].lastVelocityUpdate;
    if (timeDelta > 0) {
      float valueDelta = ProxSensors[i].currVal - ProxSensors[i].prevValForVelocity;
      ProxSensors[i].velocity = (valueDelta * 1000.0) / timeDelta;
      ProxSensors[i].lastVelocityUpdate = currentTime;
      ProxSensors[i].prevValForVelocity = ProxSensors[i].currVal;
    } else {
      ProxSensors[i].velocity = 0.0;
    }

    if (ProxSensors[i].currVal >= noiseThreshold) {
      noiseLimit = upperNoiseLimit;
    } else {
      noiseLimit = lowerNoiseLimit;
    }

    if (abs(ProxSensors[i].currVal - ProxSensors[i].prevVal) < noiseLimit) {
      if (timeDelta >= stopTimeout) {
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
        ProxSensors[i].prevChangeTime = currentTime;
        if (ProxSensors[i].hitDetected) {
          ProxSensors[i].hitDetected = false;
          ProxSensors[i].hitTime = 0;
        }
      }
    } else {
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      if (ProxSensors[i].backwardCount > 3) {
        ProxSensors[i].direction = BACKWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = currentTime;
      }
    }

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

    if (sensorCalibrated) {
      zombieDistances[i] = (ProxSensors[i].currVal - ProxRange[i][1]) /
                           (float)(ProxRange[i][0] - ProxRange[i][1]);
      zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);
    } else {
      zombieDistances[i] = 1.0;
    }
  }
}

// VEL
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

// RANGE
void findRange() {
  if (rangeFindingComplete) {
    return;
  }
  
  if (!rangeFindingActive) {
    rangeFindingActive = true;
    rangeFindingStartTime = millis();
    Serial.println(F("\nRANGE..."));
    long currentPos = encoder.read();
    
    if (leftPressed()) {
      Serial.println(F("At left"));
      rangeFindingState = RANGE_HOLD_LEFT;
      rangeFindingHoldStart = millis();
      rangeFindingLastPos = currentPos;
      rangeFindingStableTicks = 0;
      
      // When holding at left limit, use FRICTION_RIGHT (voltage to move left/positive)
      float frictionForHold = max(FRICTION_RIGHT, 1.55);
      float holdVoltage = max(frictionForHold, CALIBRATE_MIN_VOLTAGE - 0.5);
      setMotor(holdVoltage);
      return;
    }
    
    rangeFindingState = RANGE_MOVE_TO_LEFT;
    float frictionForLeft = max(FRICTION_RIGHT, 1.55);  // Minimum 1.55V if friction not characterized
    rangeFindingDriveVoltage = max(frictionForLeft + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
    setMotor(rangeFindingDriveVoltage);
    rangeFindingLastPosition = currentPos;
    rangeFindingLastMoveTime = millis();
    return;
  }
  
  long currentPos = encoder.read();
  unsigned long currentTime = millis();
  
  switch (rangeFindingState) {
    case RANGE_MOVE_TO_LEFT:
      if (leftPressed()) {
        rangeFindingState = RANGE_HOLD_LEFT;
        rangeFindingHoldStart = currentTime;
        rangeFindingLastPos = currentPos;
        rangeFindingStableTicks = 0;
        
        float frictionForHold = max(FRICTION_RIGHT, 1.55);
        float holdVoltage = max(frictionForHold, CALIBRATE_MIN_VOLTAGE - 0.5);
        setMotor(holdVoltage);
          Serial.println(F("L"));
      } else {
        if (abs(currentPos - rangeFindingLastPosition) > 2) {
          rangeFindingLastMoveTime = currentTime;
          rangeFindingLastPosition = currentPos;
        } else if (currentTime - rangeFindingLastMoveTime > 2000) {
          Serial.println(F("Stuck"));
          rangeFindingDriveVoltage = min(rangeFindingDriveVoltage + 0.8, 9.0);
          setMotor(rangeFindingDriveVoltage);
          rangeFindingLastMoveTime = currentTime;
        }
        if (currentTime - rangeFindingStartTime > 15000) {
          Serial.println(F("Timeout: left limit"));
          stopMotor();
          rangeFindingActive = false;
          rangeFindingState = RANGE_IDLE;
        }
      }
      break;
      
    case RANGE_HOLD_LEFT:
      if (currentTime - rangeFindingHoldStart >= CALIBRATE_HOLD_TIME && rangeFindingStableTicks >= CALIBRATE_STABLE_TICKS) {
        stopMotor();
        if (currentTime - rangeFindingHoldStart >= CALIBRATE_HOLD_TIME + 200) {
          encoder.write(0);
          LOWER_BOUND = 0;
          Serial.println(F("Left limit found"));
          rangeFindingState = RANGE_MOVE_TO_RIGHT;
          rangeFindingStartTime = currentTime;
          rangeFindingLastPosition = 0;
          rangeFindingLastMoveTime = currentTime;
          float frictionForRight = max(FRICTION_LEFT, 1.55);  // Minimum 1.55V if friction not characterized
          rangeFindingDriveVoltage = -max(frictionForRight + CALIBRATE_EXTRA_VOLTAGE, 3.0);  // At least 3.0V for right limit
          Serial.print(F("R:"));
          Serial.println(-rangeFindingDriveVoltage, 1);
          setMotor(rangeFindingDriveVoltage);
        }
      } else {
        if (abs(currentPos - rangeFindingLastPos) <= 1) {
          rangeFindingStableTicks++;
        } else {
          rangeFindingStableTicks = 0;
          rangeFindingLastPos = currentPos;
        }
      }
      break;
      
    case RANGE_MOVE_TO_RIGHT:
      if (rightPressed()) {
        rangeFindingState = RANGE_HOLD_RIGHT;
        rangeFindingHoldStart = currentTime;
        rangeFindingLastPos = currentPos;
        rangeFindingStableTicks = 0;
        
        float frictionForHold = max(FRICTION_LEFT, 1.55);
        float holdVoltageRight = -max(frictionForHold, CALIBRATE_MIN_VOLTAGE - 0.5);
        setMotor(holdVoltageRight);
          Serial.println(F("R"));
      } else {
        if (abs(currentPos - rangeFindingLastPosition) > 2) {
          rangeFindingLastMoveTime = currentTime;
          rangeFindingLastPosition = currentPos;
        } else if (currentTime - rangeFindingLastMoveTime > 2000) {
          Serial.print(F("Stuck:"));
          rangeFindingDriveVoltage = max(rangeFindingDriveVoltage - 0.8, -9.0);
          if (rangeFindingDriveVoltage > -3.0) {
            rangeFindingDriveVoltage = -3.0;
          }
          Serial.println(-rangeFindingDriveVoltage, 1);
          setMotor(rangeFindingDriveVoltage);
          rangeFindingLastMoveTime = currentTime;
        }
        if (currentTime - rangeFindingStartTime > 15000) {
          Serial.println(F("Timeout: right limit"));
          stopMotor();
          rangeFindingActive = false;
          rangeFindingState = RANGE_IDLE;
        }
      }
      break;
      
    case RANGE_HOLD_RIGHT:
      if (abs(currentPos - rangeFindingLastPos) <= 1) {
        rangeFindingStableTicks++;
      } else {
        rangeFindingStableTicks = 0;
        rangeFindingLastPos = currentPos;
      }
      
      if (currentTime - rangeFindingHoldStart >= CALIBRATE_HOLD_TIME && rangeFindingStableTicks >= CALIBRATE_STABLE_TICKS) {
        stopMotor();
        delay(200);
        UPPER_BOUND = encoder.read();
        RANGE_MIDPOINT = (LOWER_BOUND + UPPER_BOUND) / 2;
        Serial.print(F("R:"));
        Serial.print(UPPER_BOUND);
        Serial.print(F(" Rng:"));
        Serial.println(abs(UPPER_BOUND - LOWER_BOUND));
        if (abs(UPPER_BOUND - LOWER_BOUND) < 100) {
          Serial.println(F("ERR:Range"));
          rangeFindingActive = false;
          rangeFindingState = RANGE_IDLE;
          return;
        }
        rangeFindingComplete = true;
        rangeFindingActive = false;
        rangeFindingState = RANGE_IDLE;
        Serial.println(F("OK"));
        
      }
      break;
      
    case RANGE_IDLE:
      break;
  }
}

// SENSOR CALIBRATION
void startDynamicCalibration() {
  Serial.println(F("\nCAL (10s)"));
  
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
  
  Serial.println(F("\nCAL OK"));
  
  bool allValid = true;
  
  for (int i = 0; i < 4; i++) {
    int range = dynamicMax[i] - dynamicMin[i];
    
    Serial.print(F("L"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(dynamicMin[i]);
    Serial.print(F("-"));
    Serial.print(dynamicMax[i]);
    Serial.print(F(" ("));
    Serial.print(range);
    Serial.print(F(")"));
    
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
    Serial.println(F("\nOK"));
  } else {
    Serial.println(F("\nWARN"));
    sensorCalibrated = true;
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
      
      if (!rangeFindingComplete) {
        currentState = FIND_RANGE;
        systemEnabled = true;
      }
      else if (!dynamicCalibrationActive && !sensorCalibrated) {
        startDynamicCalibration();
        desiredPosition = LOWER_BOUND;
        systemEnabled = true;
      }
      else if (!dynamicCalibrationActive && sensorCalibrated) {
        // After sensor calibration, ensure we're at the left limit (position 0)
        // Move there if we're not already there
        long currentPos = encoder.read();
        if (currentPos < -50) {
          // We're still at or near the right limit, move to left limit first
          desiredPosition = LOWER_BOUND;
          // Stay in CALIBRATE state until we reach the left limit
        } else {
          // We're close to the left limit, reset encoder to 0 and proceed
          if (leftPressed() && abs(currentPos) <= 10) {
            encoder.write(0);
            delay(50);
            if (encoder.read() != 0) {
              encoder.write(0);
            }
          }
          currentState = CHOOSE_ACTIVE_TARGET;
          systemEnabled = true;
        }
      }
      break;
    
    case FIND_RANGE:
      findRange();
      if (rangeFindingComplete) {
        if (!sensorCalibrated) {
          startDynamicCalibration();
          desiredPosition = LOWER_BOUND;
        } else {
          // After range finding, we're at the right limit (UPPER_BOUND)
          // Move back to left limit and reset encoder to 0
          // But first, ensure encoder position is correct
          long currentPos = encoder.read();
          // If we're at the right limit, the encoder should be at UPPER_BOUND
          // But we want to be at 0 (left limit) for normal operation
          // So we'll let the motion control move us back to LOWER_BOUND (0)
          // and checkLimitSwitches will reset the encoder when we hit the left limit
          desiredPosition = LOWER_BOUND;  // Move to left limit (position 0)
          currentState = CHOOSE_ACTIVE_TARGET;
        }
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      activeTargetIndex = -1;
      closestZombieDist = 2.0;
      previousMoveStartPosition = encoder.read();
      
      for (int i = 1; i <= 2; i++) {
        if (ProxSensors[i].direction == FORWARD &&
            zombieDistances[i] < closestZombieDist) {
          closestZombieDist = zombieDistances[i];
          activeTargetIndex = i;
        }
      }
      
      if (activeTargetIndex == -1) {
        // No targets in priority lanes, check all lanes
        for (int i = 0; i < 4; i++) {
          if (ProxSensors[i].direction == FORWARD &&
              zombieDistances[i] < closestZombieDist) {
            closestZombieDist = zombieDistances[i];
            activeTargetIndex = i;
          }
        }
      } else {
        for (int i = 0; i < 4; i += 3) {  // Check lanes 1 (i=0) and 4 (i=3)
          if (ProxSensors[i].direction == FORWARD &&
              zombieDistances[i] < (closestZombieDist - 0.15)) {
            closestZombieDist = zombieDistances[i];
            activeTargetIndex = i;
          }
        }
      }
      
      if (activeTargetIndex >= 0) {
        activeTargetPosition = targetPositions[activeTargetIndex];
        WAIT_POS = false;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        previousZombieDistance = zombieDistances[activeTargetIndex];
        
        int percentToPhoto = (int)((1.0 - zombieDistances[activeTargetIndex]) * 100);
        
        Serial.print(F("T:L"));
        Serial.print(activeTargetIndex + 1);
        Serial.print(F(" "));
        Serial.println(percentToPhoto);
        
        previousTargetIndex = activeTargetIndex;
        targetHitTime = 0;
        
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        Serial.println(F("Wait"));
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
      if (fineAdjustmentActive) {
        desiredPosition = fineAdjustmentTarget;
      } else {
        desiredPosition = activeTargetPosition;
      }
      
      long currentPos = encoder.read();
      long error = desiredPosition - currentPos;
      
      if (currentPos < UPPER_BOUND - 50) {
        Serial.println(F("NearR"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
      if (activeTargetIndex >= 0 && !WAIT_POS) {
        int targetDirection = ProxSensors[activeTargetIndex].direction;
        bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
        unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
        
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
        
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
            Serial.println(F("Chg"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }
        if (targetDirection == BACKWARD &&
            zombieDistances[activeTargetIndex] > 0.80) {
            Serial.println(F("Safe"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }

        bool currentIsPriority = (activeTargetIndex == 1 || activeTargetIndex == 2);
        for (int i = 0; i < 4; i++) {
          if (i != activeTargetIndex &&
              ProxSensors[i].direction == FORWARD) {
            bool candidateIsPriority = (i == 1 || i == 2);
            float distanceThreshold = 0.20;
            
            if (currentIsPriority && !candidateIsPriority) {
              distanceThreshold = 0.30;
            } else if (!currentIsPriority && candidateIsPriority) {
              distanceThreshold = 0.15;
            }
            
            if (zombieDistances[i] < zombieDistances[activeTargetIndex] - distanceThreshold) {
              Serial.print(F("Closer:L"));
              Serial.println(i + 1);
              currentState = CHOOSE_ACTIVE_TARGET;
              break;
            }
          }
        }
      }
      
      bool hasForwardZombie = false;
      for (int i = 0; i < 4; i++) {
        if (ProxSensors[i].direction == FORWARD) {
          hasForwardZombie = true;
          break;
        }
      }

      if (!hasForwardZombie && millis() - moveStartTime > 1000) {
        Serial.println(F("NoThr"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }

      long errorToCurrentTarget = desiredPosition - currentPos;
      long errorToOriginalTarget = activeTargetPosition - currentPos;
      if (activeTargetIndex >= 0 && !WAIT_POS && !fineAdjustmentActive &&
          fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
          abs(errorToOriginalTarget) > 3 &&
          (millis() - lastFineAdjustmentTime) >= (MIN_FINE_ADJUSTMENT_INTERVAL * 2) &&
          abs(motorVelocity) < 20 &&
          abs(errorToOriginalTarget) <= 6 &&
          ProxSensors[activeTargetIndex].direction == FORWARD &&
          !ProxSensors[activeTargetIndex].hitDetected &&
          zombieDistances[activeTargetIndex] < 0.30) {
        
        fineAdjustmentTarget = activeTargetPosition;
        
        fineAdjustmentCount++;
        lastFineAdjustmentTime = millis();
        Serial.print(F("Adj:"));
        Serial.print(errorToOriginalTarget);
        Serial.print(F(" ("));
        Serial.print(fineAdjustmentCount);
        Serial.println(F(")"));
        
        fineAdjustmentActive = true;
        desiredPosition = fineAdjustmentTarget;
        arrivalTime = millis();
      }
      
      if (abs(errorToCurrentTarget) <= 2) {
        if (WAIT_POS) {
          if (hasForwardZombie) {
            Serial.println(F("Threat"));
            currentState = CHOOSE_ACTIVE_TARGET;
          }
        } else if (millis() - arrivalTime > targetActivateTime) {
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
              Serial.println(F("Retreat"));
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            else if (ProxSensors[activeTargetIndex].direction == FORWARD && 
                     !fineAdjustmentActive &&
                     fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
                     abs(errorToOriginalTarget) > 3 &&
                     (millis() - lastFineAdjustmentTime) >= (MIN_FINE_ADJUSTMENT_INTERVAL * 3) &&
                     abs(motorVelocity) < 15 &&
                     abs(errorToOriginalTarget) <= TARGET_BAND + 3 &&
                     !ProxSensors[activeTargetIndex].hitDetected) {
              
              if (abs(errorToOriginalTarget) > 3) {
                fineAdjustmentTarget = activeTargetPosition;
                fineAdjustmentCount++;
                lastFineAdjustmentTime = millis();
                Serial.print(F("Fine adj: "));
                Serial.print(errorToOriginalTarget);
                Serial.print(F("->0 ("));
                Serial.print(fineAdjustmentCount);
                Serial.println(F(")"));
                fineAdjustmentActive = true;
                desiredPosition = fineAdjustmentTarget;
                arrivalTime = millis();
              }
            }
            else if (ProxSensors[activeTargetIndex].direction == FORWARD && 
                     fineAdjustmentActive &&
                     fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
                     abs(errorToCurrentTarget) > 3 &&
                     (millis() - lastFineAdjustmentTime) >= (MIN_FINE_ADJUSTMENT_INTERVAL * 2) &&
                     abs(motorVelocity) < 15 &&
                     !ProxSensors[activeTargetIndex].hitDetected &&
                     (millis() - arrivalTime) >= 200) {
              fineAdjustmentTarget = activeTargetPosition;
              fineAdjustmentCount++;
              lastFineAdjustmentTime = millis();
              Serial.print(F("Fine adj: "));
              Serial.print(errorToCurrentTarget);
              Serial.print(F("->0 ("));
              Serial.print(fineAdjustmentCount);
              Serial.println(F(")"));
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
        if (fineAdjustmentActive && abs(errorToCurrentTarget) > 3) {
          fineAdjustmentActive = false;
          fineAdjustmentCount = 0;
          lastFineAdjustmentTime = 0;
        }
      }
      break;
  }
}

// MOTION
void runMotionControl() {
  long currentPosition = encoder.read();
  
  float originalError = desiredPosition - currentPosition;
  long absOriginalError = abs(originalError);
  
  if (absOriginalError <= 2) {
    stopMotor();
    errorIntegral = 0;
    lastError = 0;
    return;
  }
  
  long adjustedDesiredPosition = desiredPosition;
  if (!fineAdjustmentActive) {
    if (currentPosition > desiredPosition) {
      adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
    } else if (currentPosition < desiredPosition) {
      adjustedDesiredPosition = desiredPosition - LEFTWARD_DRIFT_OFFSET;
    }
  }
  
  float error = adjustedDesiredPosition - currentPosition;
  
  if (rangeFindingActive) {
    return;
  }
  
  if (currentState == MOVE_TO_TARGET && autoMode) {
    if (currentPosition > 50) {
      Serial.println(F("Drift"));
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
  
  bool atTarget = false;
  
  if (atTarget) {
    if (abs(motorVelocity) > 8) {
      float dampingVoltage = -constrain(motorVelocity * 0.025, -1.5, 1.5);
      setMotor(dampingVoltage);
      errorIntegral *= 0.85;
      lastError = 0;
    } else {
      stopMotor();
      errorIntegral = 0;
      lastError = 0;
    }
    adaptiveLearning = false;

    if (!targetReached) {
      targetReached = true;
      unsigned long settleTime = millis() - moveStartTime;
      Serial.print(F("@"));
      Serial.print(currentPosition);
      Serial.print(F(" e:"));
      Serial.print(originalError);
      Serial.print(F(" t:"));
      Serial.println(settleTime / 1000.0, 1);
    }
    return;
  }

  targetReached = false;

  if (abs(originalError) > 3 && abs(originalError) < 50) {
    if (millis() - moveStartTime > 300 && positionRetryCount < MAX_POSITION_RETRIES) {
      positionRetryCount++;
      Serial.print(F("Stuck:"));
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
    
    bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
    Serial.print(F("Frict:"));
    Serial.println(inLeftHalf ? F("L") : F("R"));
  }
  
  if (adaptiveLearning) {
    long positionChange = abs(currentPosition - lastAdaptivePosition);
    unsigned long elapsed = millis() - adaptiveStartTime;
    
    if (positionChange >= 5) {
      bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
      if (inLeftHalf) {
        adaptiveFrictionLeft = adaptiveFrictionVoltage;
        FRICTION_LEFT = adaptiveFrictionVoltage;
      } else {
        adaptiveFrictionRight = adaptiveFrictionVoltage;
        FRICTION_RIGHT = adaptiveFrictionVoltage;
      }
      
      Serial.print(F("F:"));
      Serial.print(adaptiveFrictionVoltage, 1);
      Serial.print(F(" "));
      Serial.println(inLeftHalf ? F("L") : F("R"));
      
      adaptiveLearning = false;
      adaptiveLearned = true;
      float voltage = (error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage;
      setMotor(voltage);
      return;
    }
    
    if (elapsed >= 150) {
      adaptiveFrictionVoltage += 0.3;
      adaptiveStartTime = millis();
      lastAdaptivePosition = currentPosition;
      
      if (adaptiveFrictionVoltage > 4.5) {
        Serial.println(F("F:2.5"));
        adaptiveFrictionVoltage = 2.5;
        adaptiveLearning = false;
        adaptiveLearned = true;
        bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
        if (inLeftHalf) {
          adaptiveFrictionLeft = 2.5;
          FRICTION_LEFT = 2.5;
        } else {
          adaptiveFrictionRight = 2.5;
          FRICTION_RIGHT = 2.5;
        }
      }
    }
    
    if (elapsed > 2000) {
      Serial.println(F("F:TO"));
      adaptiveLearning = false;
      adaptiveLearned = true;
      adaptiveFrictionVoltage = 2.0;
      bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
      if (inLeftHalf) {
        adaptiveFrictionLeft = 2.0;
        FRICTION_LEFT = 2.0;
      } else {
        adaptiveFrictionRight = 2.0;
        FRICTION_RIGHT = 2.0;
      }
    }
    float voltage = (error < 0) ? -adaptiveFrictionVoltage : adaptiveFrictionVoltage;
    setMotor(voltage);
    return;
  }
  
  float dt = CONTROL_PERIOD / 1000.0;
  
  long absError = abs(error);
  if (absError > 1000) {
    KP_active = KP * 2.8;
    KI_active = 0;
    KD_active = KD * 0.6;
    errorIntegral = 0;
  } else if (absError > 500) {
    KP_active = KP * 2.4;
    KI_active = 0;
    KD_active = KD * 0.6;
    errorIntegral = 0;
  } else if (absError > 300) {
    KP_active = KP * 2.0;
    KI_active = 0;
    KD_active = KD * 0.6;
    errorIntegral = 0;
  } else if (absError > 100) {
    KP_active = KP * 1.6;
    KI_active = KI * 0.5;
    KD_active = KD * 0.9;
    errorIntegral *= 0.95;
  } else if (absError > 50) {
    KP_active = KP * 1.2;
    KI_active = KI * 0.6;
    KD_active = KD * 1.0;
    errorIntegral += error * dt * 0.7;
  } else if (absError > 10) {
    KP_active = KP * 0.5;
    KI_active = KI * 0.2;
    KD_active = KD * 1.0;
    errorIntegral += error * dt * 0.2;
  } else if (absError > 3) {
    KP_active = KP * 0.15;
    KI_active = KI * 0.05;
    KD_active = KD * 1.3;
    errorIntegral += error * dt * 0.05;
  } else {
    KP_active = KP * 0.02;
    KI_active = 0;
    KD_active = KD * 1.5;
    errorIntegral *= 0.3;
  }
  
  errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  
  if ((error != 0) && (error * lastError < 0)) {
    errorIntegral *= 0.3;
  }
  
  static float filteredDerivative = 0.0;
  float rawDerivative = (error - lastError) / dt;
  filteredDerivative = 0.7 * filteredDerivative + 0.3 * rawDerivative;
  float errorDerivative = filteredDerivative;
  
  float velocityDerivative = -motorVelocity;
  float pidVoltage = 0;
  
  if (absError > 2) {
    if (absError < 50) {
      errorDerivative = 0.3 * errorDerivative + 0.7 * velocityDerivative;
    } else if (absError < 200) {
      errorDerivative = 0.6 * errorDerivative + 0.4 * velocityDerivative;
    }
    
    pidVoltage = (KP_active * error) +
                 (KI_active * errorIntegral) +
                 (KD_active * errorDerivative);
    
    float momentumCompensation = 1.0;
    if (absError > 200) {
      momentumCompensation = 1.0;
    } else if (absError < 100 && abs(motorVelocity) > 50) {
      float velocityFactor = constrain(abs(motorVelocity) / 200.0, 0.0, 1.0);
      momentumCompensation = 1.0 - (velocityFactor * 0.35);
      momentumCompensation = max(momentumCompensation, 0.65);
    } else if (absError < 50 && abs(motorVelocity) > 30) {
      float velocityFactor = constrain(abs(motorVelocity) / 100.0, 0.0, 1.0);
      momentumCompensation = 1.0 - (velocityFactor * 0.45);
      momentumCompensation = max(momentumCompensation, 0.55);
    } else if (absError < 20 && abs(motorVelocity) > 20) {
      float velocityFactor = constrain(abs(motorVelocity) / 50.0, 0.0, 1.0);
      momentumCompensation = 1.0 - (velocityFactor * 0.5);
      momentumCompensation = max(momentumCompensation, 0.5);
    }
    pidVoltage *= momentumCompensation;
  }
  
  if (absError > 2 && absError < 30 && abs(motorVelocity) > 15) {
    float velocityDamping = 1.0 - (constrain(abs(motorVelocity) / 50.0, 0.0, 0.4));
    pidVoltage *= velocityDamping;
  }

  float frictionComp = 0;
  float velocityFF = 0;
  float fineAdjustmentBoost = 0;
  if (fineAdjustmentActive && abs(error) > 2 && abs(error) <= 5 && abs(motorVelocity) < 15) {
    float boostMultiplier = 0.8;
    fineAdjustmentBoost = error * boostMultiplier;
    fineAdjustmentBoost = constrain(fineAdjustmentBoost, -1.5, 1.5);
  }

  float totalVoltage = pidVoltage + frictionComp + velocityFF + fineAdjustmentBoost;

  bool atLeftLimit = leftPressed() && currentPosition <= 10;
  bool atRightLimit = rightPressed() && currentPosition >= (UPPER_BOUND - 10);
  bool tryingToMoveAwayFromLeft = atLeftLimit && originalError < 0;
  bool tryingToMoveAwayFromRight = atRightLimit && originalError > 0;
  
  if ((tryingToMoveAwayFromLeft || tryingToMoveAwayFromRight) && abs(originalError) > 2) {
    float minVoltageToMove = 2.5;
    if (abs(totalVoltage) < minVoltageToMove) {
      totalVoltage = (originalError < 0) ? -minVoltageToMove : minVoltageToMove;
    }
  }
  
  float voltageLimit = 9.0;
  if (absOriginalError > 1000) {
    voltageLimit = 8.5;
  } else if (absOriginalError > 800) {
    voltageLimit = 8.0;
  } else if (absOriginalError > 500) {
    voltageLimit = 7.0;
  } else if (absOriginalError > 300) {
    voltageLimit = 6.0;
  } else if (absOriginalError > 100) {
    voltageLimit = 5.0;
  } else if (absOriginalError > 50) {
    voltageLimit = 4.0;
  } else if (absOriginalError > 20) {
    voltageLimit = 3.0;
  } else if (absOriginalError > 10) {
    voltageLimit = 2.0;
  } else if (absOriginalError > 3) {
    voltageLimit = 1.0;
  } else {
    voltageLimit = 0.3;
  }

  totalVoltage = constrain(totalVoltage, -voltageLimit, voltageLimit);
  
  if (!atTarget && absOriginalError > 25 && abs(motorVelocity) < 5 && absOriginalError > 50 && abs(totalVoltage) < 2.0) {
    float startupVoltage = 2.0;
    if (absOriginalError > 200) {
      startupVoltage = 2.5;
    }
    totalVoltage = (originalError < 0) ? -startupVoltage : startupVoltage;
  }

  if (currentState == MOVE_TO_TARGET && autoMode) {
    bool movingToLane1 = (activeTargetIndex == 0 && desiredPosition == TARGET_1_POSITION);
    bool movingToLane4 = (activeTargetIndex == 3 && desiredPosition == TARGET_4_POSITION);
    
    // Left limit protection (for lane 1)
    if (error > 0) {
      if (movingToLane1) {
        if (currentPosition > -150) {
          float distanceFromLimit = currentPosition + 150;  // Distance from -150 to 0
          float proximityFactor = distanceFromLimit / 150.0;
          proximityFactor = constrain(proximityFactor, 0.0, 1.0);
          // More aggressive reduction: 0.2 to 0.6 (instead of 0.3 to 0.7)
          float limitProtection = 0.2 + (proximityFactor * 0.4);
          totalVoltage *= limitProtection;
        }
        if (currentPosition > -50) {
          // Very close to limit, reduce voltage even more
          float distanceFromLimit = currentPosition + 50;
          float proximityFactor = distanceFromLimit / 50.0;
          proximityFactor = constrain(proximityFactor, 0.0, 1.0);
          float limitProtection = 0.15 + (proximityFactor * 0.25);
          totalVoltage *= limitProtection;
        }
      } else {
        // Standard protection for other lanes
        if (currentPosition > -100) {
          float proximityFactor = (currentPosition + 100) / 100.0;
          proximityFactor = constrain(proximityFactor, 0.0, 1.0);
          float limitProtection = 0.3 + (proximityFactor * 0.4);
          totalVoltage *= limitProtection;
        }
      }
      if (currentPosition > 0) {
        stopMotor();
        return;
      }
    }
    
    // Right limit protection (for lane 4)
    if (error < 0) {
      if (movingToLane4) {
        if (currentPosition < (UPPER_BOUND + 150)) {
          float distanceFromLimit = currentPosition - (UPPER_BOUND + 150);
          float proximityFactor = (distanceFromLimit + 150) / 150.0;
          proximityFactor = constrain(proximityFactor, 0.0, 1.0);
          // More aggressive reduction: 0.2 to 0.6 (instead of 0.3 to 0.7)
          float limitProtection = 0.2 + (proximityFactor * 0.4);
          totalVoltage *= limitProtection;
        }
        if (currentPosition < (UPPER_BOUND + 50)) {
          // Very close to limit, reduce voltage even more
          float distanceFromLimit = currentPosition - (UPPER_BOUND + 50);
          float proximityFactor = (distanceFromLimit + 50) / 50.0;
          proximityFactor = constrain(proximityFactor, 0.0, 1.0);
          float limitProtection = 0.15 + (proximityFactor * 0.25);
          totalVoltage *= limitProtection;
        }
      } else {
        // Standard protection for other lanes
        if (currentPosition < (UPPER_BOUND + 100)) {
          float distanceFromLimit = currentPosition - UPPER_BOUND;
          float proximityFactor = (distanceFromLimit + 100) / 100.0;
          proximityFactor = constrain(proximityFactor, 0.0, 1.0);
          float limitProtection = 0.3 + (proximityFactor * 0.4);
          totalVoltage *= limitProtection;
        }
      }
    }
  }

  if ((error != 0) && (error * lastError < 0)) {
    errorIntegral *= 0.5;
  }

  unsigned long currentTime = millis();
  if (abs(originalError) > 2) {
    if (currentTime - lastStuckCheckTime >= 200) {  // Increased from 150ms
      if (abs(currentPosition - lastStuckCheckPos) < 3 &&
          abs(originalError) > 10 &&
          abs(motorVelocity) < 5) {
        stuckCounter++;
        if (stuckCounter == 1) {
          stuckStartTime = currentTime;
          voltageRamping = false;
        }
        if (stuckCounter >= 3) {
          voltageRamping = true;
          
          bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
          float baseFrictionVoltage = inLeftHalf ? adaptiveFrictionLeft : adaptiveFrictionRight;
          float minFrictionVoltage = max(baseFrictionVoltage, 2.0f);
          unsigned long stuckDuration = currentTime - stuckStartTime;
          float rampVoltage = minFrictionVoltage;
          if (stuckDuration > 1200) {
            rampVoltage = minFrictionVoltage + 1.5f;
          } else if (stuckDuration > 800) {
            rampVoltage = minFrictionVoltage + 1.0f;
          } else if (stuckDuration > 400) {
            rampVoltage = minFrictionVoltage + 0.5f;
          }
          
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

  if (!atTarget && abs(originalError) > 2 && !voltageRamping) {
    bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
    float baseFrictionVoltage = inLeftHalf ? adaptiveFrictionLeft : adaptiveFrictionRight;
    float minFrictionVoltage = max(baseFrictionVoltage, 2.0f);
    if (abs(originalError) > 100) {
      minFrictionVoltage = max(minFrictionVoltage, 2.5f);
    }
    if (abs(totalVoltage) < minFrictionVoltage) {
      float pidMagnitude = abs(totalVoltage);
      float finalVoltage = max(pidMagnitude, minFrictionVoltage);
      totalVoltage = (error < 0) ? -finalVoltage : finalVoltage;
    }
  }

  static float lastAppliedVoltage = 0.0;
  static unsigned long lastVoltageResetTime = 0;
  const float MAX_VOLTAGE_RATE = 0.6;
  if (moveStartTime > lastVoltageResetTime + 100) {
    lastAppliedVoltage = 0.0;
    lastVoltageResetTime = moveStartTime;
  }
  
  float maxChange = MAX_VOLTAGE_RATE;
  
  if (abs(lastAppliedVoltage) < 0.5 && abs(originalError) > 50) {
    maxChange = 1.2;
  } else   if (absError < 5) {
    maxChange = 0.15;
  } else if (absError < 20) {
    maxChange = 0.25;
  } else if (absError < 50) {
    maxChange = 0.35;
  } else if (absError < 100) {
    maxChange = 0.45;
  }
  
  bool directionChanged = ((error > 0 && lastError < 0) || (error < 0 && lastError > 0));
  if (directionChanged && absError > 20) {
    maxChange *= 1.5;
  }
  
  float voltageChange = totalVoltage - lastAppliedVoltage;
  if (abs(voltageChange) > maxChange) {
    totalVoltage = lastAppliedVoltage + (voltageChange > 0 ? maxChange : -maxChange);
  }
  lastAppliedVoltage = totalVoltage;

  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
    lastAppliedVoltage = 0.0;
  }

  lastError = error;
}

// MOTOR
bool isSwitchEnabled() {
  return digitalRead(ON_OFF_SWITCH_PIN) == HIGH;
}

void setMotor(float voltage) {
  static bool lastSwitchState = true;
  bool enabled = isSwitchEnabled();
  if (!enabled) {
    if (lastSwitchState != enabled) {
      Serial.println(F("Switch OFF"));
    }
    lastSwitchState = enabled;
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, 0);
    return;
  }
  if (lastSwitchState != enabled) {
    Serial.println(F("Switch ON"));
  }
  lastSwitchState = enabled;

  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwm = 0;
  }
  if (digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
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

// LIMITS
static unsigned long lastLeftLimitReset = 0;
static unsigned long lastRightLimitReset = 0;
const unsigned long LIMIT_RESET_DEBOUNCE = 500;

void checkLimitSwitches() {
  unsigned long currentTime = millis();
  
  if (digitalRead(LIMIT_LEFT) == HIGH) {
    if ((currentState == CALIBRATE || currentState == FIND_RANGE || dynamicCalibrationActive) && 
        (autoMode || rangeFindingActive)) {
      if (abs(motorVelocity) < 10 && (currentTime - lastLeftLimitReset) > LIMIT_RESET_DEBOUNCE) {
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
        lastLeftLimitReset = currentTime;
        Serial.println(F("RecalL"));
      }
    } else if ((currentState == MOVE_TO_TARGET || currentState == CHOOSE_ACTIVE_TARGET) && autoMode) {
      long currentPos = encoder.read();
      long error = desiredPosition - currentPos;
      if (error > 0 && currentPos <= 5 && motorVelocity <= 5 && 
          (currentTime - lastLeftLimitReset) > LIMIT_RESET_DEBOUNCE) {
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
        lastLeftLimitReset = currentTime;
        Serial.println(F("RecalL"));
      }
    }
  }

  if (digitalRead(LIMIT_RIGHT) == HIGH) {
    if ((currentState == CALIBRATE || currentState == FIND_RANGE || dynamicCalibrationActive) && 
        (autoMode || rangeFindingActive)) {
      if (abs(motorVelocity) < 10 && (currentTime - lastRightLimitReset) > LIMIT_RESET_DEBOUNCE) {
        stopMotor();
        long currentPos = encoder.read();
        if (abs(currentPos - UPPER_BOUND) > 10) {
          UPPER_BOUND = currentPos;
          RANGE_MIDPOINT = (LOWER_BOUND + UPPER_BOUND) / 2;
        Serial.print(F("R:"));
        Serial.println(UPPER_BOUND);
        }
        errorIntegral = 0;
        lastRightLimitReset = currentTime;
        Serial.println(F("R"));
      }
    } else if ((currentState == MOVE_TO_TARGET || currentState == CHOOSE_ACTIVE_TARGET) && autoMode) {
      long currentPos = encoder.read();
      long error = desiredPosition - currentPos;
      if (error < 0 && currentPos <= (UPPER_BOUND + 20) && motorVelocity >= -5 &&
          (currentTime - lastRightLimitReset) > LIMIT_RESET_DEBOUNCE) {
        stopMotor();
        errorIntegral = 0;
        lastRightLimitReset = currentTime;
        Serial.println(F("R"));
      }
    }
  }
}

bool leftPressed() {
  return digitalRead(LIMIT_LEFT) == HIGH;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == HIGH;
}

// HOME
bool homeToLeftLimit() {
  Serial.println(F("\nHOME"));

  if (leftPressed()) {

    long currentPos = encoder.read();
    bool inLeftHalf = (currentPos > RANGE_MIDPOINT);
    float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
    long lastPos = encoder.read();
    unsigned long holdStart = millis();
    int stableTicks = 0;
    float holdVoltage = max(frictionForPosition, CALIBRATE_MIN_VOLTAGE - 0.5);

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
    encoder.write(0);
    delay(50);
    if (encoder.read() != 0) {
      encoder.write(0);
      delay(50);
    }
    encoder.write(0);
    delay(50);
    long finalPos = encoder.read();
    if (abs(finalPos) > 2) {
      Serial.print(F("Enc!0:"));
      Serial.println(finalPos);
      encoder.write(0);
      delay(50);
    }

    Serial.println(F("OK"));
    return true;
  }

  long currentPos = encoder.read();
  bool inLeftHalf = (currentPos > RANGE_MIDPOINT);
  float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
  unsigned long startTime = millis();
  float driveVoltage = max(frictionForPosition + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
  setMotor(driveVoltage);

  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }

  if (leftPressed()) {
    long currentPos = encoder.read();
    bool inLeftHalf = (currentPos > RANGE_MIDPOINT);
    float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
    long lastPos = currentPos;
    unsigned long holdStart = millis();
    int stableTicks = 0;
    float holdVoltage = max(frictionForPosition, CALIBRATE_MIN_VOLTAGE - 0.5);

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
    encoder.write(0);
    delay(50);
    if (encoder.read() != 0) {
      encoder.write(0);
      delay(50);
    }
    encoder.write(0);
    delay(50);
    long finalPos = encoder.read();
    if (abs(finalPos) > 2) {
      Serial.print(F("Enc!0:"));
      Serial.println(finalPos);
      encoder.write(0);
      delay(50);
    }

    Serial.println(F("OK"));
    return true;
  } else {
    stopMotor();
      Serial.println(F("TO"));
    return false;
  }
}

// PID tuning removed

// EEPROM
void loadCalibrationFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) {
    Serial.println(F("Def"));
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

  Serial.println(F("OK"));
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

// MANUAL
void manualCalibration() {
  Serial.println(F("\nCAL"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
  Serial.println(F("R/L/S 1-4=Save Q=Quit"));
  
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
          Serial.print(F("S:"));
          Serial.println(pos);
          break;
          
        case '1':
          stopMotor();
          TARGET_1_POSITION = pos;
          targetPositions[0] = pos;
          Serial.print(F("L1: "));
          Serial.println(TARGET_1_POSITION);
          break;
          
        case '2':
          stopMotor();
          TARGET_2_POSITION = pos;
          targetPositions[1] = pos;
          Serial.print(F("L2: "));
          Serial.println(TARGET_2_POSITION);
          break;
          
        case '3':
          stopMotor();
          TARGET_3_POSITION = pos;
          targetPositions[2] = pos;
          WAIT_POSITION = TARGET_3_POSITION;
          Serial.print(F("L3: "));
          Serial.println(TARGET_3_POSITION);
          break;
          
        case '4':
          stopMotor();
          TARGET_4_POSITION = pos;
          targetPositions[3] = pos;
          Serial.print(F("L4: "));
          Serial.println(TARGET_4_POSITION);
          break;
          
        case 'Q':
          stopMotor();
          Serial.println(F("\nOK"));
          Serial.print(F("L1:"));
          Serial.print(TARGET_1_POSITION);
          Serial.print(F(" L2:"));
          Serial.print(TARGET_2_POSITION);
          Serial.print(F(" L3:"));
          Serial.print(TARGET_3_POSITION);
          Serial.print(F(" L4:"));
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

// CMDS
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
          rangeFindingComplete = false;
          rangeFindingActive = false;
          rangeFindingState = RANGE_IDLE;
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
          fineAdjustmentCount = 0;
          lastFineAdjustmentTime = 0;
          stuckStartTime = 0;
          voltageRamping = false;
          activeTargetIndex = -1;
          previousTargetIndex = -1;
          WAIT_POS = true;
          previousZombieDistance = 1.0;
          
          for (int i = 0; i < 4; i++) {
            ProxSensors[i].hitDetected = false;
            ProxSensors[i].hitTime = 0;
          }
          
          targetHitTime = 0;

          Serial.println(F("OK"));
        } else {
          Serial.println(F("FAIL"));
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
        Serial.println(F("Stop first"));
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
  stuckCounter = 0;
  positionRetryCount = 0;
  stuckStartTime = 0;
  voltageRamping = false;
  
  long currentPos = encoder.read();
  long error = desiredPosition - currentPos;
  
      Serial.print(F("\nL"));
      Serial.print(lane);
      Serial.print(F(" "));
      Serial.print(currentPos);
      Serial.print(F("->"));
      Serial.print(desiredPosition);
      Serial.print(F(" e:"));
      Serial.println(error);
}

// DISP
void printWelcome() {
  Serial.println(F("\nPVZ"));
}

void printHelp() {
  Serial.println(F("\nC:Cal Z:Home G:Auto S:Stop"));
  Serial.println(F("1-4:Lanes P:Stat D:Sens M:Mon"));
  Serial.println(F("R:Rst L:Ld W:Sv H:Help"));
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
  
  Serial.println(F("\nSTAT"));
  Serial.print(autoMode ? F("AUTO") : F("MANUAL"));
  Serial.print(F(" | Pos:"));
  Serial.print(currentPos);
  Serial.print(F(" | Err:"));
  Serial.print((int)error);
  Serial.print(F(" | Vel:"));
  Serial.println((int)motorVelocity);
  
  Serial.print(F("Range: "));
  Serial.print(LOWER_BOUND);
  Serial.print(F(" to "));
  Serial.print(UPPER_BOUND);
  Serial.print(F(" | Target: "));
  if (activeTargetIndex >= 0) {
    Serial.println(activeTargetIndex + 1);
  } else {
    Serial.println(F("None"));
  }
  
  Serial.print(F("Zombies: "));
  for (int i = 0; i < 4; i++) {
    int pct = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(pct);
    if (ProxSensors[i].direction == FORWARD) Serial.print(F("▶"));
    else if (ProxSensors[i].direction == BACKWARD) Serial.print(F("◀"));
    else Serial.print(F("■"));
    Serial.print(F(" "));
  }
  Serial.println();
  
  Serial.print(F("PID: Kp="));
  Serial.print(KP, 4);
  Serial.print(F(" Ki="));
  Serial.print(KI, 4);
  Serial.print(F(" Kd="));
  Serial.println(KD, 4);
}

void printAllSensors() {
  Serial.println(F("\nSENS"));
  Serial.print(F("Enc:"));
  Serial.print(encoder.read());
  Serial.print(F(" | L:"));
  Serial.print(leftPressed() ? "1" : "0");
  Serial.print(F(" R:"));
  Serial.println(rightPressed() ? "1" : "0");
  
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
    int pct = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(pct);
    Serial.print(F("% "));
  }
  Serial.println();
}

void continuousMonitor() {
  Serial.println(F("\nMON"));
  
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
  Serial.println(F("\nStop"));
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
