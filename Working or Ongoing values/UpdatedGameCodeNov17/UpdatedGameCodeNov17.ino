// ME 350 - Plants vs Zombies

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

const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

// TARGET POSITIONS
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

// Range finding state variables
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

// PROXIMITY SENSORS
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
const int RIGHTWARD_DRIFT_OFFSET = 2;
const int LEFTWARD_DRIFT_OFFSET = 2;

// MOTION CONTROL STATE
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

// RANGE FINDING
void findRange() {
  if (!rangeFindingActive) {
    rangeFindingActive = true;
    rangeFindingState = RANGE_MOVE_TO_LEFT;
    rangeFindingStartTime = millis();
    Serial.println(F("\nFINDING RANGE..."));
    long currentPos = encoder.read();
    bool inLeftHalf = (currentPos > RANGE_MIDPOINT);
    float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
    rangeFindingDriveVoltage = max(frictionForPosition + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
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
        
        bool inLeftHalf = (currentPos > RANGE_MIDPOINT);
        float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
        float holdVoltage = max(frictionForPosition, CALIBRATE_MIN_VOLTAGE - 0.5);
        setMotor(holdVoltage);
        Serial.println(F("Left limit reached"));
      } else {
        if (abs(currentPos - rangeFindingLastPosition) > 2) {
          rangeFindingLastMoveTime = currentTime;
          rangeFindingLastPosition = currentPos;
        } else if (currentTime - rangeFindingLastMoveTime > 3000) {
          Serial.println(F("Stuck, increasing voltage"));
          rangeFindingDriveVoltage = min(rangeFindingDriveVoltage + 0.5, 8.0);
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
          bool inLeftHalf = (encoder.read() > RANGE_MIDPOINT);
          float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
          rangeFindingDriveVoltage = -max(frictionForPosition + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
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
        
        bool inLeftHalf = (currentPos > RANGE_MIDPOINT);
        float frictionForPosition = inLeftHalf ? FRICTION_LEFT : FRICTION_RIGHT;
        float holdVoltageRight = -max(frictionForPosition, CALIBRATE_MIN_VOLTAGE - 0.5);
        setMotor(holdVoltageRight);
        Serial.println(F("Right limit reached"));
      } else {
        if (abs(currentPos - rangeFindingLastPosition) > 2) {
          rangeFindingLastMoveTime = currentTime;
          rangeFindingLastPosition = currentPos;
        } else if (currentTime - rangeFindingLastMoveTime > 3000) {
          Serial.println(F("Stuck, increasing voltage"));
          rangeFindingDriveVoltage = max(rangeFindingDriveVoltage - 0.5, -8.0);
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
      if (currentTime - rangeFindingHoldStart >= CALIBRATE_HOLD_TIME && rangeFindingStableTicks >= CALIBRATE_STABLE_TICKS) {
        stopMotor();
        if (currentTime - rangeFindingHoldStart >= CALIBRATE_HOLD_TIME + 200) {
          UPPER_BOUND = encoder.read();
          RANGE_MIDPOINT = (LOWER_BOUND + UPPER_BOUND) / 2;
          Serial.print(F("Right: "));
          Serial.print(UPPER_BOUND);
          Serial.print(F(", Range: "));
          Serial.println(abs(UPPER_BOUND - LOWER_BOUND));
          if (abs(UPPER_BOUND - LOWER_BOUND) < 100) {
            Serial.println(F("ERROR: Range too small. Check limit switches."));
            rangeFindingActive = false;
            rangeFindingState = RANGE_IDLE;
            return;
          }
          rangeFindingComplete = true;
          rangeFindingActive = false;
          rangeFindingState = RANGE_IDLE;
          Serial.println(F("Range complete"));
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
      
    case RANGE_IDLE:
      break;
  }
}

// SENSOR CALIBRATION
void startDynamicCalibration() {
  Serial.println(F("\nSENSOR CALIBRATION (10s)..."));
  
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
    Serial.println(F("\nSensors OK"));
  } else {
    Serial.println(F("\nSome sensors WARN"));
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
        currentState = CHOOSE_ACTIVE_TARGET;
        systemEnabled = true;
      }
      break;
    
    case FIND_RANGE:
      findRange();
      if (rangeFindingComplete) {
        if (!sensorCalibrated) {
          startDynamicCalibration();
          desiredPosition = LOWER_BOUND;
        } else {
          currentState = CHOOSE_ACTIVE_TARGET;
        }
      }
      break;
    
    case CHOOSE_ACTIVE_TARGET:
      activeTargetIndex = -1;
      closestZombieDist = 2.0;
      previousMoveStartPosition = encoder.read();
      for (int i = 0; i < 4; i++) {
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
        
        int percentToPhoto = (int)((1.0 - zombieDistances[activeTargetIndex]) * 100);
        
        Serial.print(F("Target: L"));
        Serial.print(activeTargetIndex + 1);
        Serial.print(F(" ("));
        Serial.print(percentToPhoto);
        Serial.println(F("%)"));
        
        previousTargetIndex = activeTargetIndex;
        targetHitTime = 0;
        
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        fineAdjustmentActive = false;
        fineAdjustmentCount = 0;
        lastFineAdjustmentTime = 0;
        Serial.println(F("No targets, wait pos"));
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
        Serial.println(F("Approaching right limit"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }
      
      if (activeTargetIndex >= 0 && !WAIT_POS) {
        int targetDirection = ProxSensors[activeTargetIndex].direction;
        bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
        unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
        
        if (hitDetected && hitTime > 0) {
          if (millis() - hitTime >= MIN_HIT_TIME) {
            Serial.println(F("Target HIT"));
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
            Serial.println(F("Target HIT"));
            targetHitTime = 0;
            currentState = CHOOSE_ACTIVE_TARGET;
            break;
          }
        } else {
          targetHitTime = 0;
        }
        
        if (prevDirection == FORWARD && 
            (targetDirection == BACKWARD || targetDirection == STOPPED)) {
          Serial.println(F("Target changed"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }
        if (targetDirection == BACKWARD &&
            zombieDistances[activeTargetIndex] > 0.80) {
          Serial.println(F("Target safe"));
          currentState = CHOOSE_ACTIVE_TARGET;
          break;
        }

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
      
      bool hasForwardZombie = false;
      for (int i = 0; i < 4; i++) {
        if (ProxSensors[i].direction == FORWARD) {
          hasForwardZombie = true;
          break;
        }
      }

      if (!hasForwardZombie && millis() - moveStartTime > 1000) {
        Serial.println(F("No threats"));
        currentState = CHOOSE_ACTIVE_TARGET;
        break;
      }

      long errorToCurrentTarget = desiredPosition - currentPos;
      long errorToOriginalTarget = activeTargetPosition - currentPos;
      if (activeTargetIndex >= 0 && !WAIT_POS && !fineAdjustmentActive &&
          fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
          (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
          abs(motorVelocity) < 30 &&
          abs(errorToOriginalTarget) <= 8 &&
          abs(errorToOriginalTarget) > TARGET_BAND &&
          ProxSensors[activeTargetIndex].direction == FORWARD &&
          !ProxSensors[activeTargetIndex].hitDetected &&
          zombieDistances[activeTargetIndex] < 0.30) {
        
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
      
      if (abs(errorToCurrentTarget) <= TARGET_BAND) {
        if (WAIT_POS) {
          if (hasForwardZombie) {
            Serial.println(F("New threat"));
            currentState = CHOOSE_ACTIVE_TARGET;
          }
        } else if (millis() - arrivalTime > targetActivateTime) {
          if (activeTargetIndex >= 0) {
            bool hitDetected = ProxSensors[activeTargetIndex].hitDetected;
            unsigned long hitTime = ProxSensors[activeTargetIndex].hitTime;
            
            if (hitDetected && hitTime > 0 && millis() - hitTime >= MIN_HIT_TIME) {
              Serial.println(F("Target HIT"));
              ProxSensors[activeTargetIndex].hitDetected = false;
              ProxSensors[activeTargetIndex].hitTime = 0;
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            else if (ProxSensors[activeTargetIndex].direction == BACKWARD) {
              Serial.println(F("Target retreating"));
              currentState = CHOOSE_ACTIVE_TARGET;
            }
            else if (ProxSensors[activeTargetIndex].direction == FORWARD && 
                     !fineAdjustmentActive &&
                     fineAdjustmentCount < MAX_FINE_ADJUSTMENTS &&
                     (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
                     abs(motorVelocity) < 30 &&
                     abs(errorToOriginalTarget) <= TARGET_BAND + 5 &&
                     !ProxSensors[activeTargetIndex].hitDetected) {
              
              if (abs(errorToOriginalTarget) > 0) {
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
                     (millis() - lastFineAdjustmentTime) >= MIN_FINE_ADJUSTMENT_INTERVAL &&
                     abs(motorVelocity) < 25 &&
                     abs(errorToCurrentTarget) > 1 &&
                     !ProxSensors[activeTargetIndex].hitDetected &&
                     (millis() - arrivalTime) >= 100) {
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
  
  long adjustedDesiredPosition = desiredPosition;
  if (!fineAdjustmentActive) {
    if (currentPosition > desiredPosition) {
      adjustedDesiredPosition = desiredPosition + RIGHTWARD_DRIFT_OFFSET;
    } else if (currentPosition < desiredPosition) {
      adjustedDesiredPosition = desiredPosition - LEFTWARD_DRIFT_OFFSET;
    }
  }
  
  float error = adjustedDesiredPosition - currentPosition;
  
  if (currentState == MOVE_TO_TARGET && autoMode) {
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
      float correctionVoltage = constrain(error * 0.05, -2.0, 2.0);
      setMotor(correctionVoltage);
    } else {
      stopMotor();
    }
    return;
  }
  
  float originalError = desiredPosition - currentPosition;
  bool atTarget = fineAdjustmentActive ? (abs(originalError) <= 1) : (abs(originalError) <= TARGET_BAND);
  
  if (atTarget) {
    stopMotor();
    errorIntegral = 0;
    adaptiveLearning = false;

    if (!targetReached) {
      targetReached = true;
      unsigned long settleTime = millis() - moveStartTime;
      Serial.print(F("Reached "));
      Serial.print(currentPosition);
      Serial.print(F(" ("));
      Serial.print(originalError);
      Serial.print(F(") "));
      Serial.print(settleTime / 1000.0, 1);
      Serial.println(F("s"));
    }
    return;
  }

  targetReached = false;

  if (abs(originalError) > RETRY_ERROR_THRESHOLD && abs(originalError) < 50) {
    if (millis() - moveStartTime > 300 && positionRetryCount < MAX_POSITION_RETRIES) {
      positionRetryCount++;
      Serial.print(F("Stuck: "));
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
    Serial.print(F("Learning friction ("));
    Serial.print(inLeftHalf ? F("L") : F("R"));
    Serial.println(F(")"));
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
      
      Serial.print(F("Friction: "));
      Serial.print(adaptiveFrictionVoltage, 2);
      Serial.print(F("V ("));
      Serial.print(inLeftHalf ? F("L") : F("R"));
      Serial.println(F(")"));
      
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
        Serial.println(F("Max friction, using 2.5V"));
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
      Serial.println(F("Friction timeout"));
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
  } else {
    KP_active = KP * 1.5;
    KI_active = KI * 1.3;
    KD_active = KD * 1.2;
    
    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  }
  
  float errorDerivative = (error - lastError) / dt;
  
  float pidVoltage = (KP_active * error) +
                     (KI_active * errorIntegral) +
                     (KD_active * errorDerivative);
  
  float momentumCompensation = 1.0;
  if (abs(error) > 200) {
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

  float frictionComp = 0;  // Disabled
  float velocityFF = 0;  // Disabled
  float fineAdjustmentBoost = 0;
  if (fineAdjustmentActive && abs(error) > 0 && abs(error) <= 5 && abs(motorVelocity) < 15) {
    float boostMultiplier = 1.2;
    fineAdjustmentBoost = error * boostMultiplier;
    fineAdjustmentBoost = constrain(fineAdjustmentBoost, -2.0, 2.0);
  }

  float totalVoltage = pidVoltage + frictionComp + velocityFF + fineAdjustmentBoost;

  float voltageLimit = 7.5;
  long absErr = abs(error);
  if (absErr > 1000) {
    voltageLimit = 7.5;
  } else if (absErr > 800) {
    voltageLimit = 7.0;
  } else if (absErr > 500) {
    voltageLimit = 6.5;
  } else if (absErr > 300) {
    voltageLimit = 6.0;
  } else if (absErr > 100) {
    voltageLimit = 5.0;
  } else if (absErr > 50) {
    voltageLimit = 4.5;
  } else {
    voltageLimit = 3.5;
  }

  totalVoltage = constrain(totalVoltage, -voltageLimit, voltageLimit);

  if (currentState == MOVE_TO_TARGET && autoMode) {
    if (error > 0 && currentPosition > -100) {
      float proximityFactor = (currentPosition + 100) / 100.0;
      proximityFactor = constrain(proximityFactor, 0.0, 1.0);
      float limitProtection = 0.3 + (proximityFactor * 0.4);
      totalVoltage *= limitProtection;
      if (currentPosition > 0) {
        stopMotor();
        return;
      }
    }
    
    if (error < 0 && currentPosition < (UPPER_BOUND + 100)) {
      float distanceFromLimit = currentPosition - UPPER_BOUND;
      float proximityFactor = (distanceFromLimit + 100) / 100.0;
      proximityFactor = constrain(proximityFactor, 0.0, 1.0);
      float limitProtection = 0.3 + (proximityFactor * 0.4);
      totalVoltage *= limitProtection;
    }
  }

  if ((error != 0) && (error * lastError < 0)) {
    errorIntegral *= 0.5;
  }

  unsigned long currentTime = millis();
  if (abs(originalError) > TARGET_BAND) {
    if (currentTime - lastStuckCheckTime >= 150) {
      if (abs(currentPosition - lastStuckCheckPos) < 2) {
        stuckCounter++;
        if (stuckCounter == 1) {
          stuckStartTime = currentTime;
          voltageRamping = false;
        }
        if (stuckCounter >= 2) {
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

  if (abs(originalError) > TARGET_BAND && !voltageRamping) {
    bool inLeftHalf = (currentPosition > RANGE_MIDPOINT);
    float baseFrictionVoltage = inLeftHalf ? adaptiveFrictionLeft : adaptiveFrictionRight;
    float minFrictionVoltage = max(baseFrictionVoltage, 1.5f);
    if (abs(totalVoltage) < minFrictionVoltage) {
      float pidMagnitude = abs(totalVoltage);
      float finalVoltage = max(pidMagnitude, minFrictionVoltage);
      totalVoltage = (error < 0) ? -finalVoltage : finalVoltage;
    }
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

  if (digitalRead(LIMIT_RIGHT) == HIGH && 
      currentState != CALIBRATE && 
      currentState != FIND_RANGE &&
      !dynamicCalibrationActive &&
      abs(motorVelocity) < 10) {
    stopMotor();
    long currentPos = encoder.read();
    if (abs(currentPos - UPPER_BOUND) > 10) {
      UPPER_BOUND = currentPos;
      RANGE_MIDPOINT = (LOWER_BOUND + UPPER_BOUND) / 2;
      Serial.print(F("Right limit: "));
      Serial.println(UPPER_BOUND);
    }
    errorIntegral = 0;
    Serial.println(F("Right limit hit"));
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
  Serial.println(F("\nHOMING"));

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
      Serial.print(F("Encoder not zero: "));
      Serial.println(finalPos);
      encoder.write(0);
      delay(50);
    }

    Serial.println(F("Homed"));
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
      Serial.print(F("Encoder not zero: "));
      Serial.println(finalPos);
      encoder.write(0);
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

// PID tuning functions removed - done in separate sketch

// EEPROM
void loadCalibrationFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) {
    Serial.println(F("Using defaults"));
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
  Serial.println(F("Saved"));
}

void savePIDToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  Serial.println(F("Saved"));
}

void saveFrictionToEEPROM() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  Serial.println(F("Saved"));
}

// MANUAL CALIBRATION
void manualCalibration() {
  Serial.println(F("\nMANUAL CAL"));
  
  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();
  
  Serial.println(F("R=Right L=Left S=Stop 1-4=Save Q=Quit"));
  
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
          Serial.print(F("Stop: "));
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
          Serial.println(F("\nCal complete"));
          Serial.print(F("L1:"));
          Serial.print(TARGET_1_POSITION);
          Serial.print(F(" L2:"));
          Serial.print(TARGET_2_POSITION);
          Serial.print(F(" L3:"));
          Serial.print(TARGET_3_POSITION);
          Serial.print(F(" L4:"));
          Serial.println(TARGET_4_POSITION);

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

// COMMANDS
void processCommand() {
  char cmd = Serial.read();
  while (Serial.available()) Serial.read();
  
  cmd = toupper(cmd);
  
  switch (cmd) {
    case 'G':
      if (!autoMode) {
        Serial.println(F("\nAUTO MODE START"));

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
          rangeFindingComplete = false;  // Will find range first
          rangeFindingActive = false;  // Reset range finding state
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
          fineAdjustmentCount = 0;  // Reset counter
          lastFineAdjustmentTime = 0;  // Reset timestamp
          stuckStartTime = 0;  // Reset stuck tracking
          voltageRamping = false;  // Reset voltage ramping
          
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

          Serial.println(F("Homed"));
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
        Serial.println(F("Stop auto mode first (S)"));
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
      Serial.println(F("Saved"));
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

// DISPLAY
void printWelcome() {
  Serial.println(F("\nPVZ SYSTEM"));
}

void printHelp() {
  Serial.println(F("\nCmds: C-Cal Z-Home G-Auto S-Stop"));
  Serial.println(F("1-4:Lanes P-Status D-Sensors M-Mon"));
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
  
  Serial.println(F("\nSTATUS"));
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
  Serial.println(F("\nSENSORS"));
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
  Serial.println(F("\nMONITOR (press key to stop)"));
  
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
  Serial.println(F("\nMonitor stopped"));
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
