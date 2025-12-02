// ME350 Zombie Defense v13 - HYBRID SEQUENCING
// ==============================================
// Targeting Mode: TTI with Distance fallback
// Sequencing: Uses TTI when available, falls back to distance
// Best for: Most scenarios - adaptive behavior
// 
// Key characteristics:
// - Primary: TTI (when velocity is trackable)
// - Fallback: Distance (when TTI unavailable or unreliable)
// - Combines best of both approaches
// - Same emergency thresholds as other modes

#include <Encoder.h>
#include <EEPROM.h>

//============================================
// DEBUG
//============================================
// #define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

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
const int EEPROM_MAX_CMD_VOLTAGE = 70;
const int EEPROM_LANES_BASE = 29;
const int EEPROM_PROX_RANGE_BASE = 50;

//============================================
// STATE MACHINE
//============================================
enum State { IDLE = 0, CHOOSE_TARGET = 1, MOVE_TO_TARGET = 2, DWELL_AT_TARGET = 3 };
State state = IDLE;

const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

//============================================
// TARGET POSITIONS
//============================================
long targetPositions[4] = {-109, -376, -629, -1257};
const long WAIT_POSITION = -629;
const int TARGET_BAND = 10;

//============================================
// TRAVEL TIME MATRIX
//============================================
const int travelTimeMatrix[4][4] = {
  {  0,  76, 149, 328 },
  { 76,   0,  72, 252 },
  {149,  72,   0, 179 },
  {328, 252, 179,   0 }
};

//============================================
// CACHED VALUES
//============================================
long cachedEncoderPos = 0;
unsigned long lastEncoderRead = 0;
const unsigned long ENCODER_CACHE_INTERVAL = 5;
bool laneIsCritical[4] = {false, false, false, false};
bool laneIsShort[4] = {false, true, true, false};
bool laneIsLong[4] = {true, false, false, true};
unsigned long lastCriticalUpdate = 0;
const unsigned long CRITICAL_UPDATE_INTERVAL = 10;

//============================================
// LANE PRIORITY - AGGRESSIVE
//============================================
const float LANE_PRIORITY[4] = {1.0, 1.35, 1.35, 1.15};  // L2/L3 get 35% boost

const int SHORT_LANE_TTI_BOOST = 350;  // More boost for short lanes

// Distance thresholds - EARLIER ENGAGEMENT
const float EARLY_ENGAGE_THRESHOLD_LONG = 0.03;
const float EARLY_ENGAGE_THRESHOLD_SHORT = 0.02;
const float MIN_ENGAGE_THRESHOLD = 0.98;

// Override thresholds - AGGRESSIVE
const float OVERRIDE_THRESHOLD[4] = {0.80, 0.40, 0.40, 0.80};
const float ABSOLUTE_EMERGENCY_THRESHOLD = 0.70;  // Was 0.85
const float EXTREME_EMERGENCY_THRESHOLD = 0.85;   // Was 0.95
const float SHORT_LANE_CRITICAL_DISTANCE = 0.35;  // Was 0.50
const float LONG_LANE_CRITICAL_DISTANCE = 0.70;   // Was 0.85

// Retreat/Gone thresholds - FASTER EXIT
const float ZOMBIE_GONE_DISTANCE = 0.15;
const float L2_L3_GONE_DISTANCE = 0.12;
const float L4_GONE_DISTANCE = 0.18;

//============================================
// ANTI-OSCILLATION - REDUCED (faster response)
//============================================
const unsigned long MIN_COMMITMENT_TIME = 300;
const unsigned long TARGET_SWITCH_COOLDOWN = 600;
const unsigned long OVERRIDE_COOLDOWN = 800;

float getEarlyEngageThreshold(int lane) {
  return laneIsShort[lane] ? EARLY_ENGAGE_THRESHOLD_SHORT : EARLY_ENGAGE_THRESHOLD_LONG;
}

int getDynamicTravelTime(int targetLane) {
  long currentPos = cachedEncoderPos;
  int closestLane = 0;
  long minDist = abs(currentPos - targetPositions[0]);
  for (int i = 1; i < 4; i++) {
    long dist = abs(currentPos - targetPositions[i]);
    if (dist < minDist) { minDist = dist; closestLane = i; }
  }
  if (minDist < 50) return travelTimeMatrix[closestLane][targetLane];
  long distance = abs(currentPos - targetPositions[targetLane]);
  return (distance / 3.5) + 30;
}

//============================================
// CALIBRATION
//============================================
bool calibrationActive = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 10000;
int calibrationMin[4] = {1023, 1023, 1023, 1023};
int calibrationMax[4] = {0, 0, 0, 0};
int calibrationStart[4] = {0, 0, 0, 0};
bool calibrationUpdated[4] = {false, false, false, false};

//============================================
// SENSOR CALIBRATION
//============================================
int ProxRange[4][2] = {
  {640, 80},
  {620, 100},
  {620, 90},
  {650, 135}
};

//============================================
// PROXIMITY SENSORS
//============================================
struct ProxSensor {
  float currVal, prevVal, smoothVal;
  unsigned long prevChangeTime;
  int pin, direction, forwardCount, backwardCount;
};
ProxSensor ProxSensors[4];

const float alpha = 0.75;
const float velocityAlpha = 0.70;
const int stopTimeout = 80;
const int lowerNoiseLimit = 5;
const int upperNoiseLimit = 8;
const int noiseThreshold = 225;

//============================================
// TTI TRACKING
//============================================
float zombieVelocities[4] = {0, 0, 0, 0};
float timeToImpact[4] = {99999, 99999, 99999, 99999};
unsigned long lastTTIUpdate = 0;
const unsigned long TTI_UPDATE_INTERVAL = 25;
float prevZombieDistances[4] = {1.0, 1.0, 1.0, 1.0};

const int VEL_HISTORY_SIZE = 4;
float velocityHistory[4][VEL_HISTORY_SIZE];
int velocityHistoryIndex[4] = {0, 0, 0, 0};

//============================================
// GAME STATE
//============================================
bool gameOver = false;
int activeTargetIndex = -1;
int previousTargetIndex = -1;
long desiredPosition = WAIT_POSITION;
float zombieDistances[4];
bool WAIT_POS = true;
unsigned long targetLockTime = 0;

//============================================
// COMMITMENT SYSTEM
//============================================
bool isCommitted = false;
int committedLane = -1;
unsigned long commitStartTime = 0;
unsigned long lastTargetSwitchTime = 0;
float commitStartDistance = 1.0;
unsigned long lastCommitTime = 0;
const unsigned long MIN_COMMIT_INTERVAL = 100;

//============================================
// SEQUENCING SYSTEM
//============================================
const int SEQUENCE_SIZE = 4;
int targetSequence[SEQUENCE_SIZE] = {-1, -1, -1, -1};
int sequenceIndex = 0;
bool sequenceActive = false;
bool sequenceLocked = false;

bool laneAttempted[4] = {false, false, false, false};
unsigned long laneAttemptTime[4] = {0, 0, 0, 0};
const unsigned long ATTEMPT_COOLDOWN = 400;  // Faster re-engagement

unsigned long laneStoppedTime[4] = {0, 0, 0, 0};
const unsigned long STOPPED_TIMEOUT = 2000;

//============================================
// DWELL TRACKING
//============================================
unsigned long arrivalTime = 0;
float peakZombieDistance = 0.0;
float arrivalZombieDistance = 0.0;

const unsigned long MIN_DWELL_TIME = 250;           // Faster exit
const unsigned long MAX_DWELL_TIME = 500;           // Don't overstay
const unsigned long L4_DWELL_TIME = 650;            // L4 faster too
const unsigned long BACKWARD_CONFIRM_TIME = 20;     // Detect retreat faster
unsigned long backwardStartTime = 0;
unsigned long stoppedStartTime = 0;
float stoppedStartDistance = 1.0;

int zombiesKilled = 0;
int zombiesMissed = 0;
int lastHitLane = -1;
unsigned long lastHitTime = 0;
int consecutiveSameLane = 0;

//============================================
// PID CONTROLLER
//============================================
float KP = 0.0177;
float KI = 0.0000;
float KD = 0.0015;
float FRICTION_LEFT = 0.25;
float FRICTION_RIGHT = 0.25;
float SUPPLY_VOLTAGE = 12.0;      // Actual power supply voltage
float MAX_CMD_VOLTAGE = 5.0;      // Max voltage controller can command (limits torque)

float errorIntegral = 0;
float lastError = 0;
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
bool verboseMode = false;

unsigned long lastOverrideCommit = 0;
int lastOverrideLane = -1;
unsigned long overrideCheckTime = 0;

//============================================
// FUNCTION PROTOTYPES
//============================================
void commitToTarget(int lane);
void releaseCommitment();
float calculateThreatScore_Hybrid(int lane);
float getEffectiveTTI(int lane);
bool canReachInTime(int lane);
void calculateNewSequence_Hybrid();
int getNextSequenceTarget();
bool shouldRecalculateForCriticalLane();
void resetSequence();
void advanceSequence();
void updateTTI();
void resetTTITracking();
void chooseAndCommitTargetFromSequence();

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
  Serial.println(F("ME350 Zombie Defense v13"));
  Serial.println(F("MODE: HYBRID SEQUENCING"));
  Serial.println(F("(TTI primary, Distance fallback)"));
  
  loadFromEEPROM();
  
  ProxSensors[0].pin = PROX_SENSOR_1;
  ProxSensors[1].pin = PROX_SENSOR_2;
  ProxSensors[2].pin = PROX_SENSOR_3;
  ProxSensors[3].pin = PROX_SENSOR_4;
  
  for (int i = 0; i < 4; i++) {
    ProxSensors[i].currVal = analogRead(ProxSensors[i].pin);
    ProxSensors[i].prevVal = ProxSensors[i].currVal;
    ProxSensors[i].smoothVal = ProxSensors[i].currVal;
    ProxSensors[i].prevChangeTime = millis();
    ProxSensors[i].direction = STOPPED;
    ProxSensors[i].forwardCount = 0;
    ProxSensors[i].backwardCount = 0;
    
    for (int j = 0; j < VEL_HISTORY_SIZE; j++) velocityHistory[i][j] = 0;
  }
  
  stopMotor();
  printHelp();
}

//============================================
// GET EFFECTIVE TTI
//============================================
float getEffectiveTTI(int lane) {
  if (timeToImpact[lane] >= 99999) return 99999;
  int dynamicTravel = getDynamicTravelTime(lane);
  float effectiveTTI = timeToImpact[lane] - dynamicTravel;
  if (laneIsShort[lane]) effectiveTTI -= SHORT_LANE_TTI_BOOST;
  return max(effectiveTTI, 0.0f);
}

//============================================
// CAN REACH IN TIME
//============================================
bool canReachInTime(int lane) {
  int dynamicTravel = getDynamicTravelTime(lane);
  return (timeToImpact[lane] > dynamicTravel + 50) || (timeToImpact[lane] >= 99999);
}

//============================================
// UPDATE TTI
//============================================
void updateTTI() {
  for (int i = 0; i < 4; i++) {
    float distChange = zombieDistances[i] - prevZombieDistances[i];
    float instantVel = distChange / (float)TTI_UPDATE_INTERVAL;

    velocityHistory[i][velocityHistoryIndex[i]] = instantVel;
    velocityHistoryIndex[i] = (velocityHistoryIndex[i] + 1) % VEL_HISTORY_SIZE;

    float sortedVels[VEL_HISTORY_SIZE];
    for (int j = 0; j < VEL_HISTORY_SIZE; j++) sortedVels[j] = velocityHistory[i][j];
    for (int j = 0; j < VEL_HISTORY_SIZE - 1; j++) {
      for (int k = 0; k < VEL_HISTORY_SIZE - j - 1; k++) {
        if (sortedVels[k] > sortedVels[k + 1]) {
          float temp = sortedVels[k]; sortedVels[k] = sortedVels[k + 1]; sortedVels[k + 1] = temp;
        }
      }
    }
    float medianVel = sortedVels[VEL_HISTORY_SIZE / 2];

    zombieVelocities[i] = velocityAlpha * zombieVelocities[i] + (1 - velocityAlpha) * medianVel;

    float remainingDist = 1.0 - zombieDistances[i];

    if (zombieVelocities[i] > 0.0001 && ProxSensors[i].direction == FORWARD && remainingDist > 0) {
      timeToImpact[i] = remainingDist / zombieVelocities[i];
    } else if (zombieVelocities[i] <= 0 || ProxSensors[i].direction == BACKWARD) {
      timeToImpact[i] = 99999;
    } else if (remainingDist <= 0) {
      timeToImpact[i] = 0;
    } else {
      timeToImpact[i] = remainingDist / 0.0001;
    }

    timeToImpact[i] = constrain(timeToImpact[i], 0, 99999);
    prevZombieDistances[i] = zombieDistances[i];
  }
}

void resetTTITracking() {
  for (int i = 0; i < 4; i++) {
    zombieVelocities[i] = 0;
    timeToImpact[i] = 99999;
    prevZombieDistances[i] = zombieDistances[i];
    velocityHistoryIndex[i] = 0;
    for (int j = 0; j < VEL_HISTORY_SIZE; j++) velocityHistory[i][j] = 0;
  }
  lastTTIUpdate = millis();
}

//============================================
// CALCULATE SEQUENCE - HYBRID
// Uses TTI when available, falls back to distance
//============================================
void calculateNewSequence_Hybrid() {
  for (int i = 0; i < SEQUENCE_SIZE; i++) targetSequence[i] = -1;
  sequenceIndex = 0;

  bool isRoundStart = true;
  for (int i = 0; i < 4; i++) {
    if (zombieDistances[i] > 0.05) { isRoundStart = false; break; }
  }

  int maxSequenceLen = isRoundStart ? 4 : 2;

  struct LaneInfo {
    int lane;
    float distance;
    float tti;
    float effectivePriority;  // Hybrid score (lower = more urgent)
    bool hasTTI;
    bool isTargetable;
  };
  LaneInfo lanes[4];
  int targetableCount = 0;

  for (int i = 0; i < 4; i++) {
    lanes[i].lane = i;
    lanes[i].distance = zombieDistances[i];
    lanes[i].tti = timeToImpact[i];
    lanes[i].hasTTI = (timeToImpact[i] < 99999 && timeToImpact[i] > 0);
    
    // HYBRID PRIORITY CALCULATION
    // If TTI is available and valid, use it; otherwise use distance-based estimate
    if (lanes[i].hasTTI) {
      lanes[i].effectivePriority = getEffectiveTTI(i);
      // L4 critical boost
      if (i == 3 && lanes[i].distance >= 0.70) {
        lanes[i].effectivePriority -= 300;
      }
    } else {
      // No TTI - estimate based on distance
      // Convert distance to pseudo-TTI: (1 - dist) * 5000
      // Higher distance = lower pseudo-TTI = more urgent
      lanes[i].effectivePriority = (1.0 - lanes[i].distance) * 5000;
      
      // Short lane boost (subtract to make more urgent)
      if (laneIsShort[i]) {
        lanes[i].effectivePriority -= 400;
      }
    }
    
    // Travel time penalty
    int travelTime = getDynamicTravelTime(i);
    lanes[i].effectivePriority += travelTime * 0.5;  // Add travel time
    
    // Clamp
    if (lanes[i].effectivePriority < 0) lanes[i].effectivePriority = 0;

    bool isForward = (ProxSensors[i].direction == FORWARD);
    float minThreshold = laneIsShort[i] ? 0.0 : 0.30;
    lanes[i].isTargetable = (isForward && zombieDistances[i] >= minThreshold &&
                             zombieDistances[i] < MIN_ENGAGE_THRESHOLD);
    if (lanes[i].isTargetable) targetableCount++;
  }

  // Sort by EFFECTIVE PRIORITY (LOWEST first = most urgent)
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (lanes[j].effectivePriority < lanes[i].effectivePriority) {
        LaneInfo temp = lanes[i];
        lanes[i] = lanes[j];
        lanes[j] = temp;
      }
    }
  }

  int sequenceCount = 0;

  if (isRoundStart) {
    for (int i = 0; i < 4 && sequenceCount < 4; i++) {
      targetSequence[sequenceCount++] = lanes[i].lane;
    }
  } else {
    for (int i = 0; i < 4 && sequenceCount < maxSequenceLen; i++) {
      if (lanes[i].isTargetable) {
        targetSequence[sequenceCount++] = lanes[i].lane;
      }
    }
    if (sequenceCount == 1 && maxSequenceLen == 2) {
      targetSequence[sequenceCount++] = targetSequence[0];
    }
  }

  sequenceActive = (sequenceCount > 0);
  sequenceLocked = false;
  
  DBG_PRINT(F("SEQ: "));
  for (int i = 0; i < sequenceCount; i++) {
    int ln = targetSequence[i];
    DBG_PRINT(F("L")); DBG_PRINT(ln + 1);
    DBG_PRINT(F("("));
    if (timeToImpact[ln] < 99999) {
      DBG_PRINT(F("T")); DBG_PRINT((int)getEffectiveTTI(ln));
    } else {
      DBG_PRINT(F("D")); DBG_PRINT((int)(zombieDistances[ln] * 100));
    }
    DBG_PRINT(F(") "));
  }
  DBG_PRINTLN();
}

//============================================
// CALCULATE THREAT SCORE - HYBRID
// Uses TTI when available, distance as fallback
//============================================
float calculateThreatScore_Hybrid(int lane) {
  float dist = zombieDistances[lane];
  float score = 0;

  if (ProxSensors[lane].direction == BACKWARD) return 0;
  if (ProxSensors[lane].direction == STOPPED) {
    if (laneStoppedTime[lane] > 0 && (millis() - laneStoppedTime[lane]) > STOPPED_TIMEOUT) return 0;
    if (dist < 0.80) return 0;
    return dist * 50 * LANE_PRIORITY[lane];
  }
  if (ProxSensors[lane].direction != FORWARD) return 0;

  float effectiveTTI = getEffectiveTTI(lane);
  float rawTTI = timeToImpact[lane];

  // HYBRID SCORING
  if (rawTTI < 99999 && effectiveTTI < 5000) {
    // TTI-BASED (lower TTI = higher score)
    if (effectiveTTI < 200) {
      score = 1200 + (200 - effectiveTTI) * 6;
    } else if (effectiveTTI < 500) {
      score = 700 + (500 - effectiveTTI) * 1.7;
    } else if (effectiveTTI < 1000) {
      score = 350 + (1000 - effectiveTTI) * 0.7;
    } else if (effectiveTTI < 2000) {
      score = 150 + (2000 - effectiveTTI) * 0.2;
    } else {
      score = 50 + (5000 - effectiveTTI) * 0.03;
    }
  } else {
    // DISTANCE-BASED FALLBACK (higher dist = higher score)
    if (dist > 0.90) {
      score = 1000 + (dist - 0.90) * 2500;
    } else if (dist > 0.80) {
      score = 550 + (dist - 0.80) * 4500;
    } else if (dist > 0.65) {
      score = 250 + (dist - 0.65) * 2000;
    } else if (dist > 0.40) {
      score = 80 + (dist - 0.40) * 680;
    } else {
      score = 30 + dist * 125;
    }
  }

  // Travel time penalty
  int dynamicTravel = getDynamicTravelTime(lane);
  score -= dynamicTravel * 0.15;

  // Reachability check
  if (!canReachInTime(lane) && score < 500) {
    score *= 0.5;
  }

  // Lane-specific bonuses
  if (laneIsShort[lane]) {
    if (dist > SHORT_LANE_CRITICAL_DISTANCE) {
      score += (dist - SHORT_LANE_CRITICAL_DISTANCE) * 5000;
    } else if (dist > 0.45) {
      score += (dist - 0.45) * 650;
    } else {
      score += dist * 180;
    }
  } else {
    if (dist > LONG_LANE_CRITICAL_DISTANCE) {
      score += (dist - LONG_LANE_CRITICAL_DISTANCE) * 3000;
    } else if (dist > 0.70) {
      score += (dist - 0.70) * 1200;
    }
  }

  // Short lane protection bonus
  if (laneIsShort[lane]) {
    for (int other = 0; other < 4; other++) {
      if (other == lane) continue;
      if (laneIsLong[other] && ProxSensors[other].direction == FORWARD) {
        if (!laneIsCritical[other]) {
          score += 1800;
          break;
        }
      }
    }
  }

  score *= LANE_PRIORITY[lane];
  return max(score, 0.0f);
}

//============================================
// CHECK IF SHOULD RECALCULATE FOR CRITICAL LANE
//============================================
bool shouldRecalculateForCriticalLane() {
  const float URGENT_TTI_THRESHOLD = 2000;        // 2 seconds (more aggressive)
  const float CRITICAL_DIST_THRESHOLD = 0.55;     // 55% (more aggressive)

  int currentTarget = -1;
  float currentTTI = 99999;
  float currentDist = 0;
  if (sequenceActive && sequenceIndex < SEQUENCE_SIZE) {
    currentTarget = targetSequence[sequenceIndex];
    if (currentTarget >= 0) {
      currentTTI = getEffectiveTTI(currentTarget);
      currentDist = zombieDistances[currentTarget];
    }
  }

  for (int i = 0; i < 4; i++) {
    if (i == currentTarget) continue;
    if (ProxSensors[i].direction != FORWARD) continue;

    float tti = getEffectiveTTI(i);
    float dist = zombieDistances[i];

    // Urgent if TTI very low
    if (tti < URGENT_TTI_THRESHOLD && tti < currentTTI - 200) {
      return true;
    }

    // Also trigger on distance
    if (dist >= CRITICAL_DIST_THRESHOLD && dist < MIN_ENGAGE_THRESHOLD) {
      if (dist > currentDist + 0.10) {
        return true;
      }
    }
  }
  return false;
}

//============================================
// GET NEXT SEQUENCE TARGET
//============================================
int getNextSequenceTarget() {
  if (!sequenceActive || sequenceIndex >= SEQUENCE_SIZE) {
    calculateNewSequence_Hybrid();
    if (!sequenceActive) return -1;
  }

  if (shouldRecalculateForCriticalLane()) {
    calculateNewSequence_Hybrid();
    if (!sequenceActive) return -1;
  }

  while (sequenceIndex < SEQUENCE_SIZE) {
    int lane = targetSequence[sequenceIndex];
    if (lane < 0 || lane >= 4) { sequenceIndex++; continue; }
    
    if (ProxSensors[lane].direction == BACKWARD) { sequenceIndex++; continue; }
    
    if (laneAttempted[lane] && (millis() - laneAttemptTime[lane]) < ATTEMPT_COOLDOWN) {
      sequenceIndex++;
      continue;
    }
    
    float minThreshold = laneIsShort[lane] ? 0.0 : 0.30;
    bool isValid = (ProxSensors[lane].direction == FORWARD &&
                    zombieDistances[lane] >= minThreshold &&
                    zombieDistances[lane] < MIN_ENGAGE_THRESHOLD);
    
    if (isValid) {
      sequenceIndex++;
      return lane;
    }
    sequenceIndex++;
  }

  calculateNewSequence_Hybrid();
  if (!sequenceActive) return -1;
  
  if (targetSequence[0] >= 0) {
    sequenceIndex = 1;
    return targetSequence[0];
  }
  return -1;
}

//============================================
// RESET / ADVANCE SEQUENCE
//============================================
void resetSequence() {
  sequenceActive = false;
  sequenceLocked = false;
  sequenceIndex = 0;
  for (int i = 0; i < SEQUENCE_SIZE; i++) targetSequence[i] = -1;
}

void advanceSequence() {
  sequenceIndex++;
  if (sequenceIndex >= SEQUENCE_SIZE) {
    sequenceLocked = false;
    sequenceActive = false;
    sequenceIndex = 0;
    calculateNewSequence_Hybrid();
  } else {
    sequenceLocked = false;
  }
}

//============================================
// COMMIT / RELEASE
//============================================
void commitToTarget(int lane) {
  if (lane < 0 || lane > 3) return;
  if (millis() - lastCommitTime < MIN_COMMIT_INTERVAL) return;
  
  float dist = zombieDistances[lane];
  if (ProxSensors[lane].direction == BACKWARD) return;
  
  bool isEmergency = (dist > ABSOLUTE_EMERGENCY_THRESHOLD && ProxSensors[lane].direction == FORWARD);
  
  if (!isEmergency) {
    float laneThreshold = getEarlyEngageThreshold(lane);
    if (dist < laneThreshold || dist > MIN_ENGAGE_THRESHOLD) return;
    if (ProxSensors[lane].direction != FORWARD) return;
  }
  
  isCommitted = true;
  
  if (sequenceActive && !sequenceLocked && targetSequence[0] == lane) {
    sequenceLocked = true;
  }
  
  if (committedLane >= 0 && committedLane != lane) {
    lastTargetSwitchTime = millis();
  }
  
  committedLane = lane;
  commitStartTime = millis();
  commitStartDistance = zombieDistances[lane];
  lastCommitTime = millis();
  
  previousTargetIndex = activeTargetIndex;
  activeTargetIndex = lane;
  desiredPosition = targetPositions[lane];
  targetLockTime = millis();
  WAIT_POS = false;
  
  DBG_PRINT(F(">>LOCK L")); DBG_PRINT(lane + 1);
  if (timeToImpact[lane] < 99999) {
    DBG_PRINT(F(" TTI=")); DBG_PRINT((int)getEffectiveTTI(lane));
  }
  DBG_PRINT(F(" @")); DBG_PRINT((int)(dist * 100)); DBG_PRINTLN(F("%"));
}

void releaseCommitment() {
  isCommitted = false;
  committedLane = -1;
  commitStartTime = 0;
  commitStartDistance = 1.0;
}

//============================================
// CHOOSE AND COMMIT FROM SEQUENCE
//============================================
void chooseAndCommitTargetFromSequence() {
  // Find best target using hybrid scoring
  int bestLane = -1;
  float bestScore = 0;

  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction == FORWARD) {
      float minThreshold = laneIsShort[i] ? 0.0 : 0.30;
      if (zombieDistances[i] >= minThreshold && zombieDistances[i] < MIN_ENGAGE_THRESHOLD) {
        bool recentlyAttempted = laneAttempted[i] && (millis() - laneAttemptTime[i]) < ATTEMPT_COOLDOWN;
        if (!recentlyAttempted) {
          float score = calculateThreatScore_Hybrid(i);
          if (score > bestScore) {
            bestScore = score;
            bestLane = i;
          }
        }
      }
    }
  }

  if (bestLane >= 0) {
    calculateNewSequence_Hybrid();
    commitToTarget(bestLane);
    if (isCommitted) { state = MOVE_TO_TARGET; return; }
  }

  int nextLane = getNextSequenceTarget();
  if (nextLane >= 0) {
    commitToTarget(nextLane);
    if (isCommitted) state = MOVE_TO_TARGET;
  } else {
    desiredPosition = WAIT_POSITION;
    WAIT_POS = true;
    activeTargetIndex = -1;
    releaseCommitment();
    sequenceActive = false;
  }
}

//============================================
// MAIN LOOP
//============================================
void loop() {
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();
  
  processSerialCommands();
  
  unsigned long now = millis();
  if (now - lastEncoderRead >= ENCODER_CACHE_INTERVAL) {
    cachedEncoderPos = encoder.read();
    lastEncoderRead = now;
  }
  
  computeVelocity();
  updateSensors();
  
  if (now - lastCriticalUpdate >= CRITICAL_UPDATE_INTERVAL) {
    for (int i = 0; i < 4; i++) {
      if (laneIsShort[i]) {
        laneIsCritical[i] = (zombieDistances[i] > SHORT_LANE_CRITICAL_DISTANCE);
      } else {
        laneIsCritical[i] = (zombieDistances[i] > LONG_LANE_CRITICAL_DISTANCE);
      }
    }
    lastCriticalUpdate = now;
  }
  
  if (millis() - lastTTIUpdate >= TTI_UPDATE_INTERVAL) {
    updateTTI();
    lastTTIUpdate = millis();
  }
  
  if (calibrationActive) updateCalibration();
  
  //============================================
  // STATE MACHINE
  //============================================
  if (calibrationActive) {
    // Skip during calibration
  } else if (autoMode && systemEnabled && !gameOver) {
    
    //============================================
    // ABSOLUTE EMERGENCY OVERRIDE
    //============================================
    int absoluteEmergencyLane = -1;
    float highestEmergencyDist = 0;
    bool isDwelling = (state == DWELL_AT_TARGET);
    // AGGRESSIVE: Allow emergency override even during dwell if extreme emergency
    bool canEmergencyOverride = !isDwelling;
    
    if (isDwelling && committedLane >= 0 && ProxSensors[committedLane].direction == BACKWARD) {
      canEmergencyOverride = true;
    }
    // Also allow override during dwell if another lane is at extreme threshold
    if (isDwelling) {
      for (int i = 0; i < 4; i++) {
        if (i != committedLane && ProxSensors[i].direction == FORWARD && zombieDistances[i] > EXTREME_EMERGENCY_THRESHOLD) {
          canEmergencyOverride = true;
          break;
        }
      }
    }

    if (canEmergencyOverride) {
      for (int i = 0; i < 4; i++) {
        if (i == committedLane) continue;
        if (ProxSensors[i].direction != FORWARD) continue;
        float dist = zombieDistances[i];
        if (dist >= ABSOLUTE_EMERGENCY_THRESHOLD && dist < 0.99 && dist > highestEmergencyDist) {
          absoluteEmergencyLane = i;
          highestEmergencyDist = dist;
        }
      }

      if (absoluteEmergencyLane >= 0) {
        float currentDist = (committedLane >= 0) ? zombieDistances[committedLane] : 0;
        bool currentIsForward = (committedLane >= 0) ? (ProxSensors[committedLane].direction == FORWARD) : false;
        
        if (committedLane < 0 || !currentIsForward || highestEmergencyDist > currentDist + 0.05) {
          Serial.print(F("!!!EMERGENCY L")); Serial.print(absoluteEmergencyLane + 1);
          Serial.print(F(" @")); Serial.print((int)(highestEmergencyDist * 100)); Serial.println(F("%"));
          
          releaseCommitment();
          resetSequence();
          commitToTarget(absoluteEmergencyLane);
          desiredPosition = targetPositions[absoluteEmergencyLane];
          state = MOVE_TO_TARGET;
          lastOverrideCommit = millis();
          lastOverrideLane = absoluteEmergencyLane;
        }
      }
    }

    //============================================
    // NORMAL OVERRIDE CHECK (Hybrid: TTI + Distance)
    //============================================
    int overrideLane = -1;
    float bestOverrideScore = 0;
    unsigned long timeSinceOverride = millis() - lastOverrideCommit;
    
    bool canCheckOverride = false;
    if (timeSinceOverride < OVERRIDE_COOLDOWN) {
      canCheckOverride = false;
    } else if (isDwelling) {
      canCheckOverride = (millis() - overrideCheckTime >= 200);
    } else {
      canCheckOverride = (timeSinceOverride >= 1000) && (millis() - overrideCheckTime >= 150);
    }
    
    if (canCheckOverride) {
      overrideCheckTime = millis();
      
      for (int i = 0; i < 4; i++) {
        if (i == committedLane) continue;
        if (ProxSensors[i].direction == BACKWARD) continue;
        if (i == lastOverrideLane && timeSinceOverride < 5000) continue;
        
        float dist = zombieDistances[i];
        bool isAboutToImpact = (dist > EXTREME_EMERGENCY_THRESHOLD && dist < 0.99);
        
        if (ProxSensors[i].direction != FORWARD && !(ProxSensors[i].direction == STOPPED && isAboutToImpact)) continue;
        
        if (laneAttempted[i] && (millis() - laneAttemptTime[i]) < ATTEMPT_COOLDOWN) continue;
        
        bool isCritical = laneIsCritical[i];
        bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
        
        if (laneIsLong[i] && (committedLane >= 0) && laneIsShort[committedLane] && !isCritical) continue;
        
        bool shouldOverride = false;
        float tti = getEffectiveTTI(i);
        float currentTTI = (committedLane >= 0) ? getEffectiveTTI(committedLane) : 99999;
        
        // HYBRID override criteria
        if (dist > EXTREME_EMERGENCY_THRESHOLD && ProxSensors[i].direction == FORWARD) {
          shouldOverride = true;  // 95%+ = immediate
        } else if (tti < 99999 && currentTTI < 99999 && tti < currentTTI - 500 && tti < 1500) {
          shouldOverride = true;  // TTI-based: 500ms faster
        } else if (isCritical && !committedIsCritical) {
          shouldOverride = true;  // Critical vs non-critical
        } else if (laneIsShort[i] && (committedLane >= 0) && laneIsLong[committedLane] && dist > 0.50) {
          shouldOverride = true;  // Short lane past 50%
        } else if (dist > OVERRIDE_THRESHOLD[i]) {
          shouldOverride = true;  // Distance threshold
        }
        
        if (shouldOverride) {
          float score = calculateThreatScore_Hybrid(i);
          if (score > bestOverrideScore) {
            overrideLane = i;
            bestOverrideScore = score;
          }
        }
      }
    }
    
    if (overrideLane >= 0 && overrideLane != committedLane) {
      if (!(overrideLane == lastOverrideLane && timeSinceOverride < 5000)) {
        DBG_PRINT(F("!!! OVERRIDE L")); DBG_PRINT(overrideLane + 1);
        DBG_PRINT(F(" @")); DBG_PRINT((int)(zombieDistances[overrideLane] * 100)); DBG_PRINTLN(F("% !!!"));
        
        resetSequence();
        releaseCommitment();
        commitToTarget(overrideLane);
        
        if (isCommitted) {
          lastOverrideCommit = millis();
          lastOverrideLane = overrideLane;
          state = MOVE_TO_TARGET;
          return;
        }
      }
    }
    
    //============================================
    // NORMAL EXECUTION
    //============================================
    if (isCommitted && committedLane >= 0) {
      if (activeTargetIndex != committedLane && !calibrationActive) {
        activeTargetIndex = committedLane;
        desiredPosition = targetPositions[committedLane];
      }
      
      switch (state) {
        case MOVE_TO_TARGET: moveToTarget(); break;
        case DWELL_AT_TARGET: dwellAtTarget(); break;
        default: state = MOVE_TO_TARGET; break;
      }
    } else {
      switch (state) {
        case IDLE: state = CHOOSE_TARGET; break;
        case CHOOSE_TARGET: chooseAndCommitTargetFromSequence(); break;
        case MOVE_TO_TARGET: moveToTarget(); break;
        case DWELL_AT_TARGET: dwellAtTarget(); break;
      }
    }
  }
  
  if (systemEnabled && digitalRead(ON_OFF_SWITCH_PIN) == HIGH) {
    runPIDController();
  } else {
    stopMotor();
    errorIntegral = 0;
    if (autoMode) { autoMode = false; DBG_PRINTLN(F("Switch OFF")); }
  }
  
  if (autoMode && (millis() - lastPrintTime >= 350)) {
    lastPrintTime = millis();
    printStatus();
  }
}

//============================================
// MOVE TO TARGET
//============================================
void moveToTarget() {
  long currentPos = cachedEncoderPos;
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    arrivalTime = millis();
    if (activeTargetIndex >= 0) {
      peakZombieDistance = zombieDistances[activeTargetIndex];
      arrivalZombieDistance = zombieDistances[activeTargetIndex];
    }
    backwardStartTime = 0;
    
    if (WAIT_POS) state = CHOOSE_TARGET;
    else {
      state = DWELL_AT_TARGET;
      DBG_PRINT(F("~ARRIVE L")); DBG_PRINTLN(activeTargetIndex + 1);
    }
  }
}

//============================================
// DWELL AT TARGET
//============================================
void dwellAtTarget() {
  if (activeTargetIndex < 0 || activeTargetIndex > 3) {
    releaseCommitment();
    state = CHOOSE_TARGET;
    return;
  }
  
  unsigned long dwellTime = millis() - arrivalTime;
  float currentDist = zombieDistances[activeTargetIndex];
  int currentDir = ProxSensors[activeTargetIndex].direction;
  
  if (currentDist > peakZombieDistance) peakZombieDistance = currentDist;
  
  unsigned long maxDwell = (activeTargetIndex == 3) ? L4_DWELL_TIME : MAX_DWELL_TIME;
  
  // EXIT 1: BACKWARD
  if (currentDir == BACKWARD) {
    if (backwardStartTime == 0) backwardStartTime = millis();
    else if (millis() - backwardStartTime >= BACKWARD_CONFIRM_TIME) {
      int previousLane = activeTargetIndex;
      stoppedStartTime = 0;
      backwardStartTime = 0;
      peakZombieDistance = 0.0;
      arrivalZombieDistance = 0.0;
      arrivalTime = 0;
      
      if (previousLane >= 0 && previousLane <= 3) {
        zombiesKilled++;
        recordHit(previousLane);
        Serial.print(F("HIT L")); Serial.print(previousLane + 1);
        Serial.print(F(" [")); Serial.print(zombiesKilled); Serial.println(F("]"));
        laneAttempted[previousLane] = true;
        laneAttemptTime[previousLane] = millis();
      }
      
      releaseCommitment();
      advanceSequence();
      chooseAndCommitTargetFromSequence();
      if (isCommitted) state = MOVE_TO_TARGET;
      else state = CHOOSE_TARGET;
      return;
    }
  } else backwardStartTime = 0;
  
  // EXIT 2: STOPPED
  if (currentDir == STOPPED && dwellTime >= MIN_DWELL_TIME) {
    if (stoppedStartTime == 0) {
      stoppedStartTime = millis();
      stoppedStartDistance = currentDist;
    } else if (millis() - stoppedStartTime >= 300) {
      float distChange = abs(currentDist - stoppedStartDistance);
      if (distChange < 0.08) {
        if (peakZombieDistance > 0.70) {
          int previousLane = activeTargetIndex;
          stoppedStartTime = 0;
          backwardStartTime = 0;
          peakZombieDistance = 0.0;
          arrivalZombieDistance = 0.0;
          arrivalTime = 0;
          
          if (previousLane >= 0 && previousLane <= 3) {
            zombiesKilled++;
            recordHit(previousLane);
            Serial.print(F("HIT L")); Serial.print(previousLane + 1);
            Serial.print(F(" (stop) [")); Serial.print(zombiesKilled); Serial.println(F("]"));
            laneAttempted[previousLane] = true;
            laneAttemptTime[previousLane] = millis();
          }
          
          releaseCommitment();
          chooseAndCommitTargetFromSequence();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
          return;
        } else if (dwellTime >= 400) {
          int previousLane = activeTargetIndex;
          stoppedStartTime = 0;
          backwardStartTime = 0;
          peakZombieDistance = 0.0;
          arrivalZombieDistance = 0.0;
          arrivalTime = 0;
          
          if (previousLane >= 0 && previousLane <= 3) {
            laneAttempted[previousLane] = true;
            laneAttemptTime[previousLane] = millis();
          }
          DBG_PRINT(F("STOP L")); DBG_PRINTLN(previousLane + 1);
          
          releaseCommitment();
          chooseAndCommitTargetFromSequence();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
          return;
        }
      } else {
        stoppedStartTime = millis();
        stoppedStartDistance = currentDist;
      }
    }
  } else if (currentDir != STOPPED) stoppedStartTime = 0;
  
  // EXIT 3: GONE
  float goneThreshold = laneIsShort[activeTargetIndex] ? L2_L3_GONE_DISTANCE :
                        (activeTargetIndex == 3 ? L4_GONE_DISTANCE : ZOMBIE_GONE_DISTANCE);
  
  if (dwellTime >= MIN_DWELL_TIME && currentDist < goneThreshold && currentDir == BACKWARD) {
    int previousLane = activeTargetIndex;
    stoppedStartTime = 0;
    backwardStartTime = 0;
    peakZombieDistance = 0.0;
    arrivalZombieDistance = 0.0;
    arrivalTime = 0;
    
    DBG_PRINT(F("GONE L")); DBG_PRINTLN(previousLane + 1);
    laneAttempted[previousLane] = false;
    
    releaseCommitment();
    chooseAndCommitTargetFromSequence();
    if (isCommitted) state = MOVE_TO_TARGET;
    else state = CHOOSE_TARGET;
    return;
  }
  
  // EXIT 4: TIMEOUT
  if (dwellTime >= maxDwell) {
    int previousLane = activeTargetIndex;
    stoppedStartTime = 0;
    backwardStartTime = 0;
    peakZombieDistance = 0.0;
    arrivalZombieDistance = 0.0;
    arrivalTime = 0;
    
    if (previousLane >= 0 && previousLane <= 3) {
      laneAttempted[previousLane] = true;
      laneAttemptTime[previousLane] = millis();
      if (currentDir == FORWARD && currentDist < arrivalZombieDistance - 0.03) {
        zombiesMissed++;
        Serial.print(F("MISS L")); Serial.print(previousLane + 1);
        Serial.print(F(" [")); Serial.print(zombiesMissed); Serial.println(F("]"));
      } else {
        DBG_PRINT(F("TIME L")); DBG_PRINTLN(previousLane + 1);
      }
    }
    
    releaseCommitment();
    chooseAndCommitTargetFromSequence();
    if (isCommitted) state = MOVE_TO_TARGET;
    else state = CHOOSE_TARGET;
    return;
  }
}

//============================================
// SENSOR UPDATE
//============================================
void updateSensors() {
  unsigned long now = millis();

  for (int i = 0; i < 4; i++) {
    int rawVal = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal + (1.0 - alpha) * rawVal;
    ProxSensors[i].smoothVal = velocityAlpha * ProxSensors[i].smoothVal + (1.0 - velocityAlpha) * rawVal;

    const float range = (float)(ProxRange[i][0] - ProxRange[i][1]);
    float currentDistance = 0.0f;
    if (range != 0.0f) {
      float normalized = (float)(ProxRange[i][0] - rawVal) / range;
      currentDistance = constrain(normalized, 0.0f, 1.0f);
      if (laneIsShort[i]) currentDistance = constrain(currentDistance * 1.12f, 0.0f, 1.0f);
    }
    zombieDistances[i] = currentDistance;

    int sensorNoiseLimit = (ProxSensors[i].currVal >= noiseThreshold) ? upperNoiseLimit : lowerNoiseLimit;
    float change = ProxSensors[i].currVal - ProxSensors[i].prevVal;
    float changeMagnitude = abs(change);

    if (changeMagnitude < sensorNoiseLimit) {
      if (now - ProxSensors[i].prevChangeTime >= stopTimeout) {
        if (ProxSensors[i].direction != STOPPED) laneStoppedTime[i] = now;
        ProxSensors[i].direction = STOPPED;
        ProxSensors[i].forwardCount = 0;
        ProxSensors[i].backwardCount = 0;
      }
    } else if (change < 0) {
      if (ProxSensors[i].direction == STOPPED) laneStoppedTime[i] = 0;
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      if (ProxSensors[i].forwardCount >= 2) ProxSensors[i].direction = FORWARD;
    } else {
      if (ProxSensors[i].direction == STOPPED) laneStoppedTime[i] = 0;
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      if (ProxSensors[i].backwardCount >= 4) ProxSensors[i].direction = BACKWARD;
    }
    
    if (currentDistance > 0.85f && ProxSensors[i].direction == FORWARD) {
      if (ProxSensors[i].forwardCount < 3) ProxSensors[i].direction = STOPPED;
    }
  }
}

//============================================
// CALIBRATION
//============================================
void updateCalibration() {
  if (!calibrationActive) return;
  
  unsigned long elapsed = millis() - calibrationStartTime;
  if (elapsed >= CALIBRATION_DURATION) {
    applyCalibration();
    calibrationActive = false;
    Serial.println(F("=== CALIBRATION COMPLETE ==="));
    return;
  }
  
  for (int i = 0; i < 4; i++) {
    int rawVal = analogRead(ProxSensors[i].pin);
    if (rawVal < calibrationMin[i]) { calibrationMin[i] = rawVal; calibrationUpdated[i] = true; }
    if (rawVal > calibrationMax[i]) { calibrationMax[i] = rawVal; calibrationUpdated[i] = true; }
  }
}

void applyCalibration() {
  Serial.println(F("Applying calibration:"));
  for (int i = 0; i < 4; i++) {
    int startVal = calibrationStart[i];
    int impactVal = calibrationMin[i];
    int range = startVal - impactVal;

    if (range > 100 && impactVal < startVal) {
      int newFar = min(startVal + 2, 1023);
      int newClose = max(impactVal - 2, 0);
      
      if (newFar > newClose && (newFar - newClose) > 50) {
        ProxRange[i][0] = newFar;
        ProxRange[i][1] = newClose;
        EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4, (int)newFar);
        EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4 + 2, (int)newClose);
        Serial.print(F("  L")); Serial.print(i + 1);
        Serial.print(F(": [")); Serial.print(newFar);
        Serial.print(F(",")); Serial.print(newClose);
        Serial.println(F("] APPLIED"));
      }
    }
  }
}

void startCalibration() {
  calibrationActive = true;
  calibrationStartTime = millis();
  desiredPosition = 0;
  systemEnabled = true;
  
  Serial.println(F("=== CALIBRATION STARTED (10s) ==="));
  for (int i = 0; i < 4; i++) {
    calibrationStart[i] = analogRead(ProxSensors[i].pin);
    calibrationMin[i] = 1023;
    calibrationMax[i] = calibrationStart[i];
    calibrationUpdated[i] = false;
    Serial.print(F("  L")); Serial.print(i + 1);
    Serial.print(F(": Initial = ")); Serial.println(calibrationStart[i]);
  }
}

//============================================
// MOTOR & PID
//============================================
void computeVelocity() {
  long currentPos = encoder.read();
  unsigned long currentTime = micros();
  if (abs(currentPos - previousMotorPosition) > 2 || (currentTime - previousVelCompTime) > 10000) {
    motorVelocity = (double)(currentPos - previousMotorPosition) * 1000000.0 / (currentTime - previousVelCompTime);
    previousMotorPosition = currentPos;
    previousVelCompTime = currentTime;
  }
}

void runPIDController() {
  // During calibration, hold at home position
  if (calibrationActive) desiredPosition = 0;
  
  // Use cached encoder position for performance
  long currentPos = cachedEncoderPos;
  float positionError = desiredPosition - currentPos;
  
  // Lane-specific settling - L4 needs tighter band
  int effectiveBand = (activeTargetIndex == 3) ? 12 : TARGET_BAND;
  int effectiveVelThreshold = (activeTargetIndex == 3) ? 50 : 80;
  
  // If within band and low velocity, stop
  if (abs(positionError) <= effectiveBand && abs(motorVelocity) < effectiveVelThreshold) {
    stopMotor();
    errorIntegral = 0;
    return;
  }
  
  // Time-based integral (proper PID)
  errorIntegral += positionError * (float)executionDuration / 1000000.0;
  errorIntegral = constrain(errorIntegral, -1000, 1000);  // Tighter limits
  
  // Velocity-based derivative (smoother)
  float velocityError = 0 - motorVelocity;
  
  float voltage = KP * positionError + KI * errorIntegral + KD * velocityError;
  
  // Friction compensation
  if (positionError < -5) voltage -= FRICTION_LEFT;
  else if (positionError > 5) voltage += FRICTION_RIGHT;
  
  // Anti-windup: back out integral if saturating
  if (abs(voltage) > MAX_CMD_VOLTAGE) {
    errorIntegral -= positionError * (float)executionDuration / 1000000.0;
    voltage = constrain(voltage, -MAX_CMD_VOLTAGE, MAX_CMD_VOLTAGE);
  }
  
  setMotorVoltage(voltage);
}

void setMotorVoltage(float voltage) {
  // First constrain to MAX_CMD_VOLTAGE (limits torque)
  voltage = constrain(voltage, -MAX_CMD_VOLTAGE, MAX_CMD_VOLTAGE);
  // Then map to PWM using actual SUPPLY_VOLTAGE
  int pwm = constrain((int)(abs(voltage) * 255.0 / SUPPLY_VOLTAGE), 0, 255);
  analogWrite(MOTOR_ENA, pwm);
  // Direction: match FinalVersionCode.ino
  if (voltage >= 0) { digitalWrite(MOTOR_IN2, HIGH); digitalWrite(MOTOR_IN3, LOW); }
  else { digitalWrite(MOTOR_IN2, LOW); digitalWrite(MOTOR_IN3, HIGH); }
}

void stopMotor() {
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
}

void homeToLeftLimit() {
  Serial.println(F("Homing..."));
  
  // Move toward left limit switch softly at 2.0V
  while (digitalRead(LIMIT_LEFT) == LOW) { 
    setMotorVoltage(2.0); 
    delay(10); 
  }
  
  // Hold against limit switch to eliminate bounce
  Serial.println(F("Holding at limit..."));
  unsigned long holdStart = millis();
  const unsigned long HOLD_TIME = 500;
  
  while (millis() - holdStart < HOLD_TIME) {
    setMotorVoltage(2.0);
    delay(10);
  }
  
  // Set encoder zero while held firmly
  encoder.write(0);
  errorIntegral = 0;
  
  // Brief hold to confirm
  setMotorVoltage(2.0);
  delay(100);
  
  stopMotor();
  delay(50);
  Serial.println(F("Homed! Pos=0"));
}

void recordHit(int lane) {
  if (lane < 0 || lane > 3) return;
  if (lane == lastHitLane) consecutiveSameLane++;
  else consecutiveSameLane = 1;
  lastHitLane = lane;
  lastHitTime = millis();
}

//============================================
// EEPROM
//============================================
void loadFromEEPROM() {
  EEPROM.get(EEPROM_KP, KP);
  EEPROM.get(EEPROM_KI, KI);
  EEPROM.get(EEPROM_KD, KD);
  EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  
  if (isnan(KP) || KP < 0 || KP > 1) KP = 0.0177;
  if (isnan(KI) || KI < 0 || KI > 1) KI = 0.0000;
  if (isnan(KD) || KD < 0 || KD > 1) KD = 0.0015;
  if (isnan(FRICTION_LEFT) || FRICTION_LEFT < 0) FRICTION_LEFT = 0.25;
  if (isnan(FRICTION_RIGHT) || FRICTION_RIGHT < 0) FRICTION_RIGHT = 0.25;
  
  EEPROM.get(EEPROM_VOLTAGE, SUPPLY_VOLTAGE);
  if (isnan(SUPPLY_VOLTAGE) || SUPPLY_VOLTAGE < 1.0 || SUPPLY_VOLTAGE > 24.0) SUPPLY_VOLTAGE = 12.0;
  
  EEPROM.get(EEPROM_MAX_CMD_VOLTAGE, MAX_CMD_VOLTAGE);
  if (isnan(MAX_CMD_VOLTAGE) || MAX_CMD_VOLTAGE < 0.5 || MAX_CMD_VOLTAGE > SUPPLY_VOLTAGE) MAX_CMD_VOLTAGE = 5.0;
  
  for (int i = 0; i < 4; i++) {
    long pos;
    EEPROM.get(EEPROM_LANES_BASE + i * 4, pos);
    if (pos != 0 && pos > -2000 && pos < 100) targetPositions[i] = pos;
  }
  
  for (int i = 0; i < 4; i++) {
    int farVal, closeVal;
    EEPROM.get(EEPROM_PROX_RANGE_BASE + i * 4, farVal);
    EEPROM.get(EEPROM_PROX_RANGE_BASE + i * 4 + 2, closeVal);
    int range = farVal - closeVal;
    if (farVal > 400 && farVal < 900 && closeVal > 50 && closeVal < 400 && range > 300) {
      ProxRange[i][0] = farVal;
      ProxRange[i][1] = closeVal;
    }
  }
}

void saveToEEPROM() {
  Serial.println(F("Saving..."));
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.put(EEPROM_VOLTAGE, SUPPLY_VOLTAGE);
  EEPROM.put(EEPROM_MAX_CMD_VOLTAGE, MAX_CMD_VOLTAGE);
  for (int i = 0; i < 4; i++) EEPROM.put(EEPROM_LANES_BASE + i * 4, targetPositions[i]);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4, (int)ProxRange[i][0]);
    EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4 + 2, (int)ProxRange[i][1]);
  }
  Serial.println(F("Saved!"));
}

//============================================
// SERIAL COMMANDS
//============================================
void processSerialCommands() {
  if (Serial.available() == 0) return;
  
  char cmd = Serial.read();
  
  // Voltage adjustment: V<value> (e.g., V12)
  if (cmd == 'V' || cmd == 'v') {
    float value = Serial.parseFloat();
    if (value > 0 && value <= 24) { SUPPLY_VOLTAGE = value; }
    Serial.print(F("V=")); Serial.println(SUPPLY_VOLTAGE, 1);
    return;
  }
  
  // Max command voltage: M<value> (e.g., M5)
  if (cmd == 'M' || cmd == 'm') {
    float value = Serial.parseFloat();
    if (value > 0 && value <= SUPPLY_VOLTAGE) { MAX_CMD_VOLTAGE = value; }
    Serial.print(F("M=")); Serial.println(MAX_CMD_VOLTAGE, 1);
    return;
  }
  
  switch (cmd) {
    case 'T': case 't': case 'Y': case 'y':
      Serial.println(F("MODE: HYBRID (TTI+Dist)"));
      resetTTITracking();
      resetSequence();
      startAutoMode();
      startCalibration();
      break;
    case 'S': case 's':
      Serial.println(F("STOP"));
      autoMode = false; systemEnabled = false; stopMotor();
      releaseCommitment();
      resetSequence();
      calibrationActive = false;
      printScore();
      break;
    case 'H': case 'h':
      Serial.println(F("HOME"));
      autoMode = false; releaseCommitment();
      calibrationActive = false;
      homeToLeftLimit();
      break;
    case '1': case '2': case '3': case '4':
      if (!calibrationActive) {
        int lane = cmd - '1';
        Serial.print(F("->L")); Serial.println(lane + 1);
        desiredPosition = targetPositions[lane];
        autoMode = false; systemEnabled = true;
        releaseCommitment();
      }
      break;
    case 'X': case 'x': printCalibrationStatus(); break;
    case 'P': case 'p': printCurrentSettings(); break;
    case 'W': case 'w': saveToEEPROM(); break;
    case 'R': case 'r': zombiesKilled = 0; zombiesMissed = 0; Serial.println(F("Reset")); break;
    case 'D': case 'd': verboseMode = !verboseMode; Serial.print(F("Verbose:")); Serial.println(verboseMode ? F("ON") : F("OFF")); break;
    case '?': printHelp(); break;
  }
}

void startAutoMode() {
  autoMode = true;
  systemEnabled = true;
  gameOver = false;
  state = CHOOSE_TARGET;
  activeTargetIndex = -1;
  previousTargetIndex = -1;
  backwardStartTime = 0;
  zombiesKilled = 0;
  zombiesMissed = 0;
  releaseCommitment();
  for (int i = 0; i < 4; i++) { laneAttempted[i] = false; laneAttemptTime[i] = 0; }
}

//============================================
// PRINT FUNCTIONS
//============================================
void printHelp() {
  Serial.println(F("\n--- COMMANDS ---"));
  Serial.println(F("T/Y - Start (Hybrid mode)"));
  Serial.println(F("S - Stop, H - Home"));
  Serial.println(F("1-4 - Manual lane"));
  Serial.println(F("X - Calibration, P/W/R/D/?"));
}

void printCurrentSettings() {
  Serial.println(F("--- SETTINGS ---"));
  Serial.print(F("PID: ")); Serial.print(KP, 4);
  Serial.print(F("/")); Serial.print(KI, 4);
  Serial.print(F("/")); Serial.println(KD, 4);
  Serial.print(F("Supply V: ")); Serial.print(SUPPLY_VOLTAGE, 1);
  Serial.print(F("  Max Cmd V: ")); Serial.println(MAX_CMD_VOLTAGE, 1);
  Serial.print(F("Lanes: "));
  for (int i = 0; i < 4; i++) { Serial.print(targetPositions[i]); Serial.print(F(" ")); }
  Serial.println();
}

void printCalibrationStatus() {
  Serial.println(F("=== CALIBRATION STATUS ==="));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("L")); Serial.print(i + 1);
    Serial.print(F(": Range[")); Serial.print(ProxRange[i][0]);
    Serial.print(F(",")); Serial.print(ProxRange[i][1]);
    Serial.print(F("] Dist=")); Serial.print((int)(zombieDistances[i] * 100));
    Serial.print(F("% TTI="));
    if (timeToImpact[i] < 99999) Serial.print((int)timeToImpact[i]);
    else Serial.print(F("N/A"));
    Serial.println();
  }
}

void printScore() {
  Serial.print(F("Score: ")); Serial.print(zombiesKilled);
  Serial.print(F(" hits, ")); Serial.print(zombiesMissed);
  Serial.println(F(" misses"));
}

void printStatus() {
  Serial.print(F("L:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F(" "));
    Serial.print((int)(zombieDistances[i] * 100));
    Serial.print(F("%"));
    if (ProxSensors[i].direction == FORWARD) Serial.print(F("F"));
    else if (ProxSensors[i].direction == BACKWARD) Serial.print(F("B"));
    else Serial.print(F("-"));
  }
  if (committedLane >= 0) {
    Serial.print(F(" ->L")); Serial.print(committedLane + 1);
  }
  Serial.print(F(" [")); Serial.print(zombiesKilled); Serial.println(F("]"));
}
