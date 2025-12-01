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
const int EEPROM_PROX_RANGE_BASE = 50;  // Store calibrated ranges

//============================================
// STATE MACHINE
//============================================
enum State {
  IDLE = 0,
  CHOOSE_TARGET = 1,
  MOVE_TO_TARGET = 2,
  DWELL_AT_TARGET = 3
};

State state = IDLE;

const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

//============================================
// TARGETING MODE
//============================================
enum TargetMode {
  MODE_HYBRID = 3
};

TargetMode targetMode = MODE_HYBRID;

//============================================
// TARGET POSITIONS
//============================================
long targetPositions[4] = {-109, -376, -629, -1257};
const long WAIT_POSITION = -629;
const int TARGET_BAND = 10;

//============================================
// LANE CHARACTERISTICS
// Base travel times from WAIT_POSITION (L3) - used as reference
//============================================
const int baseTravelTime[4] = {350, 150, 50, 400};

// DYNAMIC TRAVEL TIME MATRIX
// travelTimeMatrix[from][to] = time in ms to travel from lane 'from' to lane 'to'
// Based on encoder positions: L1=-109, L2=-376, L3=-629, L4=-1257
// Travel speed is approximately 3.5 encoder ticks per ms
const int travelTimeMatrix[4][4] = {
  // To:    L1    L2    L3    L4      From L1 (pos -109)
  {          0,   76,  149,  328 },   // L1 to others
  // To:    L1    L2    L3    L4      From L2 (pos -376)  
  {         76,    0,   72,  252 },   // L2 to others
  // To:    L1    L2    L3    L4      From L3 (pos -629)
  {        149,   72,    0,  179 },   // L3 to others
  // To:    L1    L2    L3    L4      From L4 (pos -1257)
  {        328,  252,  179,    0 }    // L4 to others
};

//============================================
// CACHED VALUES FOR EFFICIENCY (DECLARED EARLY FOR USE IN FUNCTIONS)
//============================================
long cachedEncoderPos = 0;              // Cache encoder position (updated each loop)
unsigned long lastEncoderRead = 0;      // Track when encoder was last read
const unsigned long ENCODER_CACHE_INTERVAL = 5;  // Update encoder cache every 5ms max
bool laneIsCritical[4] = {false, false, false, false};  // Cache critical status per lane
bool laneIsShort[4] = {false, true, true, false};       // Cache lane type (L2/L3 are short)
bool laneIsLong[4] = {true, false, false, true};        // Cache lane type (L1/L4 are long)
unsigned long lastCriticalUpdate = 0;   // Track when critical status was last updated
const unsigned long CRITICAL_UPDATE_INTERVAL = 10;  // Update critical status every 10ms

// Get dynamic travel time from current position to target lane
// OPTIMIZATION: Uses cached encoder position for efficiency
int getDynamicTravelTime(int targetLane) {
  // OPTIMIZATION: Use cached encoder position instead of reading again
  long currentPos = cachedEncoderPos;
  
  // Find which lane we're closest to (or between)
  int closestLane = 0;
  long minDist = abs(currentPos - targetPositions[0]);
  
  for (int i = 1; i < 4; i++) {
    long dist = abs(currentPos - targetPositions[i]);
    if (dist < minDist) {
      minDist = dist;
      closestLane = i;
    }
  }
  
  // If we're already at or very close to a lane, use matrix
  if (minDist < 50) {
    return travelTimeMatrix[closestLane][targetLane];
  }
  
  // Otherwise, calculate based on actual encoder distance
  // Approximate speed: 3.5 ticks/ms (based on typical motor performance)
  long targetPos = targetPositions[targetLane];
  long distance = abs(currentPos - targetPos);
  int calculatedTime = distance / 3.5;
  
  // Add settling time (motor needs to stop and settle)
  calculatedTime += 30;
  
  return calculatedTime;
}

// Lane priority multipliers
// L2/L3 get BOOST because they're short lanes with less reaction time
// L4 gets smaller boost for being far (we need to commit early)
const float LANE_PRIORITY[4] = {1.0, 1.15, 1.15, 1.1};  // L2/L3 get 15% boost (reduced from 30%)

const int TTI_EMERGENCY_THRESHOLD[4] = {450, 650, 650, 450};
const int TTI_MIN_ENGAGE[4] = {500, 800, 800, 500};
const int TTI_MAX_ENGAGE[4] = {2200, 3000, 3000, 2200};

const int SHORT_LANE_BOOST = 150;
const float EARLY_ENGAGE_THRESHOLD_LONG[2] = {0.92, 0.92};
const float EARLY_ENGAGE_THRESHOLD_SHORT[2] = {0.95, 0.95};
const float MIN_ENGAGE_THRESHOLD = 0.05;

// Get lane-specific engagement threshold
float getEarlyEngageThreshold(int lane) {
  if (lane == 1 || lane == 2) {  // L2 or L3 (short lanes)
    return EARLY_ENGAGE_THRESHOLD_SHORT[lane == 1 ? 0 : 1];
  } else {  // L1 or L4 (long lanes)
    return EARLY_ENGAGE_THRESHOLD_LONG[lane == 0 ? 0 : 1];
  }
}

const float OVERRIDE_THRESHOLD[4] = {0.10, 0.15, 0.15, 0.10};
const float ABSOLUTE_OVERRIDE_DISTANCE = 0.08;
const float SHORT_LANE_CRITICAL_DISTANCE = 0.20;
const float LONG_LANE_CRITICAL_DISTANCE = 0.15;
const unsigned long MIN_COMMITMENT_TIME = 800;  // Minimum 800ms commitment before allowing overrides
const unsigned long TARGET_SWITCH_COOLDOWN = 1500;  // 1.5s cooldown after switching targets

const float RETREAT_CONFIRMED_DISTANCE = 0.35;
const float ZOMBIE_GONE_DISTANCE = 0.92;
const float L2_L3_GONE_DISTANCE = 0.95;
const float L4_GONE_DISTANCE = 0.90;

//============================================
// DYNAMIC CALIBRATION
//============================================
bool calibrationActive = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 15000;  // 15 seconds of calibration

// Track min/max readings per lane during calibration
int calibrationMin[4] = {1023, 1023, 1023, 1023};  // Far (no zombie)
int calibrationMax[4] = {0, 0, 0, 0};              // Close (zombie near)
bool calibrationUpdated[4] = {false, false, false, false};

//============================================
// SENSOR CALIBRATION
//============================================
int ProxRange[4][2] = {
  {615, 88},   // Lane 1: [far, close]
  {634, 124},  // Lane 2
  {622, 147},  // Lane 3
  {590, 80}    // Lane 4
};

//============================================
// PROXIMITY SENSORS
//============================================
struct ProxSensor {
  float currVal;
  float prevVal;
  float smoothVal;
  unsigned long prevChangeTime;
  int pin;
  int direction;
  int forwardCount;
  int backwardCount;
};

ProxSensor ProxSensors[4];

const float alpha = 0.75;           // Lowered from 0.85 - much faster response (25% new data)
const float velocityAlpha = 0.70;   // Lowered from 0.80 - faster velocity tracking  
const int stopTimeout = 80;         // Lowered from 100 - faster STOPPED detection
// Noise thresholds are derived per-sensor during updates to avoid cross-lane coupling.
const int lowerNoiseLimit = 5;      // Lowered from 6
const int upperNoiseLimit = 8;      // Lowered from 10
const int noiseThreshold = 225;

//============================================
// TTI TRACKING
//============================================
float zombieVelocities[4] = {0, 0, 0, 0};
float timeToImpact[4] = {99999, 99999, 99999, 99999};
unsigned long lastTTIUpdate = 0;
const unsigned long TTI_UPDATE_INTERVAL = 25;  // Lowered from 40ms for faster updates
float prevZombieDistances[4] = {1.0, 1.0, 1.0, 1.0};

const int VEL_HISTORY_SIZE = 4;  // Lowered from 5 for faster response
float velocityHistory[4][VEL_HISTORY_SIZE];
int velocityHistoryIndex[4] = {0, 0, 0, 0};

//============================================
// GAME STATE
//============================================
bool gameOver = false;

//============================================
// TARGET TRACKING
//============================================
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
unsigned long lastTargetSwitchTime = 0;  // Track when we last switched targets
float commitStartDistance = 1.0;

// Pending targets queue (legacy - keeping for compatibility)
int pendingQueue[4] = {-1, -1, -1, -1};
int pendingQueueSize = 0;

//============================================
// BATCH TARGETING SYSTEM
// Execute targets in planned batches of up to 3 (reduced from 4)
// Each lane appears at most ONCE per batch
// Recalculate only when batch complete or true emergency
//============================================
const int MAX_BATCH_SIZE = 3;            // Reduced from 4 - gives more time to reach distant targets
int targetBatch[4] = {-1, -1, -1, -1};   // Ordered list of lanes to hit (max 3 used, no repeats)
int batchSize = 0;                       // How many targets in current batch
int batchIndex = 0;                      // Current position in batch
unsigned long batchStartTime = 0;        // When current batch was created
bool batchActive = false;                // Is a batch currently being executed?
bool batchLocked = false;                // Is batch sequence locked? (stick to sequence)

// Anti-consecutive-lane tracking
int lastHitLane = -1;                    // Last lane we successfully hit
unsigned long lastHitTime = 0;           // When we hit it
int consecutiveSameLane = 0;             // How many times we've hit same lane consecutively
const int MAX_CONSECUTIVE_SAME = 2;      // Max times to hit same lane before forcing rotation

// ATTEMPTED LANES TRACKING - Prevent re-engaging lanes already attempted
bool laneAttempted[4] = {false, false, false, false};  // Track which lanes we've already attempted
unsigned long laneAttemptTime[4] = {0, 0, 0, 0};       // When we attempted each lane
const unsigned long ATTEMPT_COOLDOWN = 1000;           // CRITICAL FIX: Reduced from 3s to 1s - allow faster re-engagement

// STOPPED LANES TRACKING - Prevent targeting lanes that have been stopped too long
unsigned long laneStoppedTime[4] = {0, 0, 0, 0};       // When each lane became STOPPED (0 = not stopped)
const unsigned long STOPPED_TIMEOUT = 2000;            // Skip lanes that have been STOPPED for > 2 seconds

// Batch statistics for debugging
int batchesCompleted = 0;
int batchesReset = 0;

// Cooldown tracking to prevent spam and rapid retries
unsigned long lastSkipMessageTime = 0;
unsigned long lastBatchSkipTime = 0;
int lastSkippedLane = -1;
const unsigned long SKIP_MESSAGE_COOLDOWN = 2000;
const unsigned long BATCH_SKIP_COOLDOWN = 500;
unsigned long lastBatchRecalcTime = 0;
const unsigned long BATCH_RECALC_COOLDOWN = 300;

//============================================
// DWELL TRACKING
//============================================
unsigned long arrivalTime = 0;
float peakZombieDistance = 1.0;
float arrivalZombieDistance = 1.0;

const unsigned long MIN_DWELL_TIME = 250;
const unsigned long NORMAL_DWELL_TIME = 450;
const unsigned long MAX_DWELL_TIME = 850;
const unsigned long L4_DWELL_TIME = 1050;

const unsigned long BACKWARD_CONFIRM_TIME = 40;  // FASTER - reduced from 60
unsigned long backwardStartTime = 0;

// STOPPED detection timing
const unsigned long STOPPED_CONFIRM_TIME = 400;  // Exit quickly if zombie stopped
unsigned long stoppedStartTime = 0;
float stoppedStartDistance = 1.0;

int zombiesKilled = 0;
int zombiesMissed = 0;

//============================================
// PID CONTROLLER
//============================================
float KP = 0.0177;
float KI = 0.0000;
float KD = 0.0015;
float FRICTION_LEFT = 0.25;
float FRICTION_RIGHT = 0.25;
float SUPPLY_VOLTAGE = 4.5;

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

//============================================
// FUNCTION PROTOTYPES
//============================================
int getDynamicTravelTime(int targetLane);
float getEffectiveTTI(int lane);
bool canReachInTime(int lane);
float calculateThreatScore(int lane);
float getEarlyEngageThreshold(int lane);
void commitToTarget(int lane);
void releaseCommitment();
bool shouldOverride(int newLane);
void addToPendingQueue(int lane);
int getBestTarget();
int getNextFromQueue();
void updatePendingQueue();
void updateCalibration();
void applyCalibration();
void stopMotor();
void setMotorVoltage(float voltage);
void recordHit(int lane);

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
  Serial.println(F("ME350 Zombie Defense v9"));
  Serial.println(F("======================="));
  Serial.println(F("EARLY ENGAGE + AUTO-CAL"));
  
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
    
    for (int j = 0; j < VEL_HISTORY_SIZE; j++) {
      velocityHistory[i][j] = 0;
    }
  }
  
  stopMotor();
  printHelp();
  printCurrentSettings();
}

//============================================
// LOAD FROM EEPROM
//============================================
void loadFromEEPROM() {
  EEPROM.get(EEPROM_KP, KP);
  EEPROM.get(EEPROM_KI, KI);
  EEPROM.get(EEPROM_KD, KD);
  EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  
  if (isnan(KP) || KP < 0 || KP > 1) KP = 0.015;
  if (isnan(KI) || KI < 0 || KI > 1) KI = 0.003;
  if (isnan(KD) || KD < 0 || KD > 1) KD = 0.020;
  if (isnan(FRICTION_LEFT) || FRICTION_LEFT < 0) FRICTION_LEFT = 0.25;
  if (isnan(FRICTION_RIGHT) || FRICTION_RIGHT < 0) FRICTION_RIGHT = 0.25;
  
  EEPROM.get(EEPROM_VOLTAGE, SUPPLY_VOLTAGE);
  if (isnan(SUPPLY_VOLTAGE) || SUPPLY_VOLTAGE < 1.0 || SUPPLY_VOLTAGE > 24.0) {
    SUPPLY_VOLTAGE = 4.5;
  }
  
  for (int i = 0; i < 4; i++) {
    long pos;
    EEPROM.get(EEPROM_LANES_BASE + i * 4, pos);
    if (pos != 0 && pos > -2000 && pos < 100) {
      targetPositions[i] = pos;
    }
  }
  
  // Load calibrated prox ranges if valid
  for (int i = 0; i < 4; i++) {
    int farVal, closeVal;
    EEPROM.get(EEPROM_PROX_RANGE_BASE + i * 4, farVal);
    EEPROM.get(EEPROM_PROX_RANGE_BASE + i * 4 + 2, closeVal);
    if (farVal > 100 && farVal < 900 && closeVal > 50 && closeVal < 800 && farVal > closeVal) {
      ProxRange[i][0] = farVal;
      ProxRange[i][1] = closeVal;
    }
  }
}

//============================================
// SAVE TO EEPROM
//============================================
void saveToEEPROM() {
  Serial.println(F("Saving..."));
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.put(EEPROM_VOLTAGE, SUPPLY_VOLTAGE);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * 4, targetPositions[i]);
  }
  // Save calibrated prox ranges
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4, (int)ProxRange[i][0]);
    EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4 + 2, (int)ProxRange[i][1]);
  }
  Serial.println(F("Saved!"));
}

//============================================
// PRINT HELP
//============================================
void printHelp() {
  Serial.println(F("\n--- COMMANDS ---"));
  Serial.println(F("T/Y - Start"));
  Serial.println(F("S - Stop, H - Home"));
  Serial.println(F("M - Mode info"));
  Serial.println(F("1-4 - Manual lane"));
  Serial.println(F("C1-C4 - Capture pos"));
  Serial.println(F("X - Show calibration"));
  Serial.println(F("P/W/R/D/?"));
  Serial.println();
}

//============================================
// PRINT SETTINGS
//============================================
void printCurrentSettings() {
  Serial.println(F("--- SETTINGS ---"));
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
  Serial.print(F("Lanes: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(targetPositions[i]);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.println(F("Prox Ranges [far,close]:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": ["));
    Serial.print(ProxRange[i][0]);
    Serial.print(F(","));
    Serial.print(ProxRange[i][1]);
    Serial.println(F("]"));
  }
}

//============================================
// COMMIT TO TARGET
// Reject invalid targets (too far = noise, too close = missed)
// But ALLOW emergency overrides even if close
//============================================
unsigned long lastCommitTime = 0;  // Rate limit commits
const unsigned long MIN_COMMIT_INTERVAL = 100;  // Minimum 100ms between commits

void commitToTarget(int lane) {
  if (lane < 0 || lane > 3) return;
  
  // Rate limit to prevent spam
  if (millis() - lastCommitTime < MIN_COMMIT_INTERVAL) {
    return;
  }
  
  float dist = zombieDistances[lane];
  
  if (ProxSensors[lane].direction == BACKWARD) return;
  bool isEmergency = (dist < ABSOLUTE_OVERRIDE_DISTANCE && ProxSensors[lane].direction == FORWARD);
  if (!isEmergency) {
    float laneThreshold = getEarlyEngageThreshold(lane);
    if (dist > laneThreshold || dist < MIN_ENGAGE_THRESHOLD) return;
    if (ProxSensors[lane].direction != FORWARD) return;
    if (batchLocked && batchActive && committedLane >= 0) {
      bool isInBatch = false;
      for (int i = 0; i < batchSize; i++) {
        if (targetBatch[i] == lane) {
          isInBatch = true;
          break;
        }
      }
      if (!isInBatch) {
        float committedDist = zombieDistances[committedLane];
        float distanceGap = dist - committedDist;
        float velocity = abs(zombieVelocities[lane]);
        bool isFastMoving = (velocity > 0.0005);
        float effectiveTTI = getEffectiveTTI(lane);
        bool isUrgent = (effectiveTTI < 800);
        bool isCritical = laneIsCritical[lane];
        bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
        float breakThreshold = laneIsShort[lane] ? -0.20 : -0.25;
        bool shouldBreak = (distanceGap < breakThreshold) ||
                           (isCritical && !committedIsCritical) ||
                           (isFastMoving && distanceGap < -0.15 && dist < 0.50) ||
                           (isUrgent && distanceGap < -0.10 && dist < 0.40);
        if (!shouldBreak) return;
      }
    }
    bool isCritical = laneIsCritical[lane];
    
    if (!batchLocked || !batchActive || committedLane < 0 || isCritical) {
      float closestDist = 1.0;
      int closestLane = -1;
      
      for (int i = 0; i < 4; i++) {
        // Only consider forward-moving targets in valid range - use lane-specific threshold
        if (i != lane && 
            ProxSensors[i].direction == FORWARD &&
            zombieDistances[i] < getEarlyEngageThreshold(i) &&
            zombieDistances[i] > MIN_ENGAGE_THRESHOLD) {
          if (zombieDistances[i] < closestDist) {
            closestDist = zombieDistances[i];
            closestLane = i;
          }
        }
      }
      
      // If there's a closer forward-moving target, only commit to far lane if it's not much further
      // EXCEPTION: ANY lane at critical distance can override this (game-ending threat!)
      if (closestLane >= 0 && closestDist < dist && !isCritical) {
        float distanceGap = dist - closestDist;
        
        // TIGHTER CRITERIA: If the gap is large, prefer the closer target
        // Require larger gaps to prevent switching to slightly closer targets
        // For L2/L3, use smaller gap since they're shorter lanes, but still tighter
        // OPTIMIZATION: Use cached lane type
        float gapThreshold = laneIsShort[lane] ? 0.15 : 0.20;  // TIGHTER: Require 15-20% gap (was 10-12%)
        if (distanceGap > gapThreshold && closestDist < 0.50) {
          return;  // Reject - closer target exists
        }
      }
    }
  }  // End of !isEmergency block
  
  isCommitted = true;
  
  // LOCK BATCH: Once we commit to first target in a batch, lock the sequence
  if (batchActive && !batchLocked && batchSize > 0) {
    // Check if this lane is the first target in the batch
    if (targetBatch[0] == lane) {
      batchLocked = true;
    }
  }
  // Track target switch for cooldown period
  if (committedLane >= 0 && committedLane != lane) {
    lastTargetSwitchTime = millis();  // Record when we switched targets
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
  
  Serial.print(F(">>LOCK L"));
  Serial.print(lane + 1);
  Serial.print(F(" @"));
  Serial.print((int)((1.0 - zombieDistances[lane]) * 100));
  Serial.println(F("%"));
}

//============================================
// RELEASE COMMITMENT
//============================================
void releaseCommitment() {
  if (isCommitted && verboseMode) {
    Serial.println(F("~UNLOCK"));
  }
  isCommitted = false;
  committedLane = -1;
  commitStartTime = 0;
  commitStartDistance = 1.0;
}

//============================================
// SHOULD OVERRIDE CURRENT COMMITMENT?
// VERY restrictive - only override for TRUE emergencies
// Batch system handles normal cycling, overrides are RARE
//============================================
bool shouldOverride(int newLane) {
  if (!isCommitted) return true;
  if (newLane == committedLane) return false;
  
  if (ProxSensors[newLane].direction != FORWARD) return false;
  unsigned long commitmentDuration = millis() - commitStartTime;
  float newDist = zombieDistances[newLane];
  bool isExtremeEmergency = (newDist < 0.08);
  if (commitmentDuration < MIN_COMMITMENT_TIME && !isExtremeEmergency) return false;
  unsigned long timeSinceSwitch = millis() - lastTargetSwitchTime;
  if (timeSinceSwitch < TARGET_SWITCH_COOLDOWN && !isExtremeEmergency) return false;
  float committedDist = zombieDistances[committedLane];
  bool newIsLongLane = laneIsLong[newLane];
  bool committedIsShortLane = laneIsShort[committedLane];
  bool newIsCritical = laneIsCritical[newLane];
  if (newIsLongLane && committedIsShortLane && !newIsCritical) {
    if (newDist >= committedDist * 0.5) return false;
  }
  if (batchLocked && batchActive) {
    bool newIsShortLane = laneIsShort[newLane];
    bool committedIsLongLane = laneIsLong[committedLane];
    
    // MUCH STRICTER: Prevent switching when targets are at similar distances
    // Increased gap requirements to prevent oscillation
    // When both are far (80-100%), require at least 25% gap to override (increased from 20%)
    // When both are closer, require at least 20% gap (increased from 15%)
    // EXCEPTION: Short lane overriding long lane needs 15% gap (increased from 10%)
    // CRITICAL: At 95-100% distance, require even larger gap (35%) to prevent excessive bouncing (increased from 30%)
    float distanceGap = abs(newDist - committedDist);
    float minGapRequired;
    if (committedDist > 0.95 && newDist > 0.95) {
      // Both at 95-100% - require very large gap (35%) to prevent bouncing
      minGapRequired = 0.35;
    } else if (newIsShortLane && committedIsLongLane) {
      // Short lane overriding long lane - need 15% gap (increased from 10%)
      minGapRequired = 0.15;
    } else {
      // Normal case - require larger gap
      minGapRequired = (committedDist > 0.80 && newDist > 0.80) ? 0.25 : 0.20;
    }
    
    if (distanceGap < minGapRequired) {
      // Targets are too close in distance - don't override
      return false;
    }
    
    // When batch is locked, only override if:
    // 1. New target is critical AND committed is NOT critical (game-ending threat)
    // 2. OR new target is at least 20% closer AND both are critical
    // 3. OR new target is short lane (L2/L3) getting very close (< 15%) while committed is long lane
    bool newIsCritical = laneIsCritical[newLane];
    bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
    // Note: newIsShortLane and committedIsLongLane already declared above
    
    // PRIORITY 1: Critical vs non-critical = always override
    if (newIsCritical && !committedIsCritical) {
      return true;
    }
    
    // PRIORITY 2: Short lane getting very close while committed to long lane
    if (newIsShortLane && committedIsLongLane && newDist < 0.15) {
      return true;
    }
    
    // PRIORITY 3: Both critical, but new is significantly closer (at least 20% closer)
    if (newIsCritical && committedIsCritical && newDist < committedDist * 0.8) {
      return true;
    }
    
    // PRIORITY 4: Extreme emergency - new target at < 10% distance
    if (newDist < 0.10) {
      return true;
    }
    
    // Otherwise, stick to batch sequence
    return false;
  }
  
  // OPTIMIZATION: Use cached encoder position
  // Calculate how close we are to our target position
  long currentPos = cachedEncoderPos;
  int distToTarget = abs(currentPos - targetPositions[committedLane]);
  bool atTarget = (distToTarget < 50);        // Already at target
  bool nearingTarget = (distToTarget < 300);  // Within 300 ticks
  bool activelyMoving = (distToTarget > 100);  // More than 100 ticks away = actively moving
  
  // CRITICAL FIX: If we're AT the target (dwelling), allow overrides for critical threats
  // Especially allow short lanes (L2/L3) to override when they're getting close
  if (atTarget && ProxSensors[committedLane].direction == FORWARD) {
    bool newIsShortLane = laneIsShort[newLane];
    bool committedIsLongLane = laneIsLong[committedLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    bool newIsLongLane = laneIsLong[newLane];
    bool newIsCritical = laneIsCritical[newLane];
    
    // CRITICAL: NEVER allow L1/L4 to override L2/L3 when dwelling, unless L1/L4 is critical
    // L2/L3 have shorter lanes and must be protected to prevent impact
    if (newIsLongLane && committedIsShortLane && !newIsCritical) {
      // Long lane trying to override short lane when not critical - don't allow
      return false;
    }
    
    // CRITICAL FIX: Distance is normalized where 0.0 = at impact, 1.0 = far
    // So LOW distance values mean close to impact (urgent!)
    // We want to override when distance is LOW (close to impact)
    
    if (newIsShortLane && committedIsLongLane) {
      if (newDist < 0.20 || newIsCritical) return true;
    }
    if (newIsCritical) return true;
    if (newDist < 0.15) return true;
    if (newDist < 0.05) return true;
    return false;
  }
  if (activelyMoving && ProxSensors[committedLane].direction == FORWARD) {
    bool newIsCritical = laneIsCritical[newLane];
    bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
    bool newIsLongLane = laneIsLong[newLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    if (newIsLongLane && committedIsShortLane && !newIsCritical) return false;
    if (newIsCritical && !committedIsCritical) return true;
    if (newIsCritical && committedIsCritical && newDist < committedDist * 0.7) return true;
    return false;
  }
  float distanceGap = abs(newDist - committedDist);
  bool newIsShortLane = laneIsShort[newLane];
  bool committedIsLongLane = laneIsLong[committedLane];
  float minGapRequired;
  
  // Increased gap requirements to prevent oscillation
  if (committedDist > 0.95 && newDist > 0.95) {
    // Both at 95-100% - require very large gap (35%) to prevent bouncing
    minGapRequired = 0.35;
  } else if (newIsShortLane && committedIsLongLane) {
    // Short lane overriding long lane - need 15% gap (increased from 10%)
    minGapRequired = 0.15;
  } else {
    // Normal case - require larger gap
    minGapRequired = (committedDist > 0.80 && newDist > 0.80) ? 0.25 : 0.20;
  }
  
  if (distanceGap < minGapRequired && !isExtremeEmergency) {
    // Targets are too similar in distance - don't override (unless extreme emergency)
    return false;
  }
  
  // RULE 4: If we're nearing target and committed lane still needs attention,
  // only override if new lane is MUCH more critical
  if (nearingTarget && committedDist < 0.40 && 
      ProxSensors[committedLane].direction == FORWARD) {
    // CRITICAL: NEVER allow L1/L4 to override L2/L3 when nearing target, unless L1/L4 is critical
    bool newIsLongLane = laneIsLong[newLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    bool newIsCritical = laneIsCritical[newLane];
    
    if (newIsLongLane && committedIsShortLane && !newIsCritical) {
      // Long lane trying to override short lane when not critical - don't allow
      return false;
    }
    
    // TIGHTER: New must be at least 3x closer than committed to justify switch (was 2x)
    return newDist < committedDist * 0.3;
  }
  
  float threshold = OVERRIDE_THRESHOLD[newLane];
  if (newDist < threshold) {
    bool newIsLongLane = laneIsLong[newLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    bool newIsCritical = laneIsCritical[newLane];
    if (newIsLongLane && committedIsShortLane && !newIsCritical) return false;
    if (committedDist < 0.25 && ProxSensors[committedLane].direction == FORWARD) {
      return newDist < committedDist * 0.4;
    }
    return true;
  }
  return false;
}

void addToPendingQueue(int lane) {
  for (int i = 0; i < pendingQueueSize; i++) {
    if (pendingQueue[i] == lane) return;
  }
  
  if (pendingQueueSize < 4) {
    pendingQueue[pendingQueueSize++] = lane;
  }
}

int getBestTarget() {
  updatePendingQueue();
  
  int bestLane = -1;
  float bestScore = 0;
  
  // Check ALL lanes for the best forward-moving target
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction != FORWARD) continue;
    
    // CRITICAL: Skip STOPPED lanes that have been stopped for > 2 seconds (freeze protection)
    if (ProxSensors[i].direction == STOPPED && laneStoppedTime[i] > 0) {
      unsigned long stoppedDuration = millis() - laneStoppedTime[i];
      if (stoppedDuration > STOPPED_TIMEOUT) {
        continue;  // Skip this lane - it's been frozen too long
      }
    }
    
    float dist = zombieDistances[i];
    // Use lane-specific threshold - L2/L3 engage earlier
    float laneThreshold = getEarlyEngageThreshold(i);
    if (dist < MIN_ENGAGE_THRESHOLD || dist > laneThreshold) continue;
    
    float score = calculateThreatScore(i);
    if (score > bestScore) {
      bestScore = score;
      bestLane = i;
    }
  }
  
  // Remove from queue if it was queued
  if (bestLane >= 0) {
    for (int i = 0; i < pendingQueueSize; i++) {
      if (pendingQueue[i] == bestLane) {
        for (int j = i; j < pendingQueueSize - 1; j++) {
          pendingQueue[j] = pendingQueue[j + 1];
        }
        pendingQueueSize--;
        break;
      }
    }
  }
  
  return bestLane;
}

//============================================
// BATCH TARGETING SYSTEM
// Creates ordered batches of targets to hit
// More predictable than reactive targeting
//============================================

// Calculate and create a new target batch
void calculateNewBatch() {
  // CRITICAL FIX: Rate limit batch recalculation to prevent excessive recalculations
  // This prevents the system from constantly recalculating and missing targets
  unsigned long now = millis();
  if (lastBatchRecalcTime > 0 && (now - lastBatchRecalcTime) < BATCH_RECALC_COOLDOWN) {
    // Still in cooldown - don't recalculate yet
    return;
  }
  lastBatchRecalcTime = now;
  
  // Clear existing batch
  for (int i = 0; i < 4; i++) {
    targetBatch[i] = -1;
  }
  batchSize = 0;
  batchIndex = 0;
  batchStartTime = millis();
  
  //============================================
  // DETECTION: All zombies stopped and close to sensors
  // When all zombies are at the start position (stopped, close to sensors),
  // use a fixed sequence: L2 -> L3 -> L4 -> L1 (repeating)
  // CRITICAL: L2 and L3 MUST be prioritized to prevent impact
  //============================================
  int stoppedCloseCount = 0;
  int closeCount = 0;
  const float CLOSE_TO_SENSOR_THRESHOLD = 0.20;  // 20% or less = close to sensor
  
  for (int i = 0; i < 4; i++) {
    bool isStopped = (ProxSensors[i].direction == STOPPED || 
                      ProxSensors[i].direction == BACKWARD ||
                      abs(zombieVelocities[i]) < 0.0001);
    bool isClose = (zombieDistances[i] < CLOSE_TO_SENSOR_THRESHOLD);
    
    if (isClose) {
      closeCount++;
      if (isStopped) {
        stoppedCloseCount++;
      }
    }
  }
  
  // CRITICAL: If ALL lanes are close (< 20%), ALWAYS use fixed sequence
  // This ensures L2/L3 are prioritized when all zombies are at start position
  // Also use fixed sequence if 3+ are stopped and close
  // EXPANDED: Also trigger if all lanes are close (< 30%) - more aggressive detection
  bool allClose = (closeCount >= 4);
  bool allCloseExpanded = true;
  for (int i = 0; i < 4; i++) {
    if (zombieDistances[i] > 0.30) {
      allCloseExpanded = false;
      break;
    }
  }
  
  if (allClose || allCloseExpanded || stoppedCloseCount >= 3) {
    // Fixed sequence: L2 -> L3 -> L4 -> L1 (repeating)
    // L2 = index 1, L3 = index 2, L4 = index 3, L1 = index 0
    const int fixedSequence[4] = {1, 2, 3, 0};  // L2, L3, L4, L1
    
    // Build the full sequence, including all lanes that are valid
    // CRITICAL: Accept forward-moving targets too - they're starting to approach
    // CRITICAL: Skip STOPPED lanes that have been stopped for too long (> 2 seconds)
    for (int i = 0; i < 4 && batchSize < MAX_BATCH_SIZE; i++) {
      int lane = fixedSequence[i];
      
      // Skip STOPPED lanes that have been stopped for > 2 seconds (freeze protection)
      bool stoppedTooLong = false;
      if (ProxSensors[lane].direction == STOPPED && laneStoppedTime[lane] > 0) {
        unsigned long stoppedDuration = now - laneStoppedTime[lane];
        if (stoppedDuration > STOPPED_TIMEOUT) {
          stoppedTooLong = true;  // Skip this lane - it's been frozen too long
        }
      }
      
      bool laneValid = !stoppedTooLong && (zombieDistances[lane] < 0.35) && 
                       (ProxSensors[lane].direction == FORWARD || 
                        (ProxSensors[lane].direction == STOPPED && !stoppedTooLong));
      
      if (laneValid) {
        targetBatch[batchSize++] = lane;
      }
    }
    
    if (batchSize > 0) {
      batchIndex = 0;
      batchActive = true;
      batchLocked = true;
      return;
    }
  }
  
  // Structure to hold lane info for sorting
  struct LaneInfo {
    int lane;
    float effectiveTTI;
    float distance;
    bool isActive;
  };
  LaneInfo lanes[4];
  
  // Gather info on all lanes
  int activeLanes = 0;
  for (int i = 0; i < 4; i++) {
    lanes[i].lane = i;
    lanes[i].distance = zombieDistances[i];
    
    // CRITICAL: Only include forward-moving targets - exclude backward/stoppe
    // Backward-moving targets are retreating and should NEVER be in batches
    // Use lane-specific threshold - L2/L3 engage much earlier
    // IMPROVED: Exclude lanes that have already been attempted (within cooldown)
    // CRITICAL: Skip STOPPED lanes that have been stopped for too long (> 2 seconds)
    bool isForwardMoving = (ProxSensors[i].direction == FORWARD);
    float laneThreshold = getEarlyEngageThreshold(i);
    bool isInRange = (lanes[i].distance < laneThreshold &&
                      lanes[i].distance > MIN_ENGAGE_THRESHOLD);
    
    // IMPROVED: Check if lane was recently attempted - exclude if within cooldown
    bool recentlyAttempted = false;
    if (laneAttempted[i]) {
      unsigned long timeSinceAttempt = millis() - laneAttemptTime[i];
      if (timeSinceAttempt < ATTEMPT_COOLDOWN) {
        recentlyAttempted = true;
      } else {
        // Cooldown expired - reset attempted flag
        laneAttempted[i] = false;
      }
    }
    
    // CRITICAL: Skip STOPPED lanes that have been stopped for > 2 seconds (freeze protection)
    bool stoppedTooLong = false;
    if (ProxSensors[i].direction == STOPPED && laneStoppedTime[i] > 0) {
      unsigned long stoppedDuration = millis() - laneStoppedTime[i];
      if (stoppedDuration > STOPPED_TIMEOUT) {
        stoppedTooLong = true;  // Skip this lane - it's been frozen too long
      }
    }
    
    lanes[i].isActive = (isForwardMoving && isInRange && !recentlyAttempted && !stoppedTooLong);
    
    if (lanes[i].isActive) {
      lanes[i].effectiveTTI = getEffectiveTTI(i);
      
      // ANTI-CONSECUTIVE PENALTY: If this lane was just hit and we've hit it multiple times,
      // add a large penalty to force rotation to other lanes
      if (i == lastHitLane && consecutiveSameLane >= MAX_CONSECUTIVE_SAME) {
        // Only penalize if hit recently (within 1.5 seconds)
        if (millis() - lastHitTime < 1500) {
          lanes[i].effectiveTTI += 1500;  // Add 1.5 second penalty
        }
      }
      // Smaller penalty for just being the last hit lane (encourage rotation)
      else if (i == lastHitLane && millis() - lastHitTime < 800) {
        lanes[i].effectiveTTI += 300;  // Add 300ms penalty to encourage others first
      }
      
      activeLanes++;
    } else {
      lanes[i].effectiveTTI = 99999;
      // Mark as inactive if backward-moving (even if in range)
      if (!isForwardMoving) {
        lanes[i].isActive = false;
      }
    }
  }
  
  // If no active lanes, no batch needed
  if (activeLanes == 0) {
    batchActive = false;
    return;
  }
  
  // ANTI-REPETITION: If only ONE lane is active and it's the same lane we just hit,
  // don't create a batch yet - wait for other targets or cooldown
  if (activeLanes == 1 && consecutiveSameLane >= MAX_CONSECUTIVE_SAME) {
    // Find which lane is active
    int activeLane = -1;
    for (int i = 0; i < 4; i++) {
      if (lanes[i].isActive) {
        activeLane = i;
        break;
      }
    }
    // If it's the same lane we've been hitting repeatedly, skip this batch
    if (activeLane == lastHitLane && millis() - lastHitTime < 1000) {
      batchActive = false;
      
      // Rate-limited skip message - only print once per cooldown period
      unsigned long now = millis();
      bool shouldPrint = (now - lastSkipMessageTime >= SKIP_MESSAGE_COOLDOWN) || 
                         (lastSkippedLane != activeLane);
      
      
      // Record skip time for cooldown
      lastBatchSkipTime = now;
      return;
    }
  }
  
  // Sort lanes by PRIORITY: 
  // 1. CRITICAL: ANY lane at critical distance gets highest priority (game-ending threat!)
  //    - L2/L3 at 25% or less = critical (short lanes)
  //    - L1/L4 at 20% or less = critical (long lanes)
  // 2. Distance (closer = lowest distance value = highest priority)
  // 3. Effective TTI (lowest = most urgent) as tiebreaker
  // This ensures we prioritize closer zombies AND critical targets in ALL lanes
  // Simple bubble sort for 4 elements
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      bool shouldSwap = false;
      
      // OPTIMIZATION: Use cached critical status instead of recalculating
      // Check if lanes are critical based on their type
      bool iIsCritical = laneIsCritical[lanes[i].lane];
      bool jIsCritical = laneIsCritical[lanes[j].lane];
      
      // CRITICAL: Any lane at critical distance ALWAYS comes first
      if (jIsCritical && !iIsCritical) {
        shouldSwap = true;  // j is critical, i is not - swap
      } else if (!jIsCritical && iIsCritical) {
        shouldSwap = false;  // i is critical, j is not - don't swap
      } else if (iIsCritical && jIsCritical) {
        // Both critical - prioritize by distance (closer critical = higher priority)
        if (lanes[j].distance < lanes[i].distance) {
          shouldSwap = true;  // j is closer critical
        }
      } else {
        // Both critical or both not - sort by priority
        // CRITICAL: Short lanes (L2/L3) get priority over long lanes (L1/L4) when distances are similar
        bool iIsShort = laneIsShort[lanes[i].lane];
        bool jIsShort = laneIsShort[lanes[j].lane];
        bool iIsLong = laneIsLong[lanes[i].lane];
        bool jIsLong = laneIsLong[lanes[j].lane];
        
        // CRITICAL: When all targets are close (< 20%), L2/L3 get ABSOLUTE priority
        // This prevents L2/L3 from being skipped when all zombies are at start position
        bool allTargetsClose = true;
        for (int k = 0; k < 4; k++) {
          if (zombieDistances[k] > 0.20) {
            allTargetsClose = false;
            break;
          }
        }
        
        if (allTargetsClose) {
          // When all are close, L2/L3 ALWAYS come first, regardless of distance
          if (jIsShort && iIsLong) {
            shouldSwap = true;  // Short lane (j) always beats long lane (i)
          } else if (iIsShort && jIsLong) {
            shouldSwap = false;  // Keep short lane (i) first
          } else if (jIsShort && iIsShort) {
            // Both short - L2 (index 1) comes before L3 (index 2)
            if (lanes[j].lane == 1 && lanes[i].lane == 2) {
              shouldSwap = true;  // L2 before L3
            } else if (lanes[i].lane == 1 && lanes[j].lane == 2) {
              shouldSwap = false;  // Keep L2 first
            } else {
              // Both L2/L3 but different order - use distance
              if (lanes[j].distance < lanes[i].distance) {
                shouldSwap = true;
              }
            }
          } else {
            // Both long - use distance
            if (lanes[j].distance < lanes[i].distance) {
              shouldSwap = true;
            }
          }
        } else {
          // Normal case: Short lane vs long lane - short lane wins if distances are similar (within 15%)
          // CRITICAL: If long lane is NOT critical, short lane gets ABSOLUTE priority regardless of distance gap
          float distanceGap = abs(lanes[j].distance - lanes[i].distance);
          bool iIsLongCritical = (iIsLong && iIsCritical);
          bool jIsLongCritical = (jIsLong && jIsCritical);
          
          if (jIsShort && iIsLong) {
            // Short lane (j) vs long lane (i)
            if (!iIsLongCritical) {
              // Long lane not critical - ALWAYS prioritize short lane regardless of distance gap
              shouldSwap = true;
            } else if (distanceGap < 0.15) {
              // Long lane critical but distances very similar - prioritize short lane
              shouldSwap = true;
            }
          } else if (iIsShort && jIsLong) {
            // Short lane (i) vs long lane (j)
            if (!jIsLongCritical) {
              // Long lane not critical - ALWAYS keep short lane first regardless of distance gap
              shouldSwap = false;
            } else if (distanceGap < 0.15) {
              // Long lane critical but distances very similar - keep short lane first
              shouldSwap = false;
            } else {
              // Distance gap too large - use distance
              if (lanes[j].distance < lanes[i].distance) {
                shouldSwap = true;
              }
            }
          } else {
            // Primary sort: by distance (closer = higher priority)
            // Lower distance value = zombie is closer = should be targeted first
            if (lanes[j].distance < lanes[i].distance) {
              // Lane j is closer - higher priority
              shouldSwap = true;
            } else if (lanes[j].distance == lanes[i].distance) {
              // Same distance - prioritize short lanes, then use TTI as tiebreaker
              if (jIsShort && !iIsShort) {
                shouldSwap = true;  // Short lane wins at same distance
              } else if (!jIsShort && iIsShort) {
                shouldSwap = false;  // Keep short lane first
              } else if (lanes[j].effectiveTTI < lanes[i].effectiveTTI) {
                shouldSwap = true;  // Use TTI as tiebreaker
              }
            }
          }
        }
      }
      
      if (shouldSwap) {
        LaneInfo temp = lanes[i];
        lanes[i] = lanes[j];
        lanes[j] = temp;
      }
    }
  }
  
  // Build batch: Add each active lane ONCE in priority order (closest first)
  // NO DUPLICATES - each lane appears at most once per batch
  // LIMITED TO MAX_BATCH_SIZE (3) to allow time to reach all targets
  // DOUBLE-CHECK: Ensure we never add backward-moving lanes to batch
  // CRITICAL: L2/L3 must be included if they're close, even if batch is full
  bool l2Close = (zombieDistances[1] < 0.20 && ProxSensors[1].direction == FORWARD);
  bool l3Close = (zombieDistances[2] < 0.20 && ProxSensors[2].direction == FORWARD);
  
  for (int i = 0; i < 4; i++) {
    if (lanes[i].isActive && batchSize < MAX_BATCH_SIZE) {
      int lane = lanes[i].lane;
      
      // Final safety check: Never add backward-moving lanes to batch
      if (ProxSensors[lane].direction == FORWARD) {
        targetBatch[batchSize++] = lane;
      }
      // If lane became backward since we checked, skip it
    }
  }
  
  // CRITICAL: If L2 or L3 are close but weren't added (batch full), force them in
  // This ensures short lanes are never skipped when all zombies are at start position
  if (l2Close) {
    bool l2InBatch = false;
    for (int i = 0; i < batchSize; i++) {
      if (targetBatch[i] == 1) {
        l2InBatch = true;
        break;
      }
    }
    if (!l2InBatch && batchSize < MAX_BATCH_SIZE) {
      targetBatch[batchSize++] = 1;  // Force L2 into batch
    } else if (!l2InBatch && batchSize >= MAX_BATCH_SIZE) {
      // Batch is full - replace last entry with L2 if it's a long lane
      if (laneIsLong[targetBatch[batchSize - 1]]) {
        targetBatch[batchSize - 1] = 1;  // Replace last with L2
      }
    }
  }
  
  if (l3Close) {
    bool l3InBatch = false;
    for (int i = 0; i < batchSize; i++) {
      if (targetBatch[i] == 2) {
        l3InBatch = true;
        break;
      }
    }
    if (!l3InBatch && batchSize < MAX_BATCH_SIZE) {
      targetBatch[batchSize++] = 2;  // Force L3 into batch
    } else if (!l3InBatch && batchSize >= MAX_BATCH_SIZE) {
      // Batch is full - replace last entry with L3 if it's a long lane
      if (laneIsLong[targetBatch[batchSize - 1]]) {
        targetBatch[batchSize - 1] = 2;  // Replace last with L3
      }
    }
  }
  
  batchActive = (batchSize > 0);
  batchLocked = false;  // Batch not locked yet - will lock when first target is committed
  
  // Reset skip tracking on successful batch creation
  if (batchActive) {
    lastBatchSkipTime = 0;
    lastSkippedLane = -1;
  }
  
}

// Get the next target from the current batch
int getNextBatchTarget() {
  // If batch is empty or exhausted, calculate new one
  if (!batchActive || batchIndex >= batchSize) {
    // Check cooldown - don't recalculate immediately after a skip
    unsigned long now = millis();
    if (lastBatchSkipTime > 0 && (now - lastBatchSkipTime) < BATCH_SKIP_COOLDOWN) {
      // Still in cooldown period, return -1 without recalculating
      return -1;
    }
    
    calculateNewBatch();
    if (!batchActive) return -1;
    
    // Reset skip cooldown on successful batch creation
    if (batchActive) {
      lastBatchSkipTime = 0;
      lastSkippedLane = -1;
    }
  }
  
  // Find next valid target in batch
  while (batchIndex < batchSize) {
    int lane = targetBatch[batchIndex];
    
    // CRITICAL: Remove backward-moving lanes from batch - they're retreating!
    // Also validate forward direction, range, and lane validity
    // Use lane-specific threshold - L2/L3 have higher thresholds
    bool isValid = (lane >= 0 && lane < 4);
    bool isForward = (ProxSensors[lane].direction == FORWARD);
    bool isBackward = (ProxSensors[lane].direction == BACKWARD);
    bool isStopped = (ProxSensors[lane].direction == STOPPED);
    float laneThreshold = getEarlyEngageThreshold(lane);
    bool isInRange = (zombieDistances[lane] < laneThreshold &&
                      zombieDistances[lane] > MIN_ENGAGE_THRESHOLD);
    
    // CRITICAL: For L2/L3 in batch sequence, allow targeting even at high distance (85-100%)
    // Short lanes move fast, so even at 85-100% we might still catch them
    bool isShortLane = (lane == 1 || lane == 2);  // L2 or L3
    bool isAtHighDistance = (zombieDistances[lane] >= 0.85f && zombieDistances[lane] <= 1.0f);
    bool isStoppedBriefly = (isStopped && 
                             (laneStoppedTime[lane] == 0 || 
                              (millis() - laneStoppedTime[lane]) < 500));  // Only recently stopped
    
    // For short lanes at high distance, be more lenient - allow if forward or recently stopped
    bool isValidForShortLane = isShortLane && isAtHighDistance && 
                                (isForward || isStoppedBriefly) && 
                                zombieDistances[lane] > MIN_ENGAGE_THRESHOLD;
    
    // If target is backward-moving, remove it from batch entirely
    if (isValid && isBackward) {
      // Remove this lane from batch by shifting remaining elements
      for (int i = batchIndex; i < batchSize - 1; i++) {
        targetBatch[i] = targetBatch[i + 1];
      }
      batchSize--;
      // Don't increment batchIndex - check the same position again (now has different lane)
      continue;
    }
    
    // IMPROVED: Check if lane was recently attempted - skip if within cooldown
    bool recentlyAttempted = false;
    if (laneAttempted[lane]) {
      unsigned long timeSinceAttempt = millis() - laneAttemptTime[lane];
      if (timeSinceAttempt < ATTEMPT_COOLDOWN) {
        recentlyAttempted = true;
      } else {
        // Cooldown expired - reset attempted flag
        laneAttempted[lane] = false;
      }
    }
    
    // Check if this target is still valid (forward-moving, in range, not recently attempted)
    // OR if it's a short lane at high distance (special case for batch sequence)
    if (isValid && !recentlyAttempted && 
        ((isForward && isInRange) || isValidForShortLane)) {
      batchIndex++;  // Move to next for next call
      return lane;
    }
    
    // Target no longer valid (stopped, out of range, or already attempted) - skip it
    batchIndex++;
  }
  
  // Batch exhausted, calculate new one (with cooldown check)
  unsigned long now = millis();
  if (lastBatchSkipTime > 0 && (now - lastBatchSkipTime) < BATCH_SKIP_COOLDOWN) {
    return -1;
  }
  
  calculateNewBatch();
  if (!batchActive) return -1;
  
  // Reset skip cooldown on successful batch creation
  if (batchActive) {
    lastBatchSkipTime = 0;
    lastSkippedLane = -1;
  }
  
  // Return first target of new batch
  if (batchSize > 0) {
    batchIndex = 1;
    return targetBatch[0];
  }
  
  return -1;
}

// Reset the batch (called on emergency override or major change)
void resetBatch() {
  batchActive = false;
  batchLocked = false;  // Unlock batch on reset
  batchIndex = 0;
  batchSize = 0;
  batchesReset++;
  
  // Clear skip tracking on reset to allow immediate recalculation
  lastBatchSkipTime = 0;
  lastSkippedLane = -1;
}

// Mark current batch target as complete (hit registered)
// Also handles batch unlocking when sequence is complete
void advanceBatch() {
  // Check if batch sequence is complete
  if (batchIndex >= batchSize && batchLocked) {
    batchesCompleted++;
    batchLocked = false;
    batchActive = false;
  } else if (batchIndex >= batchSize) {
    batchesCompleted++;
    batchActive = false;
  }
}

// Check if any lane needs emergency attention (outside batch order)
int checkBatchEmergency() {
  for (int i = 0; i < 4; i++) {
    // Skip the lane we're currently committed to
    if (i == committedLane) continue;
    
    // Only check forward-moving targets
    if (ProxSensors[i].direction != FORWARD) continue;
    
    // Emergency threshold - very close to wall
    // Use lane-specific thresholds (L1/L4 at 15%, L2/L3 at 20%)
    if (zombieDistances[i] < OVERRIDE_THRESHOLD[i]) {
      return i;
    }
  }
  return -1;
}

//============================================
// GET NEXT FROM QUEUE (LEGACY - only used for queue operations)
//============================================
int getNextFromQueue() {
  updatePendingQueue();
  
  if (pendingQueueSize == 0) return -1;
  
  int bestIdx = 0;
  float bestScore = 0;
  
  for (int i = 0; i < pendingQueueSize; i++) {
    int lane = pendingQueue[i];
    if (lane < 0) continue;
    
    float score = calculateThreatScore(lane);
    if (score > bestScore) {
      bestScore = score;
      bestIdx = i;
    }
  }
  
  int result = pendingQueue[bestIdx];
  
  for (int i = bestIdx; i < pendingQueueSize - 1; i++) {
    pendingQueue[i] = pendingQueue[i + 1];
  }
  pendingQueueSize--;
  
  return result;
}

//============================================
// UPDATE PENDING QUEUE
//============================================
void updatePendingQueue() {
  int newSize = 0;
  for (int i = 0; i < pendingQueueSize; i++) {
    int lane = pendingQueue[i];
    if (lane >= 0 && lane <= 3) {
      // Remove backward-moving targets - they're retreating, not a threat
      if (ProxSensors[lane].direction == BACKWARD) continue;
      
      // Keep if zombie present and moving forward
      // Use lane-specific threshold - L2/L3 engage earlier
      if (ProxSensors[lane].direction == FORWARD) {
        float laneThreshold = getEarlyEngageThreshold(lane);
        if (zombieDistances[lane] < laneThreshold) {
          pendingQueue[newSize++] = lane;
        }
      }
    }
  }
  pendingQueueSize = newSize;
}

//============================================
// DYNAMIC CALIBRATION UPDATE
// Runs during first few seconds, updates ranges
// while still allowing normal targeting
//============================================
void updateCalibration() {
  if (!calibrationActive) return;
  
  unsigned long elapsed = millis() - calibrationStartTime;
  if (elapsed >= CALIBRATION_DURATION) {
    // Calibration period over - apply results
    applyCalibration();
    calibrationActive = false;
    Serial.println(F("=== CALIBRATION COMPLETE ==="));
    return;
  }
  
  // Update min/max for each lane based on raw readings
  for (int i = 0; i < 4; i++) {
    int rawVal = (int)ProxSensors[i].currVal;
    
    // Track minimum (far/no zombie - higher analog value typically)
    // and maximum (close/zombie present - lower analog value)
    // Note: This assumes lower value = closer object
    
    if (rawVal < calibrationMin[i]) {
      calibrationMin[i] = rawVal;
      calibrationUpdated[i] = true;
    }
    if (rawVal > calibrationMax[i]) {
      calibrationMax[i] = rawVal;
      calibrationUpdated[i] = true;
    }
  }
}

//============================================
// APPLY CALIBRATION RESULTS
// Be conservative - bad calibration causes override thrashing
//============================================
void applyCalibration() {
  Serial.println(F("Applying calibration:"));
  
  for (int i = 0; i < 4; i++) {
    if (calibrationUpdated[i]) {
      // Validate the calibration data
      int range = calibrationMax[i] - calibrationMin[i];
      
      // Need substantial range to be valid
      if (range > 150) {
        // Far = high value (no zombie), Close = low value (zombie close)
        int newFar = calibrationMax[i];
        int newClose = calibrationMin[i];
        
        // Add LARGE margins to prevent false 100% readings
        // Far: add margin so "no zombie" reads as ~5% not 0%
        newFar = min(newFar + 30, 950);
        
        // Close: CRITICAL - cap maximum close value to prevent bad calibration
        // If zombie didn't go all the way to wall during calibration, close value is too high
        // Maximum reasonable close value is ~120 (based on typical sensor behavior)
        const int MAX_CLOSE_VALUE = 120;
        
        // Close: add margin for short lanes (L2, L3) 
        if (i == 1 || i == 2) {
          // Short lanes - zombie never gets as close, add 50% of range as buffer
          newClose = max(newClose - (range / 2), 50);
        } else {
          // Long lanes (L1, L4) - smaller buffer but cap the close value
          newClose = max(newClose - 30, 50);
        }
        
        // CRITICAL: Cap close value to prevent bad calibration on L4
        // If calibration didn't capture zombie at wall, close is too high
        newClose = min(newClose, MAX_CLOSE_VALUE);
        
        // Final sanity check - ensure reasonable range remains
        if (newFar - newClose > 100) {
          ProxRange[i][0] = newFar;
          ProxRange[i][1] = newClose;
          
          Serial.print(F("  L"));
          Serial.print(i + 1);
          Serial.print(F(": ["));
          Serial.print(newFar);
          Serial.print(F(","));
          Serial.print(newClose);
          Serial.print(F("] range="));
          Serial.println(newFar - newClose);
        } else {
          Serial.print(F("  L"));
          Serial.print(i + 1);
          Serial.println(F(": range too small after margins, keeping default"));
        }
      } else {
        Serial.print(F("  L"));
        Serial.print(i + 1);
        Serial.print(F(": insufficient range ("));
        Serial.print(range);
        Serial.println(F("), keeping default"));
      }
    } else {
      Serial.print(F("  L"));
      Serial.print(i + 1);
      Serial.println(F(": no updates"));
    }
  }
}

//============================================
// START CALIBRATION
//============================================
void startCalibration() {
  calibrationActive = true;
  calibrationStartTime = millis();
  
  // Reset calibration tracking
  for (int i = 0; i < 4; i++) {
    calibrationMin[i] = 1023;
    calibrationMax[i] = 0;
    calibrationUpdated[i] = false;
  }
  
  Serial.println(F("=== CALIBRATION STARTED (15s) ==="));
  Serial.println(F("Move zombies through all lanes!"));
}

//============================================
// MAIN LOOP
//============================================
void loop() {
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();
  
  processSerialCommands();
  
  // OPTIMIZATION: Cache encoder position to avoid multiple reads
  unsigned long now = millis();
  if (now - lastEncoderRead >= ENCODER_CACHE_INTERVAL) {
    cachedEncoderPos = encoder.read();
    lastEncoderRead = now;
  }
  
  computeVelocity();
  updateSensors();
  
  // OPTIMIZATION: Update cached critical status efficiently
  if (now - lastCriticalUpdate >= CRITICAL_UPDATE_INTERVAL) {
    for (int i = 0; i < 4; i++) {
      if (laneIsShort[i]) {
        laneIsCritical[i] = (zombieDistances[i] < SHORT_LANE_CRITICAL_DISTANCE);
      } else {
        laneIsCritical[i] = (zombieDistances[i] < LONG_LANE_CRITICAL_DISTANCE);
      }
    }
    lastCriticalUpdate = now;
  }
  
  if (millis() - lastTTIUpdate >= TTI_UPDATE_INTERVAL) {
    updateTTI();
    lastTTIUpdate = millis();
  }
  
  // Update calibration if active (runs alongside normal operation)
  if (calibrationActive) {
    updateCalibration();
  }
  
  //============================================
  // STATE MACHINE - BATCH-BASED TARGETING
  //============================================
  static unsigned long lastOverrideCommit = 0;  // Track when we last committed via override
  static int lastOverrideLane = -1;             // Track which lane we overrode to
  static unsigned long overrideCheckTime = 0;   // Rate limit override checks
  
  if (autoMode && systemEnabled && !gameOver) {
    
    //============================================
    // EMERGENCY OVERRIDE CHECK
    // Interrupts batch execution for critical threats
    //============================================
    int overrideLane = -1;
    float bestOverrideScore = 0;
    
    unsigned long timeSinceOverride = millis() - lastOverrideCommit;
    // CRITICAL FIX: Add cooldown after overrides to prevent rapid switching
    // After an override, wait at least 2 seconds before allowing another override
    const unsigned long OVERRIDE_COOLDOWN = 2000;  // 2 second cooldown after override
    
    // CRITICAL FIX: Allow override checks while dwelling - but still respect cooldown
    // While dwelling, we need to continuously check for critical threats (especially short lanes)
    bool isDwelling = (state == DWELL_AT_TARGET);
    bool canCheckOverride = false;
    
    // Always respect cooldown period after override
    if (timeSinceOverride < OVERRIDE_COOLDOWN) {
      canCheckOverride = false;  // Still in cooldown - don't check overrides
    } else if (isDwelling) {
      // While dwelling: Check overrides every 200ms (slower to prevent rapid switching)
      canCheckOverride = (millis() - overrideCheckTime >= 200);
    } else {
      // While moving: Use cooldown to prevent excessive switching
      canCheckOverride = (timeSinceOverride >= 1000) && (millis() - overrideCheckTime >= 150);
      
      // Don't check overrides while actively moving unless we've been moving for a while
      // OPTIMIZATION: Use cached encoder position
      long currentPos = cachedEncoderPos;
      int distToTarget = abs(currentPos - targetPositions[committedLane]);
      bool activelyMoving = (distToTarget > 100 && state == MOVE_TO_TARGET);
      if (activelyMoving && timeSinceOverride < 3000) {
        canCheckOverride = false;  // TIGHTER: Don't override while actively moving unless 3s has passed
      }
    }
    
    if (canCheckOverride) {
      overrideCheckTime = millis();
      
      for (int i = 0; i < 4; i++) {
        if (i == committedLane) continue;
        if (ProxSensors[i].direction != FORWARD) continue;
        // TIGHTER: Longer anti-return period
        if (i == lastOverrideLane && timeSinceOverride < 2000) continue;  // TIGHTER: 2s anti-return (was 1.2s)
        
        // CRITICAL: Skip STOPPED lanes that have been stopped for > 2 seconds (freeze protection)
        if (ProxSensors[i].direction == STOPPED && laneStoppedTime[i] > 0) {
          unsigned long stoppedDuration = millis() - laneStoppedTime[i];
          if (stoppedDuration > STOPPED_TIMEOUT) {
            continue;  // Skip this lane - it's been frozen too long
          }
        }
        
        // IMPROVED: Don't override to lanes that have already been attempted recently
        if (laneAttempted[i]) {
          unsigned long timeSinceAttempt = millis() - laneAttemptTime[i];
          if (timeSinceAttempt < ATTEMPT_COOLDOWN) {
            continue;  // Skip lanes attempted within cooldown period
          }
        }
        
        // OPTIMIZATION: Use cached critical status instead of recalculating
        float dist = zombieDistances[i];
        bool isCritical = laneIsCritical[i];
        
        // CRITICAL: NEVER allow L1/L4 to override L2/L3 when L1/L4 are not critical
        // L2/L3 have shorter lanes and must be protected to prevent impact
        bool newIsLongLane = laneIsLong[i];
        bool committedIsShortLane = (committedLane >= 0) ? laneIsShort[committedLane] : false;
        if (newIsLongLane && committedIsShortLane && !isCritical) {
          continue;  // Skip - don't allow long lane to override short lane when not critical
        }
        
        // TIGHTER CRITERIA: Only override for truly critical threats
        // OPTIMIZATION: Use cached critical status for committed lane
        bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
        
        // CRITICAL FIX: More aggressive override criteria, especially for short lanes
        // Short lanes (L2/L3) can end the game - must override aggressively when they're getting close
        bool isShortLane = laneIsShort[i];
        bool committedIsLongLane = (committedLane >= 0) ? laneIsLong[committedLane] : false;
        
        bool shouldOverride = false;
        
        // CRITICAL: If batch is locked, be MUCH more restrictive
        // Prevent switching when targets are at similar distances
        if (batchLocked && batchActive) {
          float committedDist = zombieDistances[committedLane];
          float distanceGap = abs(dist - committedDist);
          
          // CRITICAL: Short lanes can override long lanes with smaller gap
          // Short lanes have less time - they need priority even at similar distances
          bool isShortLane = laneIsShort[i];
          bool committedIsLongLane = laneIsLong[committedLane];
          
          // MUCH STRICTER: Prevent switching when targets are at similar distances
          // When both are far (80-100%), require at least 20% gap to override
          // When both are closer, require at least 15% gap
          // EXCEPTION: Short lane overriding long lane only needs 10% gap
          // CRITICAL: At 95-100% distance, require even larger gap (30%) to prevent excessive bouncing
          float minGapRequired;
          if (committedDist > 0.95 && dist > 0.95) {
            // Both at 95-100% - require very large gap (30%) to prevent bouncing
            minGapRequired = 0.30;
          } else if (isShortLane && committedIsLongLane) {
            // Short lane overriding long lane - only need 10% gap
            minGapRequired = 0.10;
          } else {
            // Normal case - require larger gap
            minGapRequired = (committedDist > 0.80 && dist > 0.80) ? 0.20 : 0.15;
          }
          
          if (distanceGap < minGapRequired) {
            continue;  // Skip - targets too similar in distance
          }
          
          // CRITICAL: NEVER allow L1/L4 to override L2/L3 when L1/L4 are not critical
          // L2/L3 have shorter lanes and must be protected to prevent impact
          bool isLongLane = laneIsLong[i];
          bool committedIsShortLane = laneIsShort[committedLane];
          
          if (isLongLane && committedIsShortLane && !isCritical) {
            // Long lane (L1/L4) trying to override short lane (L2/L3) when not critical
            // STRICT: Only allow if new lane is MUCH closer (at least 50% closer) OR is critical
            // This ensures L2/L3 have priority when distances are similar
            if (dist >= committedDist * 0.5) {
              continue;  // Skip - don't allow override, protect short lanes
            }
          }
          
          // When batch is locked, only override for:
          // 1. Critical vs non-critical (game-ending threat)
          // 2. Short lane getting very close (< 15%) while committed to long lane
          // 3. Extreme emergency (< 10% distance)
          // 4. Both critical but new is significantly closer (at least 30% closer, not 20%)
          if (isCritical && !committedIsCritical) {
            shouldOverride = true;  // Critical vs non-critical = always override
          } else if (isShortLane && committedIsLongLane && dist < 0.15) {
            shouldOverride = true;  // Short lane very close while at long lane
          } else if (dist < 0.10) {
            shouldOverride = true;  // Extreme emergency
          } else if (isCritical && committedIsCritical && dist < committedDist * 0.7) {
            shouldOverride = true;  // Both critical, new is 30%+ closer (stricter)
          }
          // Otherwise, stick to batch sequence
        } else {
          // Batch not locked - but still prevent switching when targets are at similar distances
          float committedDist = zombieDistances[committedLane];
          float distanceGap = abs(dist - committedDist);
          
          // CRITICAL: Short lanes can override long lanes with smaller gap
          // Short lanes have less time - they need priority even at similar distances
          bool isShortLane = laneIsShort[i];
          bool committedIsLongLane = laneIsLong[committedLane];
          
          // Prevent switching when targets are at similar distances (even if batch not locked)
          // When both are far (80-100%), require at least 20% gap to override
          // When both are closer, require at least 15% gap
          // EXCEPTION: Short lane overriding long lane only needs 10% gap
          // CRITICAL: At 95-100% distance, require even larger gap (30%) to prevent excessive bouncing
          float minGapRequired;
          if (committedDist > 0.95 && dist > 0.95) {
            // Both at 95-100% - require very large gap (30%) to prevent bouncing
            minGapRequired = 0.30;
          } else if (isShortLane && committedIsLongLane) {
            // Short lane overriding long lane - only need 10% gap
            minGapRequired = 0.10;
          } else {
            // Normal case - require larger gap
            minGapRequired = (committedDist > 0.80 && dist > 0.80) ? 0.20 : 0.15;
          }
          
          if (distanceGap < minGapRequired) {
            continue;  // Skip - targets too similar in distance
          }
          
          // CRITICAL: NEVER allow L1/L4 to override L2/L3 when L1/L4 are not critical
          // L2/L3 have shorter lanes and must be protected to prevent impact
          bool isLongLane = laneIsLong[i];
          bool committedIsShortLane = laneIsShort[committedLane];
          
          if (isLongLane && committedIsShortLane && !isCritical) {
            // Long lane (L1/L4) trying to override short lane (L2/L3) when not critical
            // STRICT: Only allow if new lane is MUCH closer (at least 50% closer) OR is critical
            // This ensures L2/L3 have priority when distances are similar
            if (dist >= committedDist * 0.5) {
              continue;  // Skip - don't allow override, protect short lanes
            }
          }
          
          // Batch not locked - use normal override criteria
          // PRIORITY 1: Short lane getting close while committed to long lane = IMMEDIATE OVERRIDE
          // CRITICAL FIX: Distance is LOW when close to impact (0.0 = impact, 1.0 = far)
          // So check for LOW distance values, not HIGH ones
          if (isShortLane && committedIsLongLane && dist < 0.25) {
            // Short lane at < 25% distance (75%+ remaining, getting very close) while dwelling at long lane = override immediately
            shouldOverride = true;
          }
          // PRIORITY 2: Critical vs non-critical = override
          else if (isCritical && !committedIsCritical) {
            shouldOverride = true;
          }
          // PRIORITY 3: Critical and much closer (at least 30% closer)
          else if (isCritical && committedIsCritical && dist < committedDist * 0.7) {
            shouldOverride = true;
          }
          // PRIORITY 4: Short lane getting close while long lane is at less urgent distance
          // CRITICAL FIX: Check for LOW distance (close to impact), not HIGH
          else if (isShortLane && dist < 0.30 && committedIsLongLane && committedDist > 0.20) {
            // Short lane at < 30% distance (70%+ remaining) while long lane still has 20%+ distance = override
            shouldOverride = true;
          }
          // PRIORITY 5: Below override threshold and committed is not critical
          else if (dist < OVERRIDE_THRESHOLD[i] && !committedIsCritical) {
            shouldOverride = true;
          }
        }
        
        if (shouldOverride) {
          float score = calculateThreatScore(i);
          // OPTIMIZATION: Use cached lane type instead of recalculating
          // Massive boost for short lanes getting close (LOW distance = close to impact)
          if (isShortLane) {
            if (dist < 0.15) {
              score += 15000;  // Huge boost for short lanes at <15% distance (85%+ remaining, very close!)
            } else if (dist < 0.20) {
              score += 10000;  // Large boost for short lanes at <20% distance (80%+ remaining)
            } else if (isCritical) {
              score += 5000;  // Boost for critical short lanes
            }
          }
          // Boost for any critical threat
          else if (isCritical) {
            score += 5000;
          }
          if (score > bestOverrideScore) {
            bestOverrideScore = score;
            overrideLane = i;
          }
        }
      }
    }
    
    // EMERGENCY OVERRIDE - reset batch and handle immediately
    if (overrideLane >= 0 && shouldOverride(overrideLane)) {
      int previousLane = committedLane;
      
      Serial.print(F("!!! OVERRIDE L"));
      Serial.print(overrideLane + 1);
      Serial.print(F(" @"));
      Serial.print((int)((1.0 - zombieDistances[overrideLane]) * 100));
      Serial.println(F("% !!!"));
      
      // Reset batch on emergency - will recalculate after this target
      resetBatch();
      
      releaseCommitment();
      commitToTarget(overrideLane);
      
      if (isCommitted) {
        lastOverrideCommit = millis();
        lastOverrideLane = previousLane;
        state = MOVE_TO_TARGET;
      }
    }
    //============================================
    // NORMAL BATCH EXECUTION
    //============================================
    else if (isCommitted && committedLane >= 0) {
      // Ensure we're targeting the committed lane
      if (activeTargetIndex != committedLane) {
        activeTargetIndex = committedLane;
        desiredPosition = targetPositions[committedLane];
      }
      
      switch (state) {
        case MOVE_TO_TARGET:
          moveToTarget();
          break;
        case DWELL_AT_TARGET:
          dwellAtTarget();
          break;
        default:
          state = MOVE_TO_TARGET;
          break;
      }
    }
    //============================================
    // NO COMMITMENT - GET NEXT FROM BATCH
    //============================================
    else {
      switch (state) {
        case IDLE:
          state = CHOOSE_TARGET;
          break;
        
        case CHOOSE_TARGET:
          chooseAndCommitTargetFromBatch();
          break;
        
        case MOVE_TO_TARGET:
          moveToTarget();
          break;
        
        case DWELL_AT_TARGET:
          dwellAtTarget();
          break;
      }
    }
  }
  
  //============================================
  // MOTOR CONTROL
  //============================================
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
  
  //============================================
  // STATUS OUTPUT
  //============================================
  if (autoMode && (millis() - lastPrintTime >= 200)) {
    lastPrintTime = millis();
    printStatus();
  }
}

//============================================
// CHOOSE AND COMMIT TO TARGET
//============================================
void chooseAndCommitTarget() {
  // Find the best target from ALL sources - queue AND new detections
  int bestLane = -1;
  float bestScore = 0;
  
  // Check pending queue first
  updatePendingQueue();  // Clean up stale entries
  for (int i = 0; i < pendingQueueSize; i++) {
    int lane = pendingQueue[i];
    if (lane < 0 || lane > 3) continue;
    if (ProxSensors[lane].direction != FORWARD) continue;
    
    float dist = zombieDistances[lane];
    // Use lane-specific threshold - L2/L3 engage earlier
    float laneThreshold = getEarlyEngageThreshold(lane);
    if (dist < MIN_ENGAGE_THRESHOLD || dist > laneThreshold) continue;
    
    float score = calculateThreatScore(lane);
    if (score > bestScore) {
      bestScore = score;
      bestLane = lane;
    }
  }
  
  // Now check all lanes for new targets - might be better than queued
  for (int i = 0; i < 4; i++) {
    // Skip backward-moving targets - they're retreating
    if (ProxSensors[i].direction != FORWARD) continue;
    
    float dist = zombieDistances[i];
    
    // Skip if zombie is outside valid engagement range - use lane-specific threshold
    float laneThreshold = getEarlyEngageThreshold(i);
    if (dist < MIN_ENGAGE_THRESHOLD || dist > laneThreshold) continue;
    
    float score = calculateThreatScore(i);
    
    if (score > bestScore) {
      bestScore = score;
      bestLane = i;
    }
  }
  
  if (bestLane >= 0) {
    // Remove from queue if it was queued
    for (int i = 0; i < pendingQueueSize; i++) {
      if (pendingQueue[i] == bestLane) {
        for (int j = i; j < pendingQueueSize - 1; j++) {
          pendingQueue[j] = pendingQueue[j + 1];
        }
        pendingQueueSize--;
        break;
      }
    }
    
    commitToTarget(bestLane);
    if (isCommitted) {
      state = MOVE_TO_TARGET;
    }
  } else {
    // No targets - go to wait position
    desiredPosition = WAIT_POSITION;
    WAIT_POS = true;
    activeTargetIndex = -1;
    releaseCommitment();
  }
}

//============================================
// CHOOSE AND COMMIT TARGET FROM BATCH
// Uses the batch system for more predictable targeting
// STICKS TO BATCH SEQUENCE when locked (groups of 3)
//============================================
void chooseAndCommitTargetFromBatch() {
  // If batch is locked, ONLY get targets from the batch sequence
  // Don't allow breaking sequence except for backward targets (handled in getNextBatchTarget)
  if (batchLocked && batchActive) {
    // Get next target from current batch sequence
    int nextLane = getNextBatchTarget();
    
    if (nextLane >= 0) {
      // Verify this lane is still valid (forward-moving, in range)
      // Use lane-specific threshold
      // IMPROVED: Also check if lane has already been attempted
      float laneThreshold = getEarlyEngageThreshold(nextLane);
      bool recentlyAttempted = false;
      if (laneAttempted[nextLane]) {
        unsigned long timeSinceAttempt = millis() - laneAttemptTime[nextLane];
        if (timeSinceAttempt < ATTEMPT_COOLDOWN) {
          recentlyAttempted = true;
        } else {
          // Cooldown expired - reset attempted flag
          laneAttempted[nextLane] = false;
        }
      }
      
      // CRITICAL: For L2/L3 in batch sequence, still try to target even if at high distance (85-100%)
      // Short lanes move fast, so even at 85-100% we might still catch them
      bool isShortLane = (nextLane == 1 || nextLane == 2);  // L2 or L3
      bool isAtHighDistance = (zombieDistances[nextLane] >= 0.85f && zombieDistances[nextLane] <= 1.0f);
      bool isForwardOrStoppedBriefly = (ProxSensors[nextLane].direction == FORWARD) ||
                                        (ProxSensors[nextLane].direction == STOPPED && 
                                         (laneStoppedTime[nextLane] == 0 || 
                                          (millis() - laneStoppedTime[nextLane]) < 500));  // Only recently stopped
      
      // For short lanes in batch sequence at high distance, be more lenient
      bool isValidForShortLane = isShortLane && isAtHighDistance && isForwardOrStoppedBriefly && !recentlyAttempted;
      
      // Standard validation for all other cases
      bool isValidStandard = (ProxSensors[nextLane].direction == FORWARD &&
                              zombieDistances[nextLane] < laneThreshold &&
                              zombieDistances[nextLane] > MIN_ENGAGE_THRESHOLD &&
                              !recentlyAttempted);
      
      if (isValidForShortLane || isValidStandard) {
        // Target is valid - commit to it (batch sequence enforced)
        commitToTarget(nextLane);
        if (isCommitted) {
          state = MOVE_TO_TARGET;
        }
        return;
      } else {
        // Target became invalid (backward or out of range) - remove from batch and continue
        // getNextBatchTarget already handles this, so just get next one
        nextLane = getNextBatchTarget();
        if (nextLane >= 0) {
          commitToTarget(nextLane);
          if (isCommitted) {
            state = MOVE_TO_TARGET;
          }
          return;
        }
      }
    }
    
    // Batch exhausted but was locked - unlock and recalculate
    if (batchIndex >= batchSize) {
      batchLocked = false;
      batchActive = false;
    }
  }
  
  // Get next target from batch (will calculate new batch if needed)
  int nextLane = getNextBatchTarget();
  
  if (nextLane >= 0) {
    commitToTarget(nextLane);
    if (isCommitted) {
      state = MOVE_TO_TARGET;
    }
  } else {
    // No targets - go to wait position
    desiredPosition = WAIT_POSITION;
    WAIT_POS = true;
    activeTargetIndex = -1;
    releaseCommitment();
    batchActive = false;
  }
}

//============================================
// MOVE TO TARGET
//============================================
void moveToTarget() {
  // OPTIMIZATION: Use cached encoder position
  long currentPos = cachedEncoderPos;
  
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    arrivalTime = millis();
    if (activeTargetIndex >= 0) {
      peakZombieDistance = zombieDistances[activeTargetIndex];
      arrivalZombieDistance = zombieDistances[activeTargetIndex];
      
      // CRITICAL FIX: Don't mark as attempted on arrival - only mark after successful hit
      // This allows the system to re-engage the same lane if the first attempt fails
      // The attempted flag will be set only after a successful hit or timeout
      // laneAttempted[activeTargetIndex] = true;  // REMOVED - only mark after hit/timeout
    }
    backwardStartTime = 0;
    
    if (WAIT_POS) {
      state = CHOOSE_TARGET;
    } else {
      state = DWELL_AT_TARGET;
      if (verboseMode) {
        Serial.print(F("~ARRIVE L"));
        Serial.println(activeTargetIndex + 1);
      }
    }
  }
}

//============================================
// DWELL AT TARGET
//============================================
void dwellAtTarget() {
  if (activeTargetIndex < 0) {
    releaseCommitment();
    state = CHOOSE_TARGET;
    return;
  }
  
  // CRITICAL FIX: Check for emergency overrides while dwelling
  // This prevents missing short lane targets (L2/L3) that are getting close while dwelling at long lanes
  // ANTI-OSCILLATION: Check minimum commitment time and cooldown before allowing overrides
  unsigned long commitmentDuration = millis() - commitStartTime;
  unsigned long timeSinceSwitch = millis() - lastTargetSwitchTime;
  
  int overrideLane = -1;
  float bestOverrideScore = 0;
  
  for (int i = 0; i < 4; i++) {
    if (i == activeTargetIndex) continue;  // Skip committed lane
    if (ProxSensors[i].direction != FORWARD) continue;  // Only forward-moving
    
    float dist = zombieDistances[i];
    bool isExtremeEmergency = (dist < 0.08);  // Only extreme emergencies can override during cooldown
    
    // ANTI-OSCILLATION: Require minimum commitment time unless extreme emergency
    if (commitmentDuration < MIN_COMMITMENT_TIME && !isExtremeEmergency) {
      continue;  // Too soon to override - stick with current target
    }
    
    // ANTI-OSCILLATION: Check target switch cooldown
    if (timeSinceSwitch < TARGET_SWITCH_COOLDOWN && !isExtremeEmergency) {
      continue;  // Still in cooldown period - prevent oscillation
    }
    
    // CRITICAL: Skip STOPPED lanes that have been stopped for > 2 seconds (freeze protection)
    if (ProxSensors[i].direction == STOPPED && laneStoppedTime[i] > 0) {
      unsigned long stoppedDuration = millis() - laneStoppedTime[i];
      if (stoppedDuration > STOPPED_TIMEOUT) {
        continue;  // Skip this lane - it's been frozen too long
      }
    }
    
    bool isShortLane = laneIsShort[i];
    bool isLongLane = laneIsLong[i];
    bool committedIsLongLane = laneIsLong[activeTargetIndex];
    bool committedIsShortLane = laneIsShort[activeTargetIndex];
    bool isCritical = laneIsCritical[i];
    
    // CRITICAL: NEVER allow L1/L4 to override L2/L3 when dwelling, unless L1/L4 is critical
    // L2/L3 have shorter lanes and must be protected to prevent impact
    if (isLongLane && committedIsShortLane && !isCritical) {
      continue;  // Skip - don't allow long lane to override short lane when not critical
    }
    
    // CRITICAL: Short lanes (L2/L3) can override long lanes (L1/L4) when getting close
    // If we're dwelling at a long lane and a short lane is getting close, override immediately
    bool shouldOverride = false;
    
    // CRITICAL: If batch is locked, be MUCH more restrictive
    // Prevent switching when targets are at similar distances
    if (batchLocked && batchActive) {
      float committedDist = zombieDistances[activeTargetIndex];
      float distanceGap = abs(dist - committedDist);
      
      // CRITICAL: Short lanes can override long lanes with smaller gap
      // Short lanes have less time - they need priority even at similar distances
      bool isShortLane = laneIsShort[i];
      bool committedIsLongLane = laneIsLong[activeTargetIndex];
      
      // MUCH STRICTER: Prevent switching when targets are at similar distances
      // Increased gap requirements to prevent oscillation
      // When both are far (80-100%), require at least 25% gap to override (increased from 20%)
      // When both are closer, require at least 20% gap (increased from 15%)
      // EXCEPTION: Short lane overriding long lane needs 15% gap (increased from 10%)
      // CRITICAL: At 95-100% distance, require even larger gap (35%) to prevent excessive bouncing (increased from 30%)
      float minGapRequired;
      if (committedDist > 0.95 && dist > 0.95) {
        // Both at 95-100% - require very large gap (35%) to prevent bouncing
        minGapRequired = 0.35;
      } else if (isShortLane && committedIsLongLane) {
        // Short lane overriding long lane - need 15% gap (increased from 10%)
        minGapRequired = 0.15;
      } else {
        // Normal case - require larger gap
        minGapRequired = (committedDist > 0.80 && dist > 0.80) ? 0.25 : 0.20;
      }
      
      if (distanceGap < minGapRequired) {
        continue;  // Skip - targets too similar in distance
      }
      
      // When batch is locked, only override for:
      // 1. Critical vs non-critical (game-ending threat)
      // 2. Short lane getting very close (< 15%) while committed to long lane
      // 3. Extreme emergency (< 10% distance)
      // 4. Both critical but new is significantly closer (at least 20% closer)
      if (isCritical && !laneIsCritical[activeTargetIndex]) {
        shouldOverride = true;  // Critical vs non-critical = always override
      } else if (isShortLane && committedIsLongLane && dist < 0.15) {
        shouldOverride = true;  // Short lane very close while at long lane
      } else if (dist < 0.10) {
        shouldOverride = true;  // Extreme emergency
      } else if (isCritical && laneIsCritical[activeTargetIndex] && dist < committedDist * 0.8) {
        shouldOverride = true;  // Both critical, new is 20%+ closer
      }
      // Otherwise, stick to batch sequence
    } else {
      // Batch not locked - but still prevent switching when targets are at similar distances
      float committedDist = zombieDistances[activeTargetIndex];
      float distanceGap = abs(dist - committedDist);
      
      // CRITICAL: Short lanes can override long lanes with smaller gap
      // Short lanes have less time - they need priority even at similar distances
      bool isShortLane = laneIsShort[i];
      bool committedIsLongLane = laneIsLong[activeTargetIndex];
      
      // Prevent switching when targets are at similar distances (even if batch not locked)
      // Increased gap requirements to prevent oscillation
      // When both are far (80-100%), require at least 25% gap to override (increased from 20%)
      // When both are closer, require at least 20% gap (increased from 15%)
      // EXCEPTION: Short lane overriding long lane needs 15% gap (increased from 10%)
      float minGapRequired;
      if (committedDist > 0.95 && dist > 0.95) {
        // Both at 95-100% - require very large gap (35%) to prevent bouncing
        minGapRequired = 0.35;
      } else if (isShortLane && committedIsLongLane) {
        // Short lane overriding long lane - need 15% gap (increased from 10%)
        minGapRequired = 0.15;
      } else {
        // Normal case - require larger gap
        minGapRequired = (committedDist > 0.80 && dist > 0.80) ? 0.25 : 0.20;
      }
      
      if (distanceGap < minGapRequired) {
        continue;  // Skip - targets too similar in distance
      }
      
      // Batch not locked - use normal override criteria
      // CRITICAL FIX: Distance is normalized where 0.0 = at impact, 1.0 = far
      // So LOW distance values mean close to impact (urgent!)
      // If L3 shows "83%" remaining, distance = 0.17 (17% through lane, 83% remaining)
      // We want to override when distance is LOW (close to impact)
      
      if (isShortLane && committedIsLongLane) {
        if (isCritical || dist < 0.20) shouldOverride = true;
      } else if (isCritical && !laneIsCritical[activeTargetIndex]) {
        shouldOverride = true;
      } else if (dist < 0.15 && committedIsLongLane) {
        shouldOverride = true;
      } else if (dist < OVERRIDE_THRESHOLD[i]) {
        shouldOverride = true;
      }
    }
    
    if (shouldOverride) {
      float score = calculateThreatScore(i);
      if (isShortLane) {
        if (dist < 0.15) score += 15000;
        else if (dist < 0.20) score += 10000;
        else if (isCritical) score += 5000;
      }
      if (isCritical) score += 5000;
      if (score > bestOverrideScore) {
        bestOverrideScore = score;
        overrideLane = i;
      }
    }
  }
  
  // EMERGENCY OVERRIDE while dwelling - handle immediately
  if (overrideLane >= 0 && shouldOverride(overrideLane)) {
    int previousLane = activeTargetIndex;
    
    Serial.print(F("!!! OVERRIDE L"));
    Serial.print(overrideLane + 1);
    Serial.print(F(" @"));
    Serial.print((int)((1.0 - zombieDistances[overrideLane]) * 100));
    Serial.println(F("% !!!"));
    
    // Reset batch on emergency override
    resetBatch();
    
    releaseCommitment();
    commitToTarget(overrideLane);
    
    if (isCommitted) {
      state = MOVE_TO_TARGET;
      return;  // Exit dwell immediately to handle override
    }
  }
  
  unsigned long dwellTime = millis() - arrivalTime;
  float currentDist = zombieDistances[activeTargetIndex];
  int currentDir = ProxSensors[activeTargetIndex].direction;
  
  unsigned long maxDwell = (activeTargetIndex == 3) ? L4_DWELL_TIME : MAX_DWELL_TIME;
  
  if (currentDist < peakZombieDistance) {
    peakZombieDistance = currentDist;
  }
  
  if (currentDir == BACKWARD && dwellTime >= 50) {
    if (backwardStartTime == 0) {
      backwardStartTime = millis();
    }
      if (millis() - backwardStartTime >= BACKWARD_CONFIRM_TIME) {
      zombiesKilled++;
      recordHit(activeTargetIndex);  // Analysis mode tracking
      Serial.print(F("HIT L"));
      Serial.print(activeTargetIndex + 1);
      Serial.print(F(" ["));
      Serial.print(zombiesKilled);
      Serial.println(F("]"));
      
      backwardStartTime = 0;
      stoppedStartTime = 0;
      releaseCommitment();
      
      // Advance batch (will unlock if sequence complete)
      advanceBatch();
      
      // Get next target from batch (respects batch lock) - IMMEDIATE transition, no delay
      // IMPROVED: Skip lanes that have already been attempted recently
      int next = getNextBatchTarget();
      if (next >= 0 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
        commitToTarget(next);
        if (isCommitted) {
          state = MOVE_TO_TARGET;  // Immediately move to next target
        } else {
          // If commit failed, try chooseAndCommitTargetFromBatch for immediate retry
          state = CHOOSE_TARGET;
        }
      } else {
        // Batch complete or no valid targets - unlock and immediately choose next
        if (batchLocked) {
          batchLocked = false;
        }
        // Immediately try to choose next target instead of waiting
        chooseAndCommitTargetFromBatch();
        if (isCommitted) {
          state = MOVE_TO_TARGET;
        } else {
          state = CHOOSE_TARGET;
        }
      }
      return;
    }
  } else {
    backwardStartTime = 0;
  }
  
  if (dwellTime >= 100 && peakZombieDistance < 0.15) {
    // Zombie got to within 15% (close to wall)
    float retreatAmount = currentDist - peakZombieDistance;
    if (retreatAmount > 0.04) {  // Moved back 4%+ from peak
      zombiesKilled++;
      recordHit(activeTargetIndex);  // Analysis mode tracking
      Serial.print(F("HIT L"));
      Serial.print(activeTargetIndex + 1);
      Serial.print(F(" ["));
      Serial.print(zombiesKilled);
      Serial.println(F("]"));
      
      stoppedStartTime = 0;
      if (!laneAttempted[activeTargetIndex]) {
        laneAttempted[activeTargetIndex] = true;
        laneAttemptTime[activeTargetIndex] = millis();
      }
      releaseCommitment();
      advanceBatch();
      int next = getNextBatchTarget();
      if (next >= 0 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
        commitToTarget(next);
        if (isCommitted) state = MOVE_TO_TARGET;
        else {
          chooseAndCommitTargetFromBatch();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
        }
      } else {
        if (batchLocked) {
          batchLocked = false;
        }
        // Immediately try to choose next target
        chooseAndCommitTargetFromBatch();
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      }
      return;
    }
  }
  
  if (dwellTime >= MIN_DWELL_TIME &&
      currentDist > RETREAT_CONFIRMED_DISTANCE && 
      currentDist > arrivalZombieDistance + 0.15) {
    zombiesKilled++;
    recordHit(activeTargetIndex);  // Analysis mode tracking
    Serial.print(F("HIT L"));
    Serial.print(activeTargetIndex + 1);
    Serial.print(F(" (far) ["));
    Serial.print(zombiesKilled);
    Serial.println(F("]"));
    
        if (!laneAttempted[activeTargetIndex]) {
          laneAttempted[activeTargetIndex] = true;
          laneAttemptTime[activeTargetIndex] = millis();
        }
        releaseCommitment();
        stoppedStartTime = 0;
        int next = getNextBatchTarget();
        if (next >= 0 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
          commitToTarget(next);
          if (isCommitted) state = MOVE_TO_TARGET;
          else {
            chooseAndCommitTargetFromBatch();
            if (isCommitted) state = MOVE_TO_TARGET;
            else state = CHOOSE_TARGET;
          }
        } else {
          chooseAndCommitTargetFromBatch();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
        }
        return;
  }
  
  // Check for STOPPED direction OR very small velocity (pseudo-stopped)
  bool isEffectivelyStopped = (currentDir == STOPPED) || 
                               (abs(zombieVelocities[activeTargetIndex]) < 0.0001);
  
  // IMPROVED: Only check STOPPED exit after minimum persistent dwell time
  if (isEffectivelyStopped && dwellTime >= 200) {  // REDUCED: Require 200ms before checking STOPPED
    if (stoppedStartTime == 0) {
      stoppedStartTime = millis();
      stoppedStartDistance = currentDist;
    } else if (millis() - stoppedStartTime >= 300) {  // REDUCED: 300ms confirmed stopped
      // Check if distance barely changed - zombie truly stalled
      float distChange = abs(currentDist - stoppedStartDistance);
      if (distChange < 0.08) {  // Increased from 5% to 8% - more tolerant
        // If zombie was close, probably a hit - be generous
        if (peakZombieDistance < 0.30) {  // INCREASED threshold - more generous hit credit (was 0.25)
          zombiesKilled++;
          recordHit(activeTargetIndex);
          Serial.print(F("HIT L"));
          Serial.print(activeTargetIndex + 1);
          Serial.print(F(" (stop) ["));
          Serial.print(zombiesKilled);
          Serial.println(F("]"));
          
          // Mark as attempted and hit
          laneAttempted[activeTargetIndex] = true;
          laneAttemptTime[activeTargetIndex] = millis();
          
          releaseCommitment();
          stoppedStartTime = 0;
          // IMMEDIATE transition - no delay
          int next = getNextBatchTarget();
          if (next >= 0 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
            commitToTarget(next);
            if (isCommitted) state = MOVE_TO_TARGET;
            else {
              chooseAndCommitTargetFromBatch();
              if (isCommitted) state = MOVE_TO_TARGET;
              else state = CHOOSE_TARGET;
            }
          } else {
            // Immediately try to choose next target
            chooseAndCommitTargetFromBatch();
            if (isCommitted) state = MOVE_TO_TARGET;
            else state = CHOOSE_TARGET;
          }
          return;
        } else {
          // CRITICAL: For L2/L3 (short lanes), NEVER give up if they're still close to sensors
          // Short lanes have less time - we must keep trying to prevent impact
          bool isShortLane = laneIsShort[activeTargetIndex];
          bool isStillClose = (currentDist < 0.20);  // Still within 20% of sensor
          
          // For short lanes that are still close, keep trying - don't exit
          if (isShortLane && isStillClose) {
            // Keep dwelling - don't give up on short lanes when they're close
            return;  // Continue trying
          }
          
          // IMPROVED: Only exit on STOP if we've been dwelling for a long time
          // Be persistent - don't give up too easily
          if (dwellTime >= 400) {  // REDUCED: Only exit if we've been here 400ms+ without progress
            Serial.print(F("STOP L"));
            Serial.println(activeTargetIndex + 1);
            
            // Mark as attempted - prevent immediate re-engagement
            laneAttempted[activeTargetIndex] = true;
            laneAttemptTime[activeTargetIndex] = millis();
            
            releaseCommitment();
            stoppedStartTime = 0;
            // IMMEDIATE transition - no delay
            int next = getNextBatchTarget();
            if (next >= 0 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
              commitToTarget(next);
              if (isCommitted) state = MOVE_TO_TARGET;
              else {
                chooseAndCommitTargetFromBatch();
                if (isCommitted) state = MOVE_TO_TARGET;
                else state = CHOOSE_TARGET;
              }
            } else {
              // Immediately try to choose next target
              chooseAndCommitTargetFromBatch();
              if (isCommitted) state = MOVE_TO_TARGET;
              else state = CHOOSE_TARGET;
            }
            return;
          }
          // Otherwise, keep trying - don't exit yet
        }
      } else {
        // Distance changed - reset stopped timer
        stoppedStartTime = millis();
        stoppedStartDistance = currentDist;
      }
    }
  } else if (currentDir != STOPPED && abs(zombieVelocities[activeTargetIndex]) > 0.0005) {
    stoppedStartTime = 0;  // Clearly moving, reset
  }
  // Note: Don't reset if direction flickers briefly - keep timing going
  
  //--------------------------------------------
  // EXIT CONDITION 3: ZOMBIE COMPLETELY GONE
  // CRITICAL FIX: Much more conservative - only mark as GONE if:
  // 1. Target is beyond lane-specific threshold (85-90% of lane)
  // 2. Target has been consistently far for extended time
  // 3. Target is moving backward (retreating past the threshold)
  // For short lanes (L2/L3), use even higher threshold (90%+)
  //--------------------------------------------
  // Determine lane-specific gone threshold
  float goneThreshold;
  if (laneIsShort[activeTargetIndex]) {
    // L2/L3 - short lanes - VERY conservative (90%+)
    goneThreshold = L2_L3_GONE_DISTANCE;
  } else if (activeTargetIndex == 3) {
    // L4 - longer lane but still conservative (88%)
    goneThreshold = L4_GONE_DISTANCE;
  } else {
    // L1 - conservative (85%)
    goneThreshold = ZOMBIE_GONE_DISTANCE;
  }
  
  // CRITICAL FIX: Much stricter validation - only mark as GONE if zombie is TRULY unreachable
  // 1. Target must be beyond very conservative threshold (85-90% of lane)
  // 2. Target must be moving backward (retreating past threshold) OR consistently far for extended time
  // 3. Must have dwelt long enough to confirm (prevents false GONE on arrival)
  // 4. For short lanes (L2/L3), require even higher threshold (90%+) since they're critical
  bool isBeyondThreshold = (currentDist > goneThreshold);
  bool isRetreating = (currentDir == BACKWARD);
  bool hasBeenFarForExtendedTime = (dwellTime >= (MIN_DWELL_TIME * 2));  // REDUCED: Require 2x min dwell
  bool isConsistentlyFar = (currentDist > (goneThreshold - 0.05));  // Must be very close to threshold (within 5%)
  
  // CRITICAL: Only mark as GONE if:
  // - Target is retreating AND beyond threshold (definitely gone), OR
  // - Target has been consistently far beyond threshold for extended time (3x min dwell)
  if (dwellTime >= MIN_DWELL_TIME && isBeyondThreshold) {
    // Additional validation - must be retreating OR consistently far for extended time
    // CRITICAL: For short lanes, require even more validation
    bool shouldMarkGone = false;
    
    if (laneIsShort[activeTargetIndex]) {
      // L2/L3 - Short lanes: EXTREMELY conservative - only mark as GONE if truly unreachable
      // CRITICAL: NEVER mark L2/L3 as GONE if they're still close to sensors (< 20%)
      // This prevents abandoning short lanes when all zombies are at start position
      if (currentDist < 0.20) {
        shouldMarkGone = false;  // Still close - keep trying
      } else {
        // 1. Retreating AND beyond 95% threshold (only 5% remaining), OR
        // 2. Consistently far for extended time AND beyond 95% threshold
        // CRITICAL: Never mark short lanes as GONE if there's any chance of hitting them
        shouldMarkGone = (isRetreating && currentDist > L2_L3_GONE_DISTANCE) || 
                         (hasBeenFarForExtendedTime && isConsistentlyFar && currentDist > L2_L3_GONE_DISTANCE);
      }
    } else {
      // L1/L4 - Long lanes: Still conservative but slightly less strict
      // Mark as GONE if retreating OR consistently far for extended time
      shouldMarkGone = (isRetreating && currentDist > goneThreshold) || 
                       (hasBeenFarForExtendedTime && isConsistentlyFar && currentDist > goneThreshold);
    }
    
    if (shouldMarkGone) {
      Serial.print(F("GONE L"));
      Serial.print(activeTargetIndex + 1);
      Serial.print(F(" @"));
      Serial.print((int)((1.0 - currentDist) * 100));
      Serial.println(F("%"));
      
      // CRITICAL FIX: Reset attempted flag when GONE - allow immediate re-engagement if target reappears
      // This allows the system to try again immediately if a new zombie appears in this lane
      laneAttempted[activeTargetIndex] = false;  // Clear attempted flag - allow re-engagement
      
      releaseCommitment();
      // IMMEDIATE transition - don't skip attempted lanes since we're not marking as attempted
      int next = getNextBatchTarget();
      if (next >= 0 && ProxSensors[next].direction == FORWARD) {
        commitToTarget(next);
        if (isCommitted) state = MOVE_TO_TARGET;
        else {
          chooseAndCommitTargetFromBatch();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
        }
      } else {
        // Immediately try to choose next target
        chooseAndCommitTargetFromBatch();
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      }
      return;
    }
    // Otherwise, keep waiting - target might be coming back
  }
  
  //--------------------------------------------
  // EXIT CONDITION 4: TIMEOUT
  // IMPROVED: Mark lane as attempted on timeout - prevent immediate re-engagement
  //--------------------------------------------
  if (dwellTime >= maxDwell) {
    // IMPROVED: Mark lane as attempted - prevent immediate re-engagement
    laneAttempted[activeTargetIndex] = true;
    laneAttemptTime[activeTargetIndex] = millis();
    
    if (currentDir == FORWARD && currentDist < arrivalZombieDistance - 0.03) {
      zombiesMissed++;
      Serial.print(F("MISS L"));
      Serial.print(activeTargetIndex + 1);
      Serial.print(F(" ["));
      Serial.print(zombiesMissed);
      Serial.println(F("]"));
    } else if (currentDir == STOPPED) {
      // STOPPED at very close range = likely hit wall = miss
      if (currentDist < 0.08) {
      }
      Serial.print(F("STOP L"));
      Serial.println(activeTargetIndex + 1);
    } else {
      Serial.print(F("TIME L"));
      Serial.println(activeTargetIndex + 1);
    }
    
    releaseCommitment();
    // IMMEDIATE transition - no delay (skip attempted lanes)
    int next = getNextBatchTarget();
    if (next >= 0 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
      commitToTarget(next);
      if (isCommitted) state = MOVE_TO_TARGET;
      else {
        chooseAndCommitTargetFromBatch();
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      }
    } else {
      // Immediately try to choose next target (skip attempted lanes)
      chooseAndCommitTargetFromBatch();
      if (isCommitted) state = MOVE_TO_TARGET;
      else state = CHOOSE_TARGET;
    }
    return;
  }
}

//============================================
// SERIAL COMMANDS
//============================================
void processSerialCommands() {
  if (Serial.available() == 0) return;
  
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
    case 'T': case 't':
    case 'Y': case 'y':
      Serial.println(F("MODE: HYBRID"));
      targetMode = MODE_HYBRID;
      resetTTITracking();
      resetBatch();  // Reset batch system for new game
      startAutoMode();
      startCalibration();
      break;
    case 'M': case 'm':
      Serial.println(F("HYBRID MODE"));
      break;
    case 'S': case 's':
      Serial.println(F("STOP"));
      autoMode = false; systemEnabled = false; stopMotor();
      releaseCommitment(); pendingQueueSize = 0;
      resetBatch();  // Reset batch system
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
      {
        int lane = cmd - '1';
        Serial.print(F("->L")); Serial.println(lane + 1);
        desiredPosition = targetPositions[lane];
        autoMode = false; systemEnabled = true;
        releaseCommitment();
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
          Serial.print(F("CAP L")); Serial.print(lane + 1); Serial.print(F("=")); Serial.println(pos);
        }
      }
      break;
    case 'X': case 'x':
      // Show current calibration values
      Serial.println(F("=== CALIBRATION STATUS ==="));
      for (int i = 0; i < 4; i++) {
        Serial.print(F("L"));
        Serial.print(i + 1);
        Serial.print(F(": Range["));
        Serial.print(ProxRange[i][0]);
        Serial.print(F(","));
        Serial.print(ProxRange[i][1]);
        Serial.print(F("] Raw="));
        Serial.print((int)ProxSensors[i].currVal);
        Serial.print(F(" Dist="));
        Serial.println(zombieDistances[i], 2);
      }
      if (calibrationActive) {
        Serial.print(F("Calibrating... "));
        Serial.print((millis() - calibrationStartTime) / 1000);
        Serial.println(F("s"));
      }
      break;
    case 'P': case 'p': printCurrentSettings(); break;
    case 'W': case 'w': saveToEEPROM(); break;
    case 'R': case 'r': zombiesKilled = 0; zombiesMissed = 0; Serial.println(F("Reset")); break;
    case 'D': case 'd': verboseMode = !verboseMode; Serial.print(F("Verbose:")); Serial.println(verboseMode ? F("ON") : F("OFF")); break;
    case '?': printHelp(); break;
  }
}

//============================================
// START AUTO MODE
//============================================
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
  pendingQueueSize = 0;
  
}

//============================================
// RESET TTI
//============================================
void resetTTITracking() {
  for (int i = 0; i < 4; i++) {
    zombieVelocities[i] = 0;
    timeToImpact[i] = 99999;
    prevZombieDistances[i] = zombieDistances[i];
    velocityHistoryIndex[i] = 0;
    for (int j = 0; j < VEL_HISTORY_SIZE; j++) {
      velocityHistory[i][j] = 0;
    }
  }
  lastTTIUpdate = millis();
}

void recordHit(int lane) {
  if (lane < 0 || lane > 3) return;
  if (lane == lastHitLane) {
    consecutiveSameLane++;
  } else {
    consecutiveSameLane = 1;
  }
  lastHitLane = lane;
  lastHitTime = millis();
}

//============================================
// UPDATE TTI
// More robust TTI calculation with better filtering
//============================================
void updateTTI() {
  for (int i = 0; i < 4; i++) {
    float distChange = prevZombieDistances[i] - zombieDistances[i];
    float instantVel = distChange / TTI_UPDATE_INTERVAL;  // velocity in dist/ms
    
    // Store in history for median filtering
    velocityHistory[i][velocityHistoryIndex[i]] = instantVel;
    velocityHistoryIndex[i] = (velocityHistoryIndex[i] + 1) % VEL_HISTORY_SIZE;
    
    // Calculate median velocity (more robust than average)
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
    
    // Smooth the velocity (EMA filter)
    zombieVelocities[i] = velocityAlpha * zombieVelocities[i] + (1 - velocityAlpha) * medianVel;
    
    // Calculate TTI only if moving forward with meaningful velocity
    // Minimum velocity threshold: 0.00003 dist/ms = 30% distance in 10 seconds
    // This filters out noise and nearly-stopped targets
    if (zombieVelocities[i] > 0.00003 && ProxSensors[i].direction == FORWARD) {
      timeToImpact[i] = zombieDistances[i] / zombieVelocities[i];
    } else {
      // Not moving forward meaningfully - set very high TTI
      timeToImpact[i] = 99999;
    }
    
    timeToImpact[i] = constrain(timeToImpact[i], 0, 99999);
    prevZombieDistances[i] = zombieDistances[i];
  }
}

//============================================
// GET EFFECTIVE TTI
// Uses DYNAMIC travel time based on current motor position
// L2/L3 get BOOST (lower effective TTI = more urgent)
//============================================
float getEffectiveTTI(int lane) {
  if (timeToImpact[lane] >= 99999) return 99999;
  int dynamicTravel = getDynamicTravelTime(lane);
  float effectiveTTI = timeToImpact[lane] - dynamicTravel;
  // SHORT LANE BOOST for L2/L3 - they need earlier action!
  // By subtracting boost, their effective TTI becomes LOWER = MORE URGENT
  if (lane == 1 || lane == 2) effectiveTTI -= SHORT_LANE_BOOST;
  return max(effectiveTTI, 0.0f);
}

//============================================
// CAN REACH IN TIME
// Uses DYNAMIC travel time based on current motor position
//============================================
bool canReachInTime(int lane) {
  int dynamicTravel = getDynamicTravelTime(lane);
  return (timeToImpact[lane] > dynamicTravel + 50) || (timeToImpact[lane] >= 99999);
}

//============================================
// CALCULATE THREAT SCORE
//============================================
float calculateThreatScore(int lane) {
  float dist = zombieDistances[lane];
  float score = 0;
  
  if (ProxSensors[lane].direction == BACKWARD) return 0;
  if (ProxSensors[lane].direction == STOPPED) {
    if (laneStoppedTime[lane] > 0) {
      unsigned long stoppedDuration = millis() - laneStoppedTime[lane];
      if (stoppedDuration > STOPPED_TIMEOUT) return 0;
    }
    if (dist > 0.20) return 0;
    return (1.0 - dist) * 50 * LANE_PRIORITY[lane];
  }
  if (ProxSensors[lane].direction != FORWARD) return 0;
  float effectiveTTI = getEffectiveTTI(lane);
  float rawTTI = timeToImpact[lane];
  if (rawTTI < 99999 && effectiveTTI < 5000) {
    if (effectiveTTI < 200) {
      score = 1000 + (200 - effectiveTTI) * 5;
    } else if (effectiveTTI < 500) {
      score = 600 + (500 - effectiveTTI) * 1.3;
    } else if (effectiveTTI < 1000) {
      score = 300 + (1000 - effectiveTTI) * 0.6;
    } else if (effectiveTTI < 2000) {
      score = 100 + (2000 - effectiveTTI) * 0.2;
    } else {
      score = 50 + (5000 - effectiveTTI) * 0.02;
    }
  } else {
    if (dist < 0.10) {
      score = 800 + (0.10 - dist) * 2000;
    } else if (dist < 0.20) {
      score = 400 + (0.20 - dist) * 4000;
    } else if (dist < 0.35) {
      score = 150 + (0.35 - dist) * 1700;
    } else {
      score = 30 + (1.0 - dist) * 80;
    }
  }
  int dynamicTravel = getDynamicTravelTime(lane);
  score -= dynamicTravel * 0.15;
  if (!canReachInTime(lane) && score < 500) {
    score *= 0.5;
  }
  
  //============================================
  if (lane == 1 || lane == 2) {
    if (dist < 0.30) {
      score += 300 + (0.30 - dist) * 667;
    } else if (dist < 0.60) {
      score += 150 + (0.60 - dist) * 500;
    } else {
      score += 50 + (1.0 - dist) * 250;
    }
    if (dist < SHORT_LANE_CRITICAL_DISTANCE) {
      score += (SHORT_LANE_CRITICAL_DISTANCE - dist) * 5000;
    } else if (dist < 0.35) {
      score += (0.35 - dist) * 2000;
    } else if (dist < 0.55) {
      score += (0.55 - dist) * 500;
    } else if (dist < 0.80) {
      score += (0.80 - dist) * 300;
    }
    if (dist > 0.70) score += 300;
  } else {
    if (dist < LONG_LANE_CRITICAL_DISTANCE) {
      score += (LONG_LANE_CRITICAL_DISTANCE - dist) * 3000;
    } else if (dist < 0.30) {
      score += (0.30 - dist) * 1500;
    } else if (dist < 0.45) {
      score += (0.45 - dist) * 400;
    }
  }
  score *= LANE_PRIORITY[lane];
  if (lane == 1 || lane == 2) {
    for (int otherLane = 0; otherLane < 4; otherLane++) {
      if (otherLane == lane) continue;
      if (laneIsLong[otherLane] && ProxSensors[otherLane].direction == FORWARD) {
        float otherDist = zombieDistances[otherLane];
        float distanceGap = abs(dist - otherDist);
        bool otherIsCritical = laneIsCritical[otherLane];
        if (!otherIsCritical) {
          score += 2000;
          break;
        } else if (distanceGap < 0.15) {
          score += 800;
          break;
        }
      }
    }
  }
  
  return max(score, 0.0f);
}

//============================================
// UPDATE SENSORS
// More responsive direction detection for bounce-back
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
      // So: 1.0 = at impact (urgent!), 0.0 = at start
      float normalized = (ProxSensors[i].currVal - ProxRange[i][1]) / range;
      currentDistance = constrain(normalized, 0.0f, 1.0f);
      
      // CRITICAL: Make L2/L3 appear 12% shorter to prioritize them earlier
      // This makes them appear closer than they really are, triggering priority sooner
      if (i == 1 || i == 2) {  // L2 or L3 (short lanes)
        currentDistance = currentDistance * 0.88f;  // Make appear 12% closer (shorter)
        currentDistance = constrain(currentDistance, 0.0f, 1.0f);
      }
    }
    zombieDistances[i] = currentDistance;

    // Derive sensor-specific noise limit rather than sharing across lanes
    int sensorNoiseLimit = (ProxSensors[i].currVal >= noiseThreshold) ? upperNoiseLimit : lowerNoiseLimit;

    float change = ProxSensors[i].currVal - ProxSensors[i].prevVal;

    // IMPROVED: Use MAGNITUDE of change to detect fast movement
    float changeMagnitude = abs(change);
    bool fastMovement = changeMagnitude > sensorNoiseLimit * 2;  // Moving fast if >2x noise threshold

    if (changeMagnitude < sensorNoiseLimit) {
      // No significant change detected
      if (now - ProxSensors[i].prevChangeTime >= stopTimeout) {
        // Track when lane becomes STOPPED
        if (ProxSensors[i].direction != STOPPED) {
          laneStoppedTime[i] = now;  // Record when it became STOPPED
        }
        ProxSensors[i].direction = STOPPED;
        ProxSensors[i].forwardCount = 0;
        ProxSensors[i].backwardCount = 0;
      }
    } else if (change < 0) {
      // Moving forward (sensor value decreasing = getting closer)
      // Lane is moving forward - clear stopped time
      if (ProxSensors[i].direction == STOPPED) {
        laneStoppedTime[i] = 0;  // Clear stopped time when lane starts moving
      }
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      if (ProxSensors[i].forwardCount >= 2) {
        ProxSensors[i].direction = FORWARD;
      }
    } else {
      // Moving backward (sensor value increasing = getting farther)
      // Lane is moving backward - clear stopped time
      if (ProxSensors[i].direction == STOPPED) {
        laneStoppedTime[i] = 0;  // Clear stopped time when lane starts moving
      }
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      
      int backwardThreshold = 2;
      if (fastMovement) {
        backwardThreshold = 2;
      } else if (currentDistance < 0.15f) {
        backwardThreshold = 2;
      } else if (currentDistance < 0.30f) {
        backwardThreshold = 3;
      } else if (currentDistance < 0.50f) {
        backwardThreshold = 2;
      }
      
      if (ProxSensors[i].backwardCount >= backwardThreshold) {
        ProxSensors[i].direction = BACKWARD;
      }
    }
    
    // NOISE FILTER: Ignore targets below 15% that aren't clearly forward
    // This prevents false locks on sensor noise
    if (currentDistance > 0.85f && ProxSensors[i].direction == FORWARD) {
      // Very low signal - require stronger evidence
      if (ProxSensors[i].forwardCount < 3) {
        ProxSensors[i].direction = STOPPED;
      }
    }
  }
}

//============================================
// VELOCITY & PID
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
  // OPTIMIZATION: Use cached encoder position
  long currentPos = cachedEncoderPos;
  float positionError = desiredPosition - currentPos;
  
  int effectiveBand = (activeTargetIndex == 3) ? 12 : TARGET_BAND;
  int effectiveVelThreshold = (activeTargetIndex == 3) ? 50 : 80;
  
  if (abs(positionError) <= effectiveBand && abs(motorVelocity) < effectiveVelThreshold) {
    stopMotor(); errorIntegral = 0; return;
  }
  
  errorIntegral += positionError * (float)executionDuration / 1000000.0;
  errorIntegral = constrain(errorIntegral, -1000, 1000);
  
  float velocityError = 0 - motorVelocity;
  float voltage = KP * positionError + KI * errorIntegral + KD * velocityError;
  
  if (positionError < -5) voltage -= FRICTION_LEFT;
  else if (positionError > 5) voltage += FRICTION_RIGHT;
  
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
  int pwm = constrain((int)(abs(voltage) * 255.0 / SUPPLY_VOLTAGE), 0, 255);
  analogWrite(MOTOR_ENA, pwm);
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
  
  // Move toward left limit switch SOFTLY at 2.0V
  while (digitalRead(LIMIT_LEFT) == LOW) { 
    setMotorVoltage(2.0); 
    delay(10); 
  }
  
  // Hold against limit switch gently at 2.0V to eliminate bounce
  Serial.println(F("Holding at limit..."));
  unsigned long holdStart = millis();
  const unsigned long HOLD_TIME = 500;  // Hold for 500ms
  
  while (millis() - holdStart < HOLD_TIME) {
    setMotorVoltage(2.0);
    delay(10);
  }
  
  // Now set encoder zero while held firmly against switch
  encoder.write(0);
  errorIntegral = 0;
  
  // Continue holding briefly to confirm position
  setMotorVoltage(2.0);
  delay(100);
  
  stopMotor();
  delay(50);
  Serial.println(F("Homed! Pos=0"));
}

void printScore() {
  Serial.print(F("Score: K=")); Serial.print(zombiesKilled);
  Serial.print(F(" M=")); Serial.println(zombiesMissed);
}

//============================================
// PRINT STATUS
//============================================
void printStatus() {
  long pos = encoder.read();
  
  Serial.print(pos);
  Serial.print(F(" D:"));
  Serial.print(desiredPosition);
  Serial.print(F(" T:"));
  Serial.print(activeTargetIndex + 1);
  
  if (isCommitted) {
    Serial.print(F(" [L"));
    Serial.print(committedLane + 1);
    Serial.print(F("]|"));
  } else {
    Serial.print(F(" [-]|"));
  }
  
  for (int i = 0; i < 4; i++) {
    int pct = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(pct);
    
    if (ProxSensors[i].direction == FORWARD) Serial.print(F("F"));
    else if (ProxSensors[i].direction == BACKWARD) Serial.print(F("B"));
    else Serial.print(F("-"));
    
    if (timeToImpact[i] < 9999) {
      Serial.print(F("("));
      Serial.print((int)(timeToImpact[i] / 100));
      Serial.print(F(")"));
    }
    Serial.print(F(" "));
  }
  
  if (pendingQueueSize > 0) {
    Serial.print(F("Q:"));
    Serial.print(pendingQueueSize);
    Serial.print(F(" "));
  }
  
  if (calibrationActive) {
    Serial.print(F("CAL:"));
    Serial.print((millis() - calibrationStartTime) / 1000);
    Serial.print(F("s "));
  }
  
  Serial.print(F("K:"));
  Serial.print(zombiesKilled);
  Serial.print(F(" M:"));
  Serial.println(zombiesMissed);
}
