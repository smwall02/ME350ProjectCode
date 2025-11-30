// ME350 Zombie Defense - HYBRID MODE v9
// EARLY ENGAGEMENT: Hit zombies at lane start
// DYNAMIC CALIBRATION: Auto-find lane ranges first few seconds

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
  MODE_DISTANCE = 1,
  MODE_TTI = 2,
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

// Get dynamic travel time from current position to target lane
int getDynamicTravelTime(int targetLane) {
  long currentPos = encoder.read();
  
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

// SHORT_LANE_BOOST - L2/L3 need EARLIER engagement, not penalty
// This is added to their effective TTI calculation to trigger earlier action
const int SHORT_LANE_BOOST = 150;  // Give L2/L3 150ms "head start" - balanced

//============================================
// EARLY ENGAGEMENT THRESHOLDS
// Now we engage zombies as soon as they appear!
//============================================
const float EARLY_ENGAGE_THRESHOLD = 0.85;   // Engage when zombie at 85% (just appeared)
const float MIN_ENGAGE_THRESHOLD = 0.10;     // Don't engage if below 10% for normal targeting

// Override thresholds - LANE SPECIFIC!
// L2/L3 are SHORT lanes - override must trigger EARLIER (at higher distance %)
const float OVERRIDE_THRESHOLD[4] = {0.08, 0.12, 0.12, 0.08};  // TIGHT thresholds - only TRUE emergencies (92%/88%)
const float ABSOLUTE_OVERRIDE_DISTANCE = 0.10;  // Fallback for L1/L4

// Retreat detection - Lane 4 needs more tolerance due to longer travel
const float RETREAT_CONFIRMED_DISTANCE = 0.35;
const float ZOMBIE_GONE_DISTANCE = 0.65;        // Raised from 0.60
const float L4_GONE_DISTANCE = 0.75;            // Lane 4 specific - even more tolerant

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

// Anti-consecutive-lane tracking
int lastHitLane = -1;                    // Last lane we successfully hit
unsigned long lastHitTime = 0;           // When we hit it
int consecutiveSameLane = 0;             // How many times we've hit same lane consecutively
const int MAX_CONSECUTIVE_SAME = 2;      // Max times to hit same lane before forcing rotation

// Batch statistics for debugging
int batchesCompleted = 0;
int batchesReset = 0;

//============================================
// DWELL TRACKING
//============================================
unsigned long arrivalTime = 0;
float peakZombieDistance = 1.0;
float arrivalZombieDistance = 1.0;

const unsigned long MIN_DWELL_TIME = 250;       // FASTER - reduced from 400

//============================================
// ANALYSIS MODE - Lane Priority Tracking
//============================================
bool analysisMode = false;           // Toggle via 'A' command
int targetSequence[30];              // Stores sequence of targeted lanes (expanded to 30)
int sequenceIndex = 0;               // Current position in sequence
unsigned long cycleStartTime = 0;    // When current cycle started
int cycleKillCount = 0;              // Kills in current cycle
int cycleHitCount[4] = {0,0,0,0};    // Hits per lane in current cycle
int cycleMissCount[4] = {0,0,0,0};   // Misses per lane in current cycle (reached wall)
float lanePriorityScores[4];         // Calculated priority scores at decision time
int priorityOrder[4];                // Lane order by priority (highest first)
const unsigned long NORMAL_DWELL_TIME = 500;   // FASTER - reduced from 800
const unsigned long MAX_DWELL_TIME = 1200;     // FASTER - reduced from 2000
const unsigned long L4_DWELL_TIME = 1800;      // FASTER - reduced from 2500

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
void commitToTarget(int lane);
void releaseCommitment();
bool shouldOverride(int newLane);
void addToPendingQueue(int lane);
int getBestTarget();
int getNextFromQueue();
void updatePendingQueue();
void updateCalibration();
void applyCalibration();
// Analysis mode functions
void resetAnalysisCycle();
void recordTargetSelection(int lane);
void recordHit(int lane);
void recordMiss(int lane);
void printAnalysisSummary();

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
  Serial.println(F("G/T/Y - Start D/T/H"));
  Serial.println(F("S - Stop, H - Home"));
  Serial.println(F("M - Cycle modes"));
  Serial.println(F("1-4 - Manual lane"));
  Serial.println(F("C1-C4 - Capture pos"));
  Serial.println(F("X - Show calibration"));
  Serial.println(F("A - Analysis mode"));
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
  
  // For normal targeting, reject if outside range
  // But for emergencies (very close + forward), allow it
  bool isEmergency = (dist < ABSOLUTE_OVERRIDE_DISTANCE && ProxSensors[lane].direction == FORWARD);
  
  if (!isEmergency) {
    // Normal targeting rules
    if (dist > EARLY_ENGAGE_THRESHOLD || dist < MIN_ENGAGE_THRESHOLD) {
      return;
    }
    // Reject if not moving forward (unless very close emergency)
    if (ProxSensors[lane].direction != FORWARD) {
      return;
    }
  }
  
  isCommitted = true;
  committedLane = lane;
  commitStartTime = millis();
  commitStartDistance = zombieDistances[lane];
  lastCommitTime = millis();
  
  previousTargetIndex = activeTargetIndex;
  activeTargetIndex = lane;
  desiredPosition = targetPositions[lane];
  targetLockTime = millis();
  WAIT_POS = false;
  
  // Analysis mode: record target selection
  recordTargetSelection(lane);
  
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
  
  // CRITICAL: Only override for FORWARD moving targets
  if (ProxSensors[newLane].direction != FORWARD) return false;
  
  // How close is the new lane to wall? (lower = closer = more urgent)
  float newDist = zombieDistances[newLane];
  float committedDist = zombieDistances[committedLane];
  
  // Calculate how close we are to our target position
  long currentPos = encoder.read();
  int distToTarget = abs(currentPos - targetPositions[committedLane]);
  bool atTarget = (distToTarget < 50);        // Already at target
  bool nearingTarget = (distToTarget < 300);  // Within 300 ticks
  
  // RULE 1: If we're AT the target (dwelling), only override for EXTREME emergency (<5%)
  if (atTarget && ProxSensors[committedLane].direction == FORWARD) {
    return newDist < 0.05;  // 95%+ on display
  }
  
  // RULE 2: If we're nearing target and committed lane still needs attention,
  // only override if new lane is MUCH more critical
  if (nearingTarget && committedDist < 0.40 && 
      ProxSensors[committedLane].direction == FORWARD) {
    // New must be at least 2x closer than committed to justify switch
    return newDist < committedDist * 0.4;
  }
  
  // RULE 3: General case - only override if new target is at TRUE emergency level
  // AND is significantly more urgent than committed
  float threshold = OVERRIDE_THRESHOLD[newLane];
  if (newDist < threshold) {
    // If committed is still a threat, new must be MUCH more critical
    if (committedDist < 0.25 && ProxSensors[committedLane].direction == FORWARD) {
      return newDist < committedDist * 0.5;
    }
    return true;
  }
  
  // Default: DO NOT override - let batch system handle it
  return false;
}

//============================================
// ADD TO PENDING QUEUE
//============================================
void addToPendingQueue(int lane) {
  for (int i = 0; i < pendingQueueSize; i++) {
    if (pendingQueue[i] == lane) return;
  }
  
  if (pendingQueueSize < 4) {
    pendingQueue[pendingQueueSize++] = lane;
  }
}

//============================================
// GET BEST TARGET (UNIFIED)
// Check ALL lanes AND queue, return the best one
// This replaces getNextFromQueue() for HIT transitions
//============================================
int getBestTarget() {
  updatePendingQueue();
  
  int bestLane = -1;
  float bestScore = 0;
  
  // Check ALL lanes for the best forward-moving target
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction != FORWARD) continue;
    
    float dist = zombieDistances[i];
    if (dist < MIN_ENGAGE_THRESHOLD || dist > EARLY_ENGAGE_THRESHOLD) continue;
    
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
  // Clear existing batch
  for (int i = 0; i < 4; i++) {
    targetBatch[i] = -1;
  }
  batchSize = 0;
  batchIndex = 0;
  batchStartTime = millis();
  
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
    lanes[i].isActive = (ProxSensors[i].direction == FORWARD && 
                         lanes[i].distance < EARLY_ENGAGE_THRESHOLD &&
                         lanes[i].distance > MIN_ENGAGE_THRESHOLD);
    
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
      if (analysisMode) {
        Serial.print(F("[BATCH] Skipping L"));
        Serial.print(activeLane + 1);
        Serial.println(F(" - hit too many times, waiting for others"));
      }
      return;
    }
  }
  
  // Sort lanes by effective TTI (lowest first = most urgent)
  // Simple bubble sort for 4 elements
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (lanes[j].effectiveTTI < lanes[i].effectiveTTI) {
        LaneInfo temp = lanes[i];
        lanes[i] = lanes[j];
        lanes[j] = temp;
      }
    }
  }
  
  // Build batch: Add each active lane ONCE in TTI order
  // NO DUPLICATES - each lane appears at most once per batch
  // LIMITED TO MAX_BATCH_SIZE (3) to allow time to reach all targets
  for (int i = 0; i < 4; i++) {
    if (lanes[i].isActive && batchSize < MAX_BATCH_SIZE) {
      targetBatch[batchSize++] = lanes[i].lane;
    }
  }
  
  batchActive = (batchSize > 0);
  
  // Debug output
  if (analysisMode && batchActive) {
    Serial.print(F("[BATCH] New batch: "));
    for (int i = 0; i < batchSize; i++) {
      Serial.print(F("L"));
      Serial.print(targetBatch[i] + 1);
      if (i < batchSize - 1) Serial.print(F("->"));
    }
    Serial.println();
  }
}

// Get the next target from the current batch
int getNextBatchTarget() {
  // If batch is empty or exhausted, calculate new one
  if (!batchActive || batchIndex >= batchSize) {
    calculateNewBatch();
    if (!batchActive) return -1;
  }
  
  // Find next valid target in batch
  while (batchIndex < batchSize) {
    int lane = targetBatch[batchIndex];
    
    // Check if this target is still valid (forward-moving, in range)
    if (lane >= 0 && lane < 4 &&
        ProxSensors[lane].direction == FORWARD &&
        zombieDistances[lane] < EARLY_ENGAGE_THRESHOLD &&
        zombieDistances[lane] > MIN_ENGAGE_THRESHOLD) {
      batchIndex++;  // Move to next for next call
      return lane;
    }
    
    // Target no longer valid, skip it
    batchIndex++;
  }
  
  // Batch exhausted, calculate new one
  calculateNewBatch();
  if (!batchActive) return -1;
  
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
  batchIndex = 0;
  batchSize = 0;
  batchesReset++;
  
  if (analysisMode) {
    Serial.println(F("[BATCH] Reset - recalculating"));
  }
}

// Mark current batch target as complete (hit registered)
void advanceBatch() {
  // Already advanced in getNextBatchTarget, but check if batch complete
  if (batchIndex >= batchSize) {
    batchesCompleted++;
    batchActive = false;
    
    if (analysisMode) {
      Serial.print(F("[BATCH] Complete #"));
      Serial.println(batchesCompleted);
    }
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
    // Use lane-specific thresholds (L2/L3 at 15%, L1/L4 at 10%)
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
      if (ProxSensors[lane].direction == FORWARD && 
          zombieDistances[lane] < EARLY_ENGAGE_THRESHOLD) {
        pendingQueue[newSize++] = lane;
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
  computeVelocity();
  updateSensors();
  
  for (int i = 0; i < 4; i++) {
    const float range = (float)(ProxRange[i][0] - ProxRange[i][1]);
    if (range != 0.0f) {
      zombieDistances[i] = (ProxSensors[i].currVal - ProxRange[i][1]) / range;
    } else {
      // Defensive fallback to avoid divide-by-zero if calibration data is bad
      zombieDistances[i] = 1.0f;
    }
    zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);
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
    // Longer cooldown between overrides to prevent rapid switching
    bool canCheckOverride = (timeSinceOverride >= 600) && (millis() - overrideCheckTime >= 80);
    
    if (canCheckOverride) {
      overrideCheckTime = millis();
      
      for (int i = 0; i < 4; i++) {
        if (i == committedLane) continue;
        if (ProxSensors[i].direction != FORWARD) continue;
        // Longer anti-return period
        if (i == lastOverrideLane && timeSinceOverride < 1200) continue;
        
        // Use TIGHT OVERRIDE_THRESHOLD - only TRUE emergencies
        if (zombieDistances[i] < OVERRIDE_THRESHOLD[i]) {
          float score = calculateThreatScore(i);
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
// CRITICAL: Always choose highest threat score, whether queued or new
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
    if (dist < MIN_ENGAGE_THRESHOLD || dist > EARLY_ENGAGE_THRESHOLD) continue;
    
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
    
    // Skip if zombie is outside valid engagement range
    if (dist < MIN_ENGAGE_THRESHOLD || dist > EARLY_ENGAGE_THRESHOLD) continue;
    
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
//============================================
void chooseAndCommitTargetFromBatch() {
  // Get next target from batch (will calculate new batch if needed)
  int nextLane = getNextBatchTarget();
  
  if (nextLane >= 0) {
    // Print selection in analysis mode
    if (analysisMode) {
      Serial.print(F("[PRIORITY] "));
      // Show all lane scores
      float scores[4];
      int order[4] = {0, 1, 2, 3};
      for (int i = 0; i < 4; i++) {
        scores[i] = calculateThreatScore(i);
      }
      // Sort by score
      for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
          if (scores[order[j]] > scores[order[i]]) {
            int temp = order[i];
            order[i] = order[j];
            order[j] = temp;
          }
        }
      }
      for (int i = 0; i < 4; i++) {
        Serial.print(F("L"));
        Serial.print(order[i] + 1);
        Serial.print(F(":"));
        Serial.print((int)scores[order[i]]);
        if (i < 3) Serial.print(F(" > "));
      }
      Serial.print(F(" | Selected:L"));
      Serial.println(nextLane + 1);
    }
    
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
  long currentPos = encoder.read();
  
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    arrivalTime = millis();
    if (activeTargetIndex >= 0) {
      peakZombieDistance = zombieDistances[activeTargetIndex];
      arrivalZombieDistance = zombieDistances[activeTargetIndex];
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
// With improved hit detection for bounce-back
//============================================
void dwellAtTarget() {
  if (activeTargetIndex < 0) {
    releaseCommitment();
    state = CHOOSE_TARGET;
    return;
  }
  
  unsigned long dwellTime = millis() - arrivalTime;
  float currentDist = zombieDistances[activeTargetIndex];
  int currentDir = ProxSensors[activeTargetIndex].direction;
  
  unsigned long maxDwell = (activeTargetIndex == 3) ? L4_DWELL_TIME : MAX_DWELL_TIME;
  
  if (currentDist < peakZombieDistance) {
    peakZombieDistance = currentDist;
  }
  
  //--------------------------------------------
  // EXIT CONDITION 1: BACKWARD DETECTED = HIT
  // Requires minimum dwell time to avoid false positives
  //--------------------------------------------
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
      stoppedStartTime = 0;  // Reset stopped tracker
      releaseCommitment();
      
      // Get next target from batch
      int next = getNextBatchTarget();
      if (next >= 0 && ProxSensors[next].direction == FORWARD) {
        commitToTarget(next);
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      } else {
        state = CHOOSE_TARGET;
      }
      return;
    }
  } else {
    backwardStartTime = 0;
  }
  
  //--------------------------------------------
  // EXIT CONDITION 1.5: PEAK RETREAT DETECTION
  // If zombie got very close and is now farther, count as hit
  // Even if direction isn't "BACKWARD" yet (sensor lag)
  //--------------------------------------------
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
      
      stoppedStartTime = 0;  // Reset stopped tracker
      releaseCommitment();
      int next = getNextBatchTarget();
      if (next >= 0 && ProxSensors[next].direction == FORWARD) {
        commitToTarget(next);
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      } else {
        state = CHOOSE_TARGET;
      }
      return;
    }
  }
  
  //--------------------------------------------
  // EXIT CONDITION 2: ZOMBIE RETREATED FAR
  // Only valid if we dwelt long enough
  //--------------------------------------------
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
    
    releaseCommitment();
    stoppedStartTime = 0;  // Reset stopped tracker
    int next = getNextBatchTarget();
    if (next >= 0 && ProxSensors[next].direction == FORWARD) {
      commitToTarget(next);
      if (isCommitted) state = MOVE_TO_TARGET;
      else state = CHOOSE_TARGET;
    } else {
      state = CHOOSE_TARGET;
    }
    return;
  }
  
  //--------------------------------------------
  // EXIT CONDITION 2.5: ZOMBIE STOPPED - FAST EXIT
  // If zombie hasn't moved significantly for STOPPED_CONFIRM_TIME, move on
  // This catches stalled zombies that aren't clearly hit or retreating
  //--------------------------------------------
  // Check for STOPPED direction OR very small velocity (pseudo-stopped)
  bool isEffectivelyStopped = (currentDir == STOPPED) || 
                               (abs(zombieVelocities[activeTargetIndex]) < 0.0001);
  
  if (isEffectivelyStopped && dwellTime >= 150) {  // Reduced from 200ms
    if (stoppedStartTime == 0) {
      stoppedStartTime = millis();
      stoppedStartDistance = currentDist;
    } else if (millis() - stoppedStartTime >= 300) {  // Reduced from 400ms for faster exit
      // Check if distance barely changed - zombie truly stalled
      float distChange = abs(currentDist - stoppedStartDistance);
      if (distChange < 0.08) {  // Increased from 5% to 8% - more tolerant
        // If zombie was close, probably a hit
        if (peakZombieDistance < 0.25) {  // Increased threshold - more generous hit credit
          zombiesKilled++;
          recordHit(activeTargetIndex);
          Serial.print(F("HIT L"));
          Serial.print(activeTargetIndex + 1);
          Serial.print(F(" (stop) ["));
          Serial.print(zombiesKilled);
          Serial.println(F("]"));
        } else {
          Serial.print(F("STOP L"));
          Serial.println(activeTargetIndex + 1);
        }
        
        releaseCommitment();
        stoppedStartTime = 0;
        int next = getNextBatchTarget();
        if (next >= 0 && ProxSensors[next].direction == FORWARD) {
          commitToTarget(next);
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
        } else {
          state = CHOOSE_TARGET;
        }
        return;
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
  // Must have dwelt minimum time - prevents false GONE on arrival
  // Lane 4 uses higher threshold due to longer travel time
  //--------------------------------------------
  float goneThreshold = (activeTargetIndex == 3) ? L4_GONE_DISTANCE : ZOMBIE_GONE_DISTANCE;
  if (dwellTime >= MIN_DWELL_TIME && currentDist > goneThreshold) {
    Serial.print(F("GONE L"));
    Serial.println(activeTargetIndex + 1);
    
    releaseCommitment();
    int next = getNextBatchTarget();
    if (next >= 0 && ProxSensors[next].direction == FORWARD) {
      commitToTarget(next);
      if (isCommitted) state = MOVE_TO_TARGET;
      else state = CHOOSE_TARGET;
    } else {
      state = CHOOSE_TARGET;
    }
    return;
  }
  
  //--------------------------------------------
  // EXIT CONDITION 4: TIMEOUT
  //--------------------------------------------
  if (dwellTime >= maxDwell) {
    if (currentDir == FORWARD && currentDist < arrivalZombieDistance - 0.03) {
      zombiesMissed++;
      recordMiss(activeTargetIndex);  // Analysis mode tracking
      Serial.print(F("MISS L"));
      Serial.print(activeTargetIndex + 1);
      Serial.print(F(" ["));
      Serial.print(zombiesMissed);
      Serial.println(F("]"));
    } else if (currentDir == STOPPED) {
      // STOPPED at very close range = likely hit wall = miss
      if (currentDist < 0.08) {
        recordMiss(activeTargetIndex);  // Analysis mode tracking
      }
      Serial.print(F("STOP L"));
      Serial.println(activeTargetIndex + 1);
    } else {
      Serial.print(F("TIME L"));
      Serial.println(activeTargetIndex + 1);
    }
    
    releaseCommitment();
    int next = getNextBatchTarget();
    if (next >= 0 && ProxSensors[next].direction == FORWARD) {
      commitToTarget(next);
      if (isCommitted) state = MOVE_TO_TARGET;
      else state = CHOOSE_TARGET;
    } else {
      state = CHOOSE_TARGET;
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
    case 'G': case 'g':
      Serial.println(F("MODE: DISTANCE"));
      targetMode = MODE_DISTANCE;
      startAutoMode();
      startCalibration();  // Auto-calibrate on start
      break;
    case 'T': case 't':
      Serial.println(F("MODE: TTI"));
      targetMode = MODE_TTI;
      resetTTITracking();
      resetBatch();  // Reset batch system for new game
      startAutoMode();
      startCalibration();
      break;
    case 'Y': case 'y':
      Serial.println(F("MODE: HYBRID"));
      targetMode = MODE_HYBRID;
      resetTTITracking();
      resetBatch();  // Reset batch system for new game
      startAutoMode();
      startCalibration();
      break;
    case 'M': case 'm':
      if (targetMode == MODE_DISTANCE) { targetMode = MODE_TTI; resetTTITracking(); Serial.println(F("->TTI")); }
      else if (targetMode == MODE_TTI) { targetMode = MODE_HYBRID; Serial.println(F("->HYBRID")); }
      else { targetMode = MODE_DISTANCE; Serial.println(F("->DISTANCE")); }
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
    case 'A': case 'a': 
      analysisMode = !analysisMode; 
      if (analysisMode) {
        resetAnalysisCycle();
        Serial.println(F("ANALYSIS MODE: ON"));
        Serial.println(F("Tracking: Lane sequence, priority scores, hits/misses"));
      } else {
        printAnalysisSummary();
        Serial.println(F("ANALYSIS MODE: OFF"));
      }
      break;
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
  
  // Reset analysis tracking for new run
  if (analysisMode) {
    resetAnalysisCycle();
    Serial.println(F("[Analysis: New cycle started]"));
  }
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

//============================================
// ANALYSIS MODE FUNCTIONS
//============================================
void resetAnalysisCycle() {
  sequenceIndex = 0;
  cycleStartTime = millis();
  cycleKillCount = 0;
  for (int i = 0; i < 4; i++) {
    cycleHitCount[i] = 0;
    cycleMissCount[i] = 0;
  }
  for (int i = 0; i < 30; i++) {
    targetSequence[i] = -1;
  }
}

void recordTargetSelection(int lane) {
  if (!analysisMode || lane < 0) return;
  
  // Record in sequence (expanded to 30)
  if (sequenceIndex < 30) {
    targetSequence[sequenceIndex++] = lane;
  }
  
  // Calculate and store priority scores for all lanes at this moment
  for (int i = 0; i < 4; i++) {
    lanePriorityScores[i] = calculateThreatScore(i);
  }
  
  // Sort lanes by priority (highest first)
  for (int i = 0; i < 4; i++) priorityOrder[i] = i;
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (lanePriorityScores[priorityOrder[j]] > lanePriorityScores[priorityOrder[i]]) {
        int temp = priorityOrder[i];
        priorityOrder[i] = priorityOrder[j];
        priorityOrder[j] = temp;
      }
    }
  }
  // Note: PRIORITY output now handled in chooseAndCommitTargetFromBatch to avoid duplicates
}

void recordHit(int lane) {
  if (lane < 0 || lane > 3) return;
  
  // Track consecutive same-lane hits for anti-repetition
  if (lane == lastHitLane) {
    consecutiveSameLane++;
  } else {
    consecutiveSameLane = 1;  // Reset to 1 (this is the first hit on this lane)
  }
  lastHitLane = lane;
  lastHitTime = millis();
  
  // Analysis mode tracking
  if (!analysisMode) return;
  cycleHitCount[lane]++;
  cycleKillCount++;
}

void recordMiss(int lane) {
  if (!analysisMode || lane < 0) return;
  cycleMissCount[lane]++;
  
  // Print warning when L2/L3 gets missed
  if (lane == 1 || lane == 2) {
    Serial.print(F("*** L"));
    Serial.print(lane + 1);
    Serial.println(F(" WALL IMPACT - SHORT LANE MISS ***"));
  }
}

void printAnalysisSummary() {
  // Stats were inaccurate, just print basic info
  Serial.println(F("\nANALYSIS MODE: OFF"));
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
// CRITICAL: Use TTI as PRIMARY factor, distance as SECONDARY
// BACKWARD = NO THREAT (period - no exceptions)
// This scoring is used for all modes - they differ only in engagement timing
//============================================
float calculateThreatScore(int lane) {
  float dist = zombieDistances[lane];
  float score = 0;
  
  // Backward-moving targets are NOT a threat - they're retreating
  // NO EXCEPTIONS - we were over-prioritizing L2/L3 with bounce-back logic
  if (ProxSensors[lane].direction == BACKWARD) {
    return 0;
  }
  
  // Stopped targets are low priority unless very close
  if (ProxSensors[lane].direction == STOPPED) {
    if (dist > 0.20) return 0;  // Far and stopped = ignore
    // Close and stopped = minor threat (it might start moving any moment)
    return (1.0 - dist) * 50 * LANE_PRIORITY[lane];
  }
  
  // Only forward-moving targets are real threats
  if (ProxSensors[lane].direction != FORWARD) return 0;
  
  // Get effective TTI (time until zombie hits wall, accounting for travel time)
  float effectiveTTI = getEffectiveTTI(lane);
  float rawTTI = timeToImpact[lane];
  
  //============================================
  // PRIMARY SCORING: Based on effective TTI
  // Lower TTI = Higher score (more urgent)
  //============================================
  if (rawTTI < 99999 && effectiveTTI < 5000) {
    // We have valid TTI data - use it as primary factor
    if (effectiveTTI < 200) {
      // CRITICAL: Can't possibly reach in time or barely can
      // This is highest priority - zombie is about to impact!
      score = 1000 + (200 - effectiveTTI) * 5;  // 1000-2000 range
    } else if (effectiveTTI < 500) {
      // EMERGENCY: Very little time to spare
      score = 600 + (500 - effectiveTTI) * 1.3;  // 600-1000 range
    } else if (effectiveTTI < 1000) {
      // URGENT: Need to act soon
      score = 300 + (1000 - effectiveTTI) * 0.6;  // 300-600 range
    } else if (effectiveTTI < 2000) {
      // CLOSE: Approaching
      score = 100 + (2000 - effectiveTTI) * 0.2;  // 100-300 range
    } else {
      // EARLY: Can engage early
      score = 50 + (5000 - effectiveTTI) * 0.02;  // 50-100 range
    }
  } else {
    //============================================
    // FALLBACK: No valid TTI - use distance only
    // This happens when zombie just appeared or is moving slowly
    //============================================
    if (dist < 0.10) {
      // EMERGENCY by distance
      score = 800 + (0.10 - dist) * 2000;  // 800-1000 range
    } else if (dist < 0.20) {
      // URGENT by distance
      score = 400 + (0.20 - dist) * 4000;  // 400-800 range
    } else if (dist < 0.35) {
      // CLOSE by distance
      score = 150 + (0.35 - dist) * 1700;  // 150-400 range
    } else {
      // EARLY by distance - prefer ones that just entered
      score = 30 + (1.0 - dist) * 80;  // 30-100 range
    }
  }
  
  //============================================
  // SECONDARY FACTORS
  //============================================
  
  // Travel time penalty - DYNAMIC based on current position!
  // This ensures we prefer lanes we can actually reach quickly
  int dynamicTravel = getDynamicTravelTime(lane);
  score -= dynamicTravel * 0.15;  // Increased penalty weight for dynamic travel
  
  // Small bonus for targets we can definitely reach
  // Discourages chasing impossible targets
  if (!canReachInTime(lane) && score < 500) {
    score *= 0.5;  // Halve score if we probably can't reach it
  }
  
  // LANE PRIORITY MULTIPLIER (L4 gets 1.2x boost)
  score *= LANE_PRIORITY[lane];
  
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

    // Derive sensor-specific noise limit rather than sharing across lanes
    int sensorNoiseLimit = (ProxSensors[i].currVal >= noiseThreshold) ? upperNoiseLimit : lowerNoiseLimit;

    float change = ProxSensors[i].currVal - ProxSensors[i].prevVal;

    // IMPROVED: Use MAGNITUDE of change to detect fast movement
    float changeMagnitude = abs(change);
    bool fastMovement = changeMagnitude > sensorNoiseLimit * 2;  // Moving fast if >2x noise threshold

    if (changeMagnitude < sensorNoiseLimit) {
      // No significant change detected
      if (now - ProxSensors[i].prevChangeTime >= stopTimeout) {
        ProxSensors[i].direction = STOPPED;
        ProxSensors[i].forwardCount = 0;
        ProxSensors[i].backwardCount = 0;
      }
    } else if (change < 0) {
      // Moving forward (sensor value decreasing = getting closer)
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      if (ProxSensors[i].forwardCount >= 2) {
        ProxSensors[i].direction = FORWARD;
      }
    } else {
      // Moving backward (sensor value increasing = getting farther)
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      
      // IMPROVED BACKWARD DETECTION:
      // 1. If moving FAST backward, detect immediately (2 samples)
      // 2. If at wall (dist < 15%), assume backward after bounce
      // 3. Otherwise use distance-based threshold
      int backwardThreshold = 2;
      
      if (fastMovement) {
        // Fast backward movement - detect quickly
        backwardThreshold = 2;
      } else if (zombieDistances[i] < 0.15) {
        // Very close to wall - likely bouncing back
        backwardThreshold = 2;
      } else if (zombieDistances[i] < 0.30) {
        backwardThreshold = 3;
      } else if (zombieDistances[i] < 0.50) {
        backwardThreshold = 2;
      }
      
      if (ProxSensors[i].backwardCount >= backwardThreshold) {
        ProxSensors[i].direction = BACKWARD;
      }
    }
    
    // NOISE FILTER: Ignore targets below 15% that aren't clearly forward
    // This prevents false locks on sensor noise
    if (zombieDistances[i] > 0.85 && ProxSensors[i].direction == FORWARD) {
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
  long currentPos = encoder.read();
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
