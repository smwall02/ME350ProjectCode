#include <Encoder.h>
#include <EEPROM.h>

//============================================
// DEBUG OPTIMIZATION - Comment out to save flash memory
//============================================
// Uncomment the line below to enable debug Serial output
// #define DEBUG_SERIAL  // DISABLED for faster processing

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
// NEW SEMANTICS: 0% = start, 100% = impact
// EARLY_ENGAGE = minimum % through lane to engage (lower = engage earlier)
// MIN_ENGAGE = maximum % through lane to engage (set above 1 to allow engaging near impact)
const float EARLY_ENGAGE_THRESHOLD_LONG[2] = {0.08, 0.08};   // Engage L1/L4 when > 8% through
const float EARLY_ENGAGE_THRESHOLD_SHORT[2] = {0.05, 0.05};  // Engage L2/L3 when > 5% through
const float MIN_ENGAGE_THRESHOLD = 1.01;  // Upper cap intentionally above normalized range (0-1)

// Get lane-specific engagement threshold
float getEarlyEngageThreshold(int lane) {
  if (lane == 1 || lane == 2) {  // L2 or L3 (short lanes)
    return EARLY_ENGAGE_THRESHOLD_SHORT[lane == 1 ? 0 : 1];
  } else {  // L1 or L4 (long lanes)
    return EARLY_ENGAGE_THRESHOLD_LONG[lane == 0 ? 0 : 1];
  }
}

// NEW SEMANTICS: 0% = at start, 100% = at impact (traveled through lane)
// Higher values = more urgent (closer to impact)
const float OVERRIDE_THRESHOLD[4] = {0.90, 0.50, 0.50, 0.90};  // Override when > this % through lane
const float ABSOLUTE_OVERRIDE_DISTANCE = 0.92;  // Critical when > 92% through lane
const float SHORT_LANE_CRITICAL_DISTANCE = 0.50;  // L2/L3 critical at 50% through lane
const float LONG_LANE_CRITICAL_DISTANCE = 0.85;  // L1/L4 critical at 85% through lane
const unsigned long MIN_COMMITMENT_TIME = 800;  // Minimum 800ms commitment before allowing overrides
const unsigned long TARGET_SWITCH_COOLDOWN = 1500;  // 1.5s cooldown after switching targets

const float RETREAT_CONFIRMED_DISTANCE = 0.65;  // Retreat confirmed when drops below 65% through
const float ZOMBIE_GONE_DISTANCE = 0.08;  // Zombie gone when < 8% through (near start)
const float L2_L3_GONE_DISTANCE = 0.05;  // L2/L3 gone when < 5% through
const float L4_GONE_DISTANCE = 0.10;  // L4 gone when < 10% through

//============================================
// DYNAMIC CALIBRATION
//============================================
bool calibrationActive = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 10000;  // 10 seconds of calibration

// Track min/max readings per lane during calibration
int calibrationMin[4] = {1023, 1023, 1023, 1023};  // Impact (low reading)
int calibrationMax[4] = {0, 0, 0, 0};              // Start (high reading)
int calibrationStart[4] = {0, 0, 0, 0};            // Initial reading at calibration start (= 0%)
bool calibrationUpdated[4] = {false, false, false, false};

//============================================
// SENSOR CALIBRATION
// Sensor is at START of lane. Target at impact is FAR from sensor.
// ProxRange[i][0] = high reading (target close to sensor = at START)
// ProxRange[i][1] = low reading (target far from sensor = at IMPACT)
// Sensor decreasing = zombie moving toward impact (FORWARD)
// Sensor increasing = zombie moving away from impact (BACKWARD)
//============================================
int ProxRange[4][2] = {
  {640, 80},   // Lane 1: [start/0%, impact/100%] - ~560 range
  {620, 100},  // Lane 2: [start/0%, impact/100%] - ~520 range
  {620, 90},   // Lane 3: [start/0%, impact/100%] - ~530 range
  {650, 135}   // Lane 4: [start/0%, impact/100%] - ~515 range
};

// Precomputed proximity scaling for faster sensor updates
float proxRangeInv[4] = {0, 0, 0, 0};
float proxRangeScaled[4] = {0, 0, 0, 0};
float laneDistanceScale[4] = {1.0f, 1.12f, 1.12f, 1.0f};

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
// SEQUENCING SYSTEM
// Sequences targets in groups of 4 (3/4 unique, 1 repeat allowed for short lanes L2/L3)
// Primary sort: distance from end of lane (using calibration ranges)
// Secondary sort: velocity and direction
//============================================
const int SEQUENCE_SIZE = 4;              // 4 targets per sequence
int targetSequence[SEQUENCE_SIZE] = {-1, -1, -1, -1};  // Ordered list of lanes in current sequence
int sequenceIndex = 0;                    // Current position in sequence
bool sequenceActive = false;              // Is a sequence currently active?
bool sequenceLocked = false;              // Is sequence locked? (stick to sequence)

// ATTEMPTED LANES TRACKING - Prevent re-engaging lanes already attempted
bool laneAttempted[4] = {false, false, false, false};  // Track which lanes we've already attempted
unsigned long laneAttemptTime[4] = {0, 0, 0, 0};       // When we attempted each lane
const unsigned long ATTEMPT_COOLDOWN = 1000;           // Allow faster re-engagement

// STOPPED LANES TRACKING - Prevent targeting lanes that have been stopped too long
unsigned long laneStoppedTime[4] = {0, 0, 0, 0};       // When each lane became STOPPED (0 = not stopped)
const unsigned long STOPPED_TIMEOUT = 2000;            // Skip lanes that have been STOPPED for > 2 seconds

//============================================
// DWELL TRACKING
//============================================
unsigned long arrivalTime = 0;
// NEW SEMANTICS: 0.0 = at start (reset value), higher = closer to impact
float peakZombieDistance = 0.0;
float arrivalZombieDistance = 0.0;

const unsigned long MIN_DWELL_TIME = 450;  // Increased to 450ms - no early exits
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

// Hit tracking for statistics
int lastHitLane = -1;                    // Last lane we successfully hit
unsigned long lastHitTime = 0;           // When we hit it
int consecutiveSameLane = 0;             // How many times we've hit same lane consecutively

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
void updateProxScaling();

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
  updateProxScaling();
  
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
  // CRITICAL: Require minimum range of 300 to prevent using corrupted/tiny ranges
  for (int i = 0; i < 4; i++) {
    int farVal, closeVal;
    EEPROM.get(EEPROM_PROX_RANGE_BASE + i * 4, farVal);
    EEPROM.get(EEPROM_PROX_RANGE_BASE + i * 4 + 2, closeVal);
    int range = farVal - closeVal;
    // Require: valid bounds AND minimum range of 300 (prevents corrupted tiny ranges)
    if (farVal > 400 && farVal < 900 && closeVal > 50 && closeVal < 400 && range > 300) {
      ProxRange[i][0] = farVal;
      ProxRange[i][1] = closeVal;
    } else {
      Serial.print(F("L")); Serial.print(i+1);
      Serial.print(F(": EEPROM range invalid (")); Serial.print(range);
      Serial.println(F("), using default"));
    }
  }

  updateProxScaling();
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
  // NEW SEMANTICS: Higher dist = more urgent (closer to impact)
  bool isEmergency = (dist > ABSOLUTE_OVERRIDE_DISTANCE && ProxSensors[lane].direction == FORWARD);
  if (!isEmergency) {
    float laneThreshold = getEarlyEngageThreshold(lane);
    // Valid range: dist > laneThreshold (upper cap intentionally disabled by MIN_ENGAGE_THRESHOLD > 1)
    if (dist < laneThreshold || dist > MIN_ENGAGE_THRESHOLD) return;
    if (ProxSensors[lane].direction != FORWARD) return;
    if (sequenceLocked && sequenceActive && committedLane >= 0) {
      bool isInSequence = false;
      for (int i = 0; i < SEQUENCE_SIZE; i++) {
        if (targetSequence[i] == lane) {
          isInSequence = true;
          break;
        }
      }
      if (!isInSequence) {
        float committedDist = zombieDistances[committedLane];
        float distanceGap = dist - committedDist;  // Positive = this lane further through
        float velocity = abs(zombieVelocities[lane]);
        bool isFastMoving = (velocity > 0.0005);
        float effectiveTTI = getEffectiveTTI(lane);
        bool isUrgent = (effectiveTTI < 800);
        bool isCritical = laneIsCritical[lane];
        bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
        float breakThreshold = laneIsShort[lane] ? 0.20 : 0.25;  // Positive gap = further through = more urgent
        bool shouldBreak = (distanceGap > breakThreshold) ||
                           (isCritical && !committedIsCritical) ||
                           (isFastMoving && distanceGap > 0.15 && dist > 0.50) ||
                           (isUrgent && distanceGap > 0.10 && dist > 0.60);
        if (!shouldBreak) return;
      }
    }
    bool isCritical = laneIsCritical[lane];

    if (!sequenceLocked || !sequenceActive || committedLane < 0 || isCritical) {
      float closestDist = 0.0;  // Track highest dist (closest to impact)
      int closestLane = -1;

      for (int i = 0; i < 4; i++) {
        // Only consider forward-moving targets in valid range - use lane-specific threshold
        if (i != lane &&
            ProxSensors[i].direction == FORWARD &&
            zombieDistances[i] > getEarlyEngageThreshold(i) &&
            zombieDistances[i] < MIN_ENGAGE_THRESHOLD) {
          if (zombieDistances[i] > closestDist) {  // Higher = closer to impact
            closestDist = zombieDistances[i];
            closestLane = i;
          }
        }
      }
      
      // If there's a more urgent forward-moving target, only commit to less urgent lane if gap is small
      // EXCEPTION: ANY lane at critical distance can override this (game-ending threat!)
      // NEW SEMANTICS: closestDist > dist means closest is more urgent (further through lane)
      if (closestLane >= 0 && closestDist > dist && !isCritical) {
        float distanceGap = closestDist - dist;  // Gap between most urgent and this lane
        
        // TIGHTER CRITERIA: If the gap is large, prefer the more urgent target
        // Require larger gaps to prevent switching to slightly more urgent targets
        // For L2/L3, use smaller gap since they're shorter lanes, but still tighter
        // OPTIMIZATION: Use cached lane type
        float gapThreshold = laneIsShort[lane] ? 0.15 : 0.20;  // TIGHTER: Require 15-20% gap (was 10-12%)
        // NEW SEMANTICS: Reject if there's a more urgent target (higher dist = closer to impact)
        if (distanceGap > gapThreshold && closestDist > 0.50) {
          return;  // Reject - more urgent target exists (past 50% through lane)
        }
      }
    }
  }  // End of !isEmergency block
  
  isCommitted = true;
  
  // LOCK SEQUENCE: Once we commit to first target in a sequence, lock the sequence
  if (sequenceActive && !sequenceLocked && targetSequence[0] >= 0) {
    // Check if this lane is the first target in the sequence
    if (targetSequence[0] == lane) {
      sequenceLocked = true;
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
  
  DBG_PRINT(F(">>LOCK L"));
  DBG_PRINT(lane + 1);
  DBG_PRINT(F(" @"));
      DBG_PRINT((int)(zombieDistances[lane] * 100));
  DBG_PRINTLN(F("%"));
}

//============================================
// RELEASE COMMITMENT
//============================================
void releaseCommitment() {
  if (isCommitted && verboseMode) {
    DBG_PRINTLN(F("~UNLOCK"));
  }
  isCommitted = false;
  committedLane = -1;
  commitStartTime = 0;
  commitStartDistance = 1.0;
}

//============================================
// SHOULD OVERRIDE CURRENT COMMITMENT?
// DISABLED: User requested NO overrides
// The batch system handles all target cycling
//============================================
bool shouldOverride(int newLane) {
  // OVERRIDES DISABLED - always return false
  // User explicitly requested no overrides
  return false;

  // --- DISABLED CODE BELOW ---
  /*
  if (!isCommitted) return true;
  if (newLane == committedLane) return false;

  // CRITICAL: NEVER override to backward-moving zombies - they're retreating!
  if (ProxSensors[newLane].direction == BACKWARD) return false;
  if (ProxSensors[newLane].direction != FORWARD) return false;
  unsigned long commitmentDuration = millis() - commitStartTime;
  float newDist = zombieDistances[newLane];
  // NEW SEMANTICS: Extreme emergency = very close to impact (> 92%)
  bool isExtremeEmergency = (newDist > 0.92);
  if (commitmentDuration < MIN_COMMITMENT_TIME && !isExtremeEmergency) return false;
  unsigned long timeSinceSwitch = millis() - lastTargetSwitchTime;
  if (timeSinceSwitch < TARGET_SWITCH_COOLDOWN && !isExtremeEmergency) return false;
  float committedDist = zombieDistances[committedLane];
  bool newIsLongLane = laneIsLong[newLane];
  bool committedIsShortLane = laneIsShort[committedLane];
  bool newIsCritical = laneIsCritical[newLane];
  // NEW SEMANTICS: Don't let long lane override short lane unless significantly further through
  if (newIsLongLane && committedIsShortLane && !newIsCritical) {
    if (newDist <= committedDist + 0.30) return false;  // Require 30%+ gap
  }
  if (sequenceLocked && sequenceActive) {
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
    
    // PRIORITY 2: Short lane getting close while committed to long lane - MUCH EARLIER!
    // NEW SEMANTICS: Override when short lane is past 50% through lane
    if (newIsShortLane && committedIsLongLane && newDist > 0.50) {
      return true;
    }

    // PRIORITY 3: Both critical, but new is significantly further through (20%+ more)
    // NEW SEMANTICS: Higher dist = closer to impact = higher priority
    if (newIsCritical && committedIsCritical && newDist > committedDist + 0.15) {
      return true;
    }

    // PRIORITY 4: Extreme emergency - new target past 90% through lane
    // NEW SEMANTICS: High dist = close to impact = urgent
    if (newDist > 0.90) {
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
    
    // NEW SEMANTICS: Distance is normalized where 0% = at start, 100% = at impact
    // So HIGH distance values mean close to impact (urgent!)
    // We want to override when distance is HIGH (close to impact)
    // Override MUCH earlier for short lanes!

    if (newIsShortLane && committedIsLongLane) {
      // Short lane past 50% through lane while at long lane = override immediately!
      if (newDist > 0.50 || newIsCritical) return true;
    } else if (newIsShortLane && !committedIsLongLane) {
      // Short lane past 70% through lane while at another lane = override
      if (newDist > 0.70) return true;
    }
    if (newIsCritical) return true;
    if (newDist > OVERRIDE_THRESHOLD[newLane]) return true;
    if (newDist > 0.95) return true;  // Extreme emergency (past 95%)
    return false;
  }
  if (activelyMoving && ProxSensors[committedLane].direction == FORWARD) {
    bool newIsCritical = laneIsCritical[newLane];
    bool committedIsCritical = (committedLane >= 0) ? laneIsCritical[committedLane] : false;
    bool newIsLongLane = laneIsLong[newLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    if (newIsLongLane && committedIsShortLane && !newIsCritical) return false;
    if (newIsCritical && !committedIsCritical) return true;
    // NEW SEMANTICS: Override if new is much further through lane (higher %)
    if (newIsCritical && committedIsCritical && newDist > committedDist + 0.20) return true;
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
  
  // RULE 4: If we're nearing target and committed lane still early (< 60%),
  // only override if new lane is MUCH more critical
  // NEW SEMANTICS: committedDist < 0.60 means committed is still early in lane
  if (nearingTarget && committedDist < 0.60 &&
      ProxSensors[committedLane].direction == FORWARD) {
    // CRITICAL: NEVER allow L1/L4 to override L2/L3 when nearing target, unless L1/L4 is critical
    bool newIsLongLane = laneIsLong[newLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    bool newIsCritical = laneIsCritical[newLane];

    if (newIsLongLane && committedIsShortLane && !newIsCritical) {
      // Long lane trying to override short lane when not critical - don't allow
      return false;
    }

    // NEW SEMANTICS: New must be significantly further through lane (40%+ ahead)
    return newDist > committedDist + 0.40;
  }

  float threshold = OVERRIDE_THRESHOLD[newLane];
  // NEW SEMANTICS: Override when new is PAST threshold (high dist = close to impact)
  if (newDist > threshold) {
    bool newIsLongLane = laneIsLong[newLane];
    bool committedIsShortLane = laneIsShort[committedLane];
    bool newIsCritical = laneIsCritical[newLane];
    if (newIsLongLane && committedIsShortLane && !newIsCritical) return false;
    // NEW SEMANTICS: Only override if new is much further through (35%+ ahead)
    if (committedDist > 0.75 && ProxSensors[committedLane].direction == FORWARD) {
      return newDist > committedDist + 0.10;  // Need 10%+ gap when both are far through
    }
    return true;
  }
  return false;
  */
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
    // Upper cap intentionally disabled (MIN_ENGAGE_THRESHOLD > 1) so we prioritize far-through targets
    float laneThreshold = getEarlyEngageThreshold(i);
    if (dist < laneThreshold || dist > MIN_ENGAGE_THRESHOLD) continue;
    
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
// SEQUENCING SYSTEM
// Creates ordered sequences prioritized by TIME TO IMPACT (TTI)
// Primary sort: Effective TTI (lowest first = will hit soonest)
// TTI = remaining distance / velocity, adjusted for travel time
//============================================

// Calculate and create a new target sequence
// RULES:
// - Only FORWARD-moving targets that meet threshold criteria
// - Sorted by EFFECTIVE TTI (lowest first = closest to impact)
// - Effective TTI = raw TTI - travel time - lane priority boost
// - Round start (all 0-5%): 4-lane sequence
// - Otherwise: 2-lane sequence with repeats allowed
void calculateNewSequence() {
  // Clear existing sequence
  for (int i = 0; i < SEQUENCE_SIZE; i++) {
    targetSequence[i] = -1;
  }
  sequenceIndex = 0;

  // Check if this is round start (all lanes at 0-5%)
  bool isRoundStart = true;
  for (int i = 0; i < 4; i++) {
    if (zombieDistances[i] > 0.05) {
      isRoundStart = false;
      break;
    }
  }

  // Determine sequence length: 4 at round start, 2 otherwise
  int maxSequenceLen = isRoundStart ? 4 : 2;

  // Gather lane info - NOW USING TTI FOR PRIORITIZATION
  struct LaneInfo {
    int lane;
    float distance;      // Current position (0-1, higher = closer to impact)
    float tti;           // Time To Impact in ms (LOWER = more urgent!)
    float effectiveTTI;  // TTI adjusted for travel time and lane priority
    bool isTargetable;   // FORWARD and meets threshold
  };
  LaneInfo lanes[4];
  int targetableCount = 0;

  // TTI boost for lanes 2 and 3 (indices 1 and 2) - subtract 200ms to make them more urgent
  const float LANE_2_3_TTI_BOOST = 200.0;
  // Lane 4 gets priority boost when at 70%+ (30% from impact)
  const float LANE_4_CRITICAL_THRESHOLD = 0.70;
  const float LANE_4_TTI_BOOST = 300.0;  // 300ms boost when critical

  for (int i = 0; i < 4; i++) {
    lanes[i].lane = i;
    lanes[i].distance = zombieDistances[i];
    lanes[i].tti = timeToImpact[i];

    // Calculate effective TTI = raw TTI - travel time - lane boost
    // Lower effective TTI = MORE URGENT
    int travelTime = getDynamicTravelTime(i);
    lanes[i].effectiveTTI = lanes[i].tti - travelTime;

    // Apply TTI boost for lanes 2 and 3 (make them more urgent by subtracting)
    if (i == 1 || i == 2) {
      lanes[i].effectiveTTI -= LANE_2_3_TTI_BOOST;
    }

    // Lane 4 (index 3) gets priority boost when at 70%+ (30% from impact)
    if (i == 3 && lanes[i].distance >= LANE_4_CRITICAL_THRESHOLD) {
      lanes[i].effectiveTTI -= LANE_4_TTI_BOOST;
    }

    // Clamp to reasonable range
    if (lanes[i].effectiveTTI < 0) lanes[i].effectiveTTI = 0;
    if (lanes[i].tti >= 99999) lanes[i].effectiveTTI = 99999;  // Invalid TTI

    // TARGETABLE: Must be FORWARD
    // Lanes 1 and 4 (indices 0 and 3): require >= 30% threshold
    // Lanes 2 and 3 (indices 1 and 2): no threshold (always eligible if FORWARD)
    bool isForward = (ProxSensors[i].direction == FORWARD);
    bool isPastThreshold;
    if (i == 1 || i == 2) {
      // Lanes 2 and 3: no threshold
      isPastThreshold = true;
    } else {
      // Lanes 1 and 4: require >= 30%
      isPastThreshold = (zombieDistances[i] >= 0.30);
    }
    lanes[i].isTargetable = (isForward && isPastThreshold);
    if (lanes[i].isTargetable) targetableCount++;
  }

  // Sort by EFFECTIVE TTI (LOWEST first = closest to impact = highest priority)
  // This prioritizes targets that will hit soonest, accounting for travel time
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (lanes[j].effectiveTTI < lanes[i].effectiveTTI) {
        LaneInfo temp = lanes[i];
        lanes[i] = lanes[j];
        lanes[j] = temp;
      }
    }
  }

  int sequenceCount = 0;

  if (isRoundStart) {
    // ROUND START: Include all 4 lanes, sorted by TTI
    // Even if not all are forward yet, include them for coverage
    for (int i = 0; i < 4 && sequenceCount < 4; i++) {
      targetSequence[sequenceCount++] = lanes[i].lane;
    }
  } else {
    // NORMAL: Build 2-lane sequence from targetable lanes, sorted by TTI
    for (int i = 0; i < 4 && sequenceCount < maxSequenceLen; i++) {
      if (lanes[i].isTargetable) {
        targetSequence[sequenceCount++] = lanes[i].lane;
      }
    }

    // If we have 1 target but need 2, allow repeat
    if (sequenceCount == 1 && maxSequenceLen == 2) {
      targetSequence[sequenceCount++] = targetSequence[0];
    }
  }

  sequenceActive = (sequenceCount > 0);
  sequenceLocked = false;
}

// Forward declaration
bool shouldRecalculateForLowLane();

// Get the next target from the current sequence
int getNextSequenceTarget() {
  // If sequence is empty or exhausted, calculate new one
  if (!sequenceActive || sequenceIndex >= SEQUENCE_SIZE) {
    calculateNewSequence();
    if (!sequenceActive) return -1;
  }

  // Check if any lane has 20% or less remaining that's not our current target
  // If so, recalculate sequence to prioritize critical lanes
  if (shouldRecalculateForLowLane()) {
    calculateNewSequence();
    if (!sequenceActive) return -1;
  }

  // Find next valid target in sequence
  while (sequenceIndex < SEQUENCE_SIZE) {
    int lane = targetSequence[sequenceIndex];
    
    // Check if lane is valid
    if (lane < 0 || lane >= 4) {
      sequenceIndex++;
      continue;
    }
    
    // Validate: must be FORWARD (lanes 1/4 need >= 30%, lanes 2/3 no threshold)
    bool isForward = (ProxSensors[lane].direction == FORWARD);
    bool isBackward = (ProxSensors[lane].direction == BACKWARD);
    // Lanes 2 and 3 (indices 1 and 2): no threshold
    // Lanes 1 and 4 (indices 0 and 3): require >= 30%
    float minThreshold = (lane == 1 || lane == 2) ? 0.0 : 0.30;
    bool isInRange = (zombieDistances[lane] >= minThreshold &&
                      zombieDistances[lane] < MIN_ENGAGE_THRESHOLD);
    
    // If target is backward-moving, skip it
    if (isBackward) {
      sequenceIndex++;
      continue;
    }
    
    // Check if lane was recently attempted - skip if within cooldown
    bool recentlyAttempted = false;
    if (laneAttempted[lane]) {
      unsigned long timeSinceAttempt = millis() - laneAttemptTime[lane];
      if (timeSinceAttempt < ATTEMPT_COOLDOWN) {
        recentlyAttempted = true;
      } else {
        laneAttempted[lane] = false;  // Cooldown expired
      }
    }
    
    // Check if this target is still valid (forward-moving, in range, not recently attempted)
    if (isForward && isInRange && !recentlyAttempted) {
      sequenceIndex++;  // Move to next for next call
      return lane;
    }
    
    // Target no longer valid - skip it
    sequenceIndex++;
  }
  
  // Sequence exhausted, calculate new one
  calculateNewSequence();
  if (!sequenceActive) return -1;
  
  // Return first target of new sequence
  if (targetSequence[0] >= 0) {
    sequenceIndex = 1;
    return targetSequence[0];
  }
  
  return -1;
}

// Reset the sequence (called on emergency override or major change)
void resetSequence() {
  sequenceActive = false;
  sequenceLocked = false;  // Unlock sequence on reset
  sequenceIndex = 0;
  for (int i = 0; i < SEQUENCE_SIZE; i++) {
    targetSequence[i] = -1;
  }
}

// Check if any lane needs urgent attention based on TTI or distance
// Returns true if recalculation is needed (a critical lane is not the current sequence target)
// Uses TTI when available, falls back to distance threshold
bool shouldRecalculateForLowLane() {
  const float LOW_LANE_THRESHOLD = 0.70;  // 70% progress = 30% remaining
  const float URGENT_TTI_THRESHOLD = 1500;  // 1.5 seconds to impact = urgent

  // Find the current sequence target (if any)
  int currentTarget = -1;
  float currentTTI = 99999;
  if (sequenceActive && sequenceIndex < SEQUENCE_SIZE) {
    currentTarget = targetSequence[sequenceIndex];
    if (currentTarget >= 0) {
      currentTTI = getEffectiveTTI(currentTarget);
    }
  }

  // Check each lane for urgent TTI or low remaining distance
  for (int i = 0; i < 4; i++) {
    // Skip if this is already our current target
    if (i == currentTarget) continue;

    // Only check FORWARD-moving targets
    if (ProxSensors[i].direction != FORWARD) continue;

    float tti = getEffectiveTTI(i);
    float dist = zombieDistances[i];

    // URGENT if TTI is very low (will hit soon) and lower than current target
    if (tti < URGENT_TTI_THRESHOLD && tti < currentTTI - 200) {
      return true;  // This lane will hit sooner - recalculate!
    }

    // Also trigger on distance threshold as backup
    if (dist >= LOW_LANE_THRESHOLD && dist < MIN_ENGAGE_THRESHOLD) {
      return true;
    }
  }

  return false;
}

// Mark current sequence target as complete (hit registered)
// After each target is hit, advance to next in sequence
void advanceSequence() {
  sequenceIndex++;  // Move to next target in sequence
  
  // If sequence is complete, calculate new one
  if (sequenceIndex >= SEQUENCE_SIZE) {
    sequenceLocked = false;
    sequenceActive = false;
    sequenceIndex = 0;
    
    // Immediately recalculate new sequence
    calculateNewSequence();
  } else {
    // Still targets remaining in current sequence
    sequenceLocked = false;  // Allow recalculation if needed
  }
}

// Check if any lane needs emergency attention (outside sequence order)
int checkSequenceEmergency() {
  for (int i = 0; i < 4; i++) {
    // Skip the lane we're currently committed to
    if (i == committedLane) continue;
    
    // Only check forward-moving targets
    if (ProxSensors[i].direction != FORWARD) continue;
    
    // Emergency threshold - very close to impact
    // Use lane-specific thresholds (L1/L4 at 90%, L2/L3 at 50%)
    // NEW SEMANTICS: Higher dist = closer to impact
    if (zombieDistances[i] > OVERRIDE_THRESHOLD[i]) {
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
      // NEW SEMANTICS: Keep if dist > laneThreshold (past minimum engagement point)
      if (ProxSensors[lane].direction == FORWARD) {
        float laneThreshold = getEarlyEngageThreshold(lane);
        if (zombieDistances[lane] > laneThreshold) {
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
    
    // CRITICAL: Verify ranges are actually updated and will be used
    Serial.println(F("=== VERIFICATION: Ranges now in use ==="));
    for (int i = 0; i < 4; i++) {
      Serial.print(F("L"));
      Serial.print(i + 1);
      Serial.print(F(": ProxRange["));
      Serial.print(ProxRange[i][0]);
      Serial.print(F(","));
      Serial.print(ProxRange[i][1]);
      Serial.print(F("] - Distance calculations will use these values"));
      Serial.println();
    }
    Serial.println(F("All distance calculations now use calibrated ranges."));
    return;
  }
  
  // Update min/max for each lane based on RAW analog readings (not smoothed!)
  // CRITICAL: Must use raw analogRead() to capture true min/max values
  // Calibration mapping:
  // - Start (0%): HIGH sensor reading (target far from sensor) = calibrationMax
  // - Impact (100%): LOW sensor reading (target close to sensor) = calibrationMin
  for (int i = 0; i < 4; i++) {
    // Use raw analog reading directly - smoothed values won't capture true extremes
    int rawVal = analogRead(ProxSensors[i].pin);
    
    // Track minimum (impact/100% = lowest sensor reading when target is closest)
    // Track maximum (start/0% = highest sensor reading when target is at beginning)
    // Sensor behavior: LOW value = target close (at impact/100%), HIGH value = target far (at start/0%)
    
    if (rawVal < calibrationMin[i]) {
      calibrationMin[i] = rawVal;  // Minimum = target at impact/100% (lowest reading)
      calibrationUpdated[i] = true;
    }
    if (rawVal > calibrationMax[i]) {
      calibrationMax[i] = rawVal;  // Maximum = target at start/0% (highest reading, may be slightly +/-)
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
    // Use calibrationStart (captured at begin) as 0% baseline
    // Use calibrationMin (tracked during cal) as 100% impact point
    // Use the highest observed value as the "start" reference. If the user moved a target
    // closer to the sensor after calibration began, calibrationMax will capture it. This
    // avoids compressing the usable range around a stale initial reading.
    int startVal = max(calibrationStart[i], calibrationMax[i]);  // Initial/maximum = 0%
    // Impact is always the lowest value we observed (target far from sensor)
    int impactVal = calibrationMin[i];   // Minimum reading = 100%
    int range = startVal - impactVal;

    // Debug: Print what we collected
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": Start(0%)="));
    Serial.print(startVal);
    Serial.print(F(", Impact(100%)="));
    Serial.print(impactVal);
    Serial.print(F(", Max seen="));
    Serial.print(calibrationMax[i]);
    Serial.print(F(", range="));
    Serial.println(range);

    // Treat untouched sensors as invalid; they will keep defaults
    if (!calibrationUpdated[i]) {
      Serial.print(F("  L"));
      Serial.print(i + 1);
      Serial.println(F(": no movement captured, keeping existing range"));
      continue;
    }

    // Check if we have valid calibration data (range > 150 for good resolution)
    if (range > 150 && impactVal < startVal) {
      // Use the INITIAL reading (calibrationStart) as 0%
      // Use the MINIMUM seen (calibrationMin) as 100%
      int newFar = startVal;    // Start/0% = initial reading
      int newClose = impactVal; // Impact/100% = minimum reading

      // Add tiny margins for sensor noise
      newFar = min(newFar + 2, 1023);
      newClose = max(newClose - 2, 0);
      
      // Final sanity check - ensure reasonable range remains (tightened to 120 counts)
      int finalRange = newFar - newClose;
      if (newFar > newClose && finalRange > 120) {
        // Save OLD values before updating for comparison
        int oldFar = ProxRange[i][0];
        int oldClose = ProxRange[i][1];
        
        // CRITICAL: Update ProxRange with calibrated values
        // ProxRange[0] = far value = start/0% (high reading)
        // ProxRange[1] = close value = impact/100% (low reading)
        ProxRange[i][0] = newFar;   // Start/0% (high reading when target at beginning)
        ProxRange[i][1] = newClose; // Impact/100% (low reading when target at impact)
        
        // Automatically save to EEPROM so calibration persists across power cycles
        EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4, (int)newFar);
        EEPROM.put(EEPROM_PROX_RANGE_BASE + i * 4 + 2, (int)newClose);
        
        Serial.print(F("  L"));
        Serial.print(i + 1);
        Serial.print(F(": OLD ["));
        Serial.print(oldFar);
        Serial.print(F(","));
        Serial.print(oldClose);
        Serial.print(F("] -> NEW ["));
        Serial.print(newFar);
        Serial.print(F(","));
        Serial.print(newClose);
        Serial.print(F("] range="));
        Serial.print(finalRange);
        Serial.println(F(" [APPLIED & SAVED]"));
      } else {
        Serial.print(F("  L"));
        Serial.print(i + 1);
        Serial.print(F(": range too small after margins ("));
        Serial.print(finalRange);
        Serial.println(F("), keeping default"));
      }
    } else {
      Serial.print(F("  L"));
      Serial.print(i + 1);
      Serial.print(F(": insufficient calibration data (range="));
      Serial.print(range);
      Serial.print(F(", min="));
      Serial.print(calibrationMin[i]);
      Serial.print(F(", max="));
      Serial.print(calibrationMax[i]);
      Serial.println(F("), keeping default"));
    }
  }

  updateProxScaling();

  // No additional normalization needed - we used calibrationStart (captured at begin) as 0%
  // This ensures targets at their initial position = exactly 0%

  // Force a print of current ranges to verify they were updated
  Serial.println(F("--- Final Prox Ranges (NOW IN USE) ---"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": ["));
    Serial.print(ProxRange[i][0]);
    Serial.print(F(","));
    Serial.print(ProxRange[i][1]);
    Serial.print(F("] -> Distance calculation will use these values"));
    Serial.println();
  }
  Serial.println(F("Calibration ranges are now active and will be used for all distance calculations."));
}

//============================================
// PRECOMPUTE PROXIMITY SCALING
// Calculates spans and inverses to avoid repeated division in updateSensors()
//============================================
void updateProxScaling() {
  for (int i = 0; i < 4; i++) {
    float span = (float)(ProxRange[i][0] - ProxRange[i][1]);

    // Prevent divide-by-zero and ensure reasonable defaults
    if (span < 1.0f) {
      span = 1.0f;
    }

    proxRangeInv[i] = 1.0f / span;
    // Combine lane scaling to remove an extra multiply per updateSensors() call
    proxRangeScaled[i] = laneDistanceScale[i] * proxRangeInv[i];
  }
}

//============================================
// START CALIBRATION
//============================================
void startCalibration() {
  calibrationActive = true;
  calibrationStartTime = millis();
  
  // CRITICAL: Move mechanism to encoder home (position 0) and keep it there during calibration
  desiredPosition = 0;
  systemEnabled = true;  // Ensure PID controller is active to maintain position
  
  // Reset calibration tracking
  // CRITICAL: Calibration starts when targets are at 0% (beginning of lane)
  // At start (0%): sensor reads HIGH (target close to sensor) = calibrationStart
  // At impact (100%): sensor reads LOW (target far from sensor) = calibrationMin
  Serial.println(F("=== CALIBRATION STARTED (10s) ==="));
  Serial.println(F("Capturing initial RAW readings as 0% baseline..."));
  for (int i = 0; i < 4; i++) {
    // Capture RAW initial reading - this is the 0% position
    calibrationStart[i] = analogRead(ProxSensors[i].pin);
    // Initialize min/max tracking
    calibrationMin[i] = 1023;  // Will track down to find minimum (impact/100%)
    calibrationMax[i] = calibrationStart[i];  // Start with initial reading as max
    calibrationUpdated[i] = false;
    Serial.print(F("  L"));
    Serial.print(i + 1);
    Serial.print(F(": Initial RAW = "));
    Serial.print(calibrationStart[i]);
    Serial.println(F(" (= 0%)"));
  }
  Serial.println(F("Mechanism moving to home (position 0)..."));
  Serial.println(F("Move targets to 100% (impact) and back!"));
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
  // NEW SEMANTICS: Higher dist = closer to impact = critical
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
  
  // Update calibration if active (runs alongside normal operation)
  if (calibrationActive) {
    updateCalibration();
  }

  // If our committed target starts retreating, abandon it and pick a new threat
  if (autoMode && systemEnabled && isCommitted && committedLane >= 0 &&
      ProxSensors[committedLane].direction == BACKWARD) {
    laneAttempted[committedLane] = true;
    laneAttemptTime[committedLane] = millis();
    activeTargetIndex = -1;
    releaseCommitment();
    state = CHOOSE_TARGET;
  }
  
  //============================================
  // STATE MACHINE - BATCH-BASED TARGETING
  //============================================
  static unsigned long lastOverrideCommit = 0;  // Track when we last committed via override
  static int lastOverrideLane = -1;             // Track which lane we overrode to
  static unsigned long overrideCheckTime = 0;   // Rate limit override checks
  
  // CRITICAL: Skip state machine during calibration - mechanism must stay at position 0
  if (calibrationActive) {
    // State machine disabled during calibration - PID controller will maintain position 0
  } else if (autoMode && systemEnabled && !gameOver) {

    //============================================
    // ABSOLUTE EMERGENCY OVERRIDE - BYPASSES ALL COOLDOWNS
    // If ANY lane is at 85%+ FORWARD, IMMEDIATELY target it
    // This prevents lanes from reaching 100% (game over)
    // BUT: Do NOT interrupt dwell time - let the hit register first
    //============================================
    const float ABSOLUTE_EMERGENCY_THRESHOLD = 0.85;  // 85% = 15% remaining
    int absoluteEmergencyLane = -1;
    float highestEmergencyDist = 0;

    // Check if we're currently dwelling - don't interrupt dwell!
    bool isDwelling = (state == DWELL_AT_TARGET);
    bool canEmergencyOverride = !isDwelling;  // Only override if NOT dwelling

    // If dwelling, only allow emergency override if current target moved BACKWARD
    if (isDwelling && committedLane >= 0) {
      if (ProxSensors[committedLane].direction == BACKWARD) {
        canEmergencyOverride = true;  // Target retreated - OK to override
      }
    }

    if (canEmergencyOverride) {
      for (int i = 0; i < 4; i++) {
        // Skip current target - we're already on it
        if (i == committedLane) continue;

        // Only check FORWARD-moving targets
        if (ProxSensors[i].direction != FORWARD) continue;

        float dist = zombieDistances[i];

        // Check if this lane is in absolute emergency (85%+ and higher than any we've seen)
        if (dist >= ABSOLUTE_EMERGENCY_THRESHOLD && dist < 0.99 && dist > highestEmergencyDist) {
          absoluteEmergencyLane = i;
          highestEmergencyDist = dist;
        }
      }

      // If we found an absolute emergency lane, IMMEDIATELY commit to it
      if (absoluteEmergencyLane >= 0) {
        // Only switch if emergency lane is more critical than current target
        float currentDist = (committedLane >= 0) ? zombieDistances[committedLane] : 0;
        bool currentIsForward = (committedLane >= 0) ? (ProxSensors[committedLane].direction == FORWARD) : false;

        // Switch if: no current target, current is backward, or emergency is further along
        if (committedLane < 0 || !currentIsForward || highestEmergencyDist > currentDist + 0.05) {
          Serial.print(F("!!!EMERGENCY L"));
          Serial.print(absoluteEmergencyLane + 1);
          Serial.print(F(" @"));
          Serial.print((int)(highestEmergencyDist * 100));
          Serial.println(F("%"));

          // Force immediate commit - bypass all normal checks
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
    // EMERGENCY OVERRIDE CHECK
    // Interrupts batch execution for critical threats
    //============================================
    int overrideLane = -1;
    float bestOverrideScore = 0;

    unsigned long timeSinceOverride = millis() - lastOverrideCommit;
    // CRITICAL FIX: Add cooldown after overrides to prevent rapid switching
    // After an override, wait at least 2 seconds before allowing another override
    const unsigned long OVERRIDE_COOLDOWN = 2000;  // 2 second cooldown after override (increased to prevent loops)
    
    // CRITICAL FIX: Allow override checks while dwelling - but still respect cooldown
    // While dwelling, we need to continuously check for critical threats (especially short lanes)
    // Note: isDwelling already declared above in emergency override section
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
      // FIXED: Allow overrides while moving if short lane is getting critical (reduced from 3s to 1.5s)
      if (activelyMoving && timeSinceOverride < 1500) {
        canCheckOverride = false;  // Don't override while actively moving unless 1.5s has passed
      }
    }
    
    if (canCheckOverride) {
      overrideCheckTime = millis();
      
      for (int i = 0; i < 4; i++) {
        if (i == committedLane) continue;
        // CRITICAL: NEVER target backward-moving zombies - they're retreating!
        if (ProxSensors[i].direction == BACKWARD) continue;
        // CRITICAL: Allow STOPPED lanes if they're at 95%+ through lane (about to impact!)
        // Also allow FORWARD lanes
        // NEW SEMANTICS: Higher dist = closer to impact
        float dist = zombieDistances[i];
        bool isAboutToImpact = (dist > 0.95 && dist < 0.99);  // 95%+ through lane
        if (ProxSensors[i].direction != FORWARD && !(ProxSensors[i].direction == STOPPED && isAboutToImpact)) {
          continue;  // Skip non-forward lanes, unless they're stopped and about to impact
        }
        // TIGHTER: Longer anti-return period - prevent overriding back to same lane
        // CRITICAL FIX: Increase anti-return time to prevent loops
        if (i == lastOverrideLane && timeSinceOverride < 5000) continue;  // 5s anti-return to prevent loops (increased from 3s)

        // CRITICAL: Skip STOPPED lanes that have been stopped for > 2 seconds (freeze protection)
        // EXCEPTION: Allow STOPPED lanes if they're at 95%+ through lane (about to impact!)
        dist = zombieDistances[i];
        isAboutToImpact = (dist > 0.95 && dist < 0.99);  // 95%+ through lane
        if (ProxSensors[i].direction == STOPPED && laneStoppedTime[i] > 0 && !isAboutToImpact) {
          unsigned long stoppedDuration = millis() - laneStoppedTime[i];
          if (stoppedDuration > STOPPED_TIMEOUT) {
            continue;  // Skip this lane - it's been frozen too long (unless about to impact)
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
        dist = zombieDistances[i];
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
        if (sequenceLocked && sequenceActive) {
          float committedDist = zombieDistances[committedLane];
          float distanceGap = abs(dist - committedDist);
          
          // CRITICAL: Short lanes can override long lanes with smaller gap
          // Short lanes have less time - they need priority even at similar distances
          // Reuse isShortLane and committedIsLongLane already declared above
          committedIsLongLane = laneIsLong[committedLane];
          
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
            // STRICT: Only allow if new lane is MUCH further through (50%+ more than committed)
            // NEW SEMANTICS: Higher dist = closer to impact
            if (dist <= committedDist + 0.50) {
              continue;  // Skip - don't allow override, protect short lanes
            }
          }

          // When batch is locked, only override for:
          // 0. HIGHEST: ANY lane at 95%+ through lane (about to impact!) - OVERRIDE EVERYTHING
          // 1. Critical vs non-critical (game-ending threat)
          // 2. Short lane getting close (> 50%) while committed to long lane - MUCH EARLIER!
          // 3. Extreme emergency (> 90% through)
          // 4. Both critical but new is significantly further through (at least 30% more)
          // 5. L4 at 90%+ through lane - HIGH PRIORITY
          // NEW SEMANTICS: Higher dist = closer to impact
          if (dist > 0.95 && ProxSensors[i].direction == FORWARD && dist < 0.99) {
            // ANY lane at 95%+ through lane (about to impact!) - override immediately!
            shouldOverride = true;
          } else if (i == 3 && dist > 0.90 && ProxSensors[i].direction == FORWARD && dist < 0.98) {
            // L4 at 90%+ through lane - override immediately!
            shouldOverride = true;
          } else if (isCritical && !committedIsCritical) {
            shouldOverride = true;  // Critical vs non-critical = always override
          } else if (isShortLane && committedIsLongLane && dist > 0.50) {
            shouldOverride = true;  // Short lane past 50% while at long lane - override!
          } else if (dist > 0.90) {
            shouldOverride = true;  // Extreme emergency (90%+ through)
          } else if (isCritical && committedIsCritical && dist > committedDist + 0.30) {
            shouldOverride = true;  // Both critical, new is 30%+ further through
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
            // STRICT: Only allow if new lane is MUCH further through (50%+ more than committed)
            // NEW SEMANTICS: Higher dist = closer to impact
            if (dist <= committedDist + 0.50) {
              continue;  // Skip - don't allow override, protect short lanes
            }
          }

          // Batch not locked - use normal override criteria
          // CRITICAL: Check if L4 is also close - if so, reduce L2/L3 override aggressiveness
          // Only consider L4 "close" if it's actually moving forward and past start position
          // NEW SEMANTICS: Higher dist = closer to impact
          bool l4AlsoClose = (zombieDistances[3] > 0.90 &&
                              ProxSensors[3].direction == FORWARD &&
                              zombieDistances[3] < 0.98);  // L4 at 90%+ through lane and moving forward

          // HIGHEST PRIORITY 0: ANY lane at 95%+ through lane = IMMEDIATE OVERRIDE
          // This is game-ending - zombie is about to impact!
          // NEW SEMANTICS: Higher dist = closer to impact
          if (dist > 0.95 && ProxSensors[i].direction == FORWARD && dist < 0.99) {
            // ANY lane at 95%+ through lane (about to impact!) - override immediately!
            shouldOverride = true;
          }
          // PRIORITY 1: Long lane (L4) getting very close = HIGH PRIORITY
          // L4 at 90%+ through lane needs immediate attention
          else if (i == 3 && dist > 0.90 && ProxSensors[i].direction == FORWARD && dist < 0.98) {
            // L4 is at 90%+ through lane and moving forward - override immediately!
            shouldOverride = true;
          }
          // PRIORITY 2: Short lane getting close while committed to long lane = IMMEDIATE OVERRIDE
          // BUT: Reduce aggressiveness if L4 is also very close
          // NEW SEMANTICS: Higher dist = closer to impact
          else if (isShortLane && committedIsLongLane && dist > 0.50) {
            // If L4 is also very close, require short lane to be even closer (60%+ through)
            if (l4AlsoClose && dist < 0.60) {
              shouldOverride = false;  // L4 is more urgent - don't override to short lane yet
            } else {
              // Short lane past 50% while at long lane = override immediately!
              shouldOverride = true;
            }
          }
          // PRIORITY 3: Short lane getting close while committed to another short lane
          else if (isShortLane && !committedIsLongLane && dist > 0.30) {
            // Short lane past 30% while at another lane = override
            shouldOverride = true;
          }
          // PRIORITY 4: Critical vs non-critical = override
          else if (isCritical && !committedIsCritical) {
            shouldOverride = true;
          }
          // PRIORITY 5: Critical and much further through (at least 30% more)
          // NEW SEMANTICS: Higher dist = closer to impact
          else if (isCritical && committedIsCritical && dist > committedDist + 0.30) {
            shouldOverride = true;
          }
          // PRIORITY 6: Short lane getting close while long lane is less urgent
          // NEW SEMANTICS: Higher dist = closer to impact
          else if (isShortLane && dist > 0.40 && committedIsLongLane && committedDist < 0.70) {
            // Short lane past 40% while long lane still below 70% = override
            shouldOverride = true;
          }
          // PRIORITY 7: Above override threshold and committed is not critical
          // NEW SEMANTICS: Higher dist = closer to impact
          else if (dist > OVERRIDE_THRESHOLD[i] && !committedIsCritical) {
            shouldOverride = true;
          }
        }
        
        if (shouldOverride) {
          // PRIORITIZE BY TIME TO IMPACT (TTI) - which will hit first?
          // Use effective TTI to determine priority (lower TTI = will hit sooner = higher priority)
          float effectiveTTI = getEffectiveTTI(i);
          
          // If TTI is invalid, use a very high value (low priority)
          if (effectiveTTI >= 99999) {
            effectiveTTI = 999999;  // Very low priority
          }
          
          // Select the lane with the LOWEST TTI (will hit first)
          // If TTI is the same, use score as tiebreaker
          bool shouldSelect = false;
          if (overrideLane < 0) {
            // No override selected yet - select this one
            shouldSelect = true;
          } else {
            float currentTTI = getEffectiveTTI(overrideLane);
            if (currentTTI >= 99999) currentTTI = 999999;
            
            if (effectiveTTI < currentTTI) {
              // This lane will hit sooner - select it
              shouldSelect = true;
            } else if (effectiveTTI == currentTTI) {
              // Same TTI - use score as tiebreaker
              float thisScore = calculateThreatScore(i);
              float currentScore = calculateThreatScore(overrideLane);
              if (thisScore > currentScore) {
                shouldSelect = true;
              }
            }
          }
          
          if (shouldSelect) {
            overrideLane = i;
            bestOverrideScore = effectiveTTI;  // Store TTI for comparison
          }
        }
      }
    }
    
    // EMERGENCY OVERRIDE - reset batch and handle immediately
    // CRITICAL FIX: Only allow override if we haven't just overridden to this lane
    // Prevent override loops by checking if we're already committed to the override lane
    if (overrideLane >= 0 && shouldOverride(overrideLane) && overrideLane != committedLane) {
      int previousLane = committedLane;
      
      // CRITICAL: Prevent override loops - don't override if we just overrode to this lane recently
      if (overrideLane == lastOverrideLane && timeSinceOverride < 5000) {
        // Just overrode to this lane within 5 seconds - skip to prevent loop (increased from 3s)
        overrideLane = -1;
      } else {
        // CRITICAL: Only print and commit ONCE - prevent spam
        // Check if we're already moving to this target
        if (isCommitted && committedLane == overrideLane) {
          // Already committed to this lane - don't override again
          overrideLane = -1;
        } else {
          DBG_PRINT(F("!!! OVERRIDE L"));
          DBG_PRINT(overrideLane + 1);
          DBG_PRINT(F(" @"));
          DBG_PRINT((int)(zombieDistances[overrideLane] * 100));
          DBG_PRINTLN(F("% !!!"));
          
          // Reset batch on emergency - will recalculate after this target
          resetSequence();
          
          releaseCommitment();
          commitToTarget(overrideLane);
          
          if (isCommitted) {
            lastOverrideCommit = millis();
            lastOverrideLane = overrideLane;  // Track which lane we overrode TO, not FROM
            state = MOVE_TO_TARGET;
            // CRITICAL: Exit early to prevent checking overrides again this loop
            overrideLane = -1;  // Clear to prevent re-checking
            return;  // Exit immediately to prevent re-checking in same loop
          } else {
            // Commit failed - clear override to prevent spam
            overrideLane = -1;
          }
        }
      }
    }
    //============================================
    // NORMAL BATCH EXECUTION
    //============================================
    else if (isCommitted && committedLane >= 0) {
      // Ensure we're targeting the committed lane
      // CRITICAL: Don't change desiredPosition during calibration
      if (activeTargetIndex != committedLane && !calibrationActive) {
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
          chooseAndCommitTargetFromSequence();
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
      DBG_PRINTLN(F("Switch OFF"));
    }
  }
  
  //============================================
  // STATUS OUTPUT
  //============================================
  if (autoMode && (millis() - lastPrintTime >= 350)) {  // Reduced from 200ms to 350ms for faster processing
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
    // CRITICAL: NEVER target backward-moving zombies - they're retreating!
    if (ProxSensors[lane].direction == BACKWARD) continue;
    if (ProxSensors[lane].direction != FORWARD) continue;
    
    float dist = zombieDistances[lane];
    // Use lane-specific threshold - L2/L3 engage earlier
    // NEW SEMANTICS: Valid range is dist > laneThreshold AND dist < MIN_ENGAGE_THRESHOLD
    float laneThreshold = getEarlyEngageThreshold(lane);
    if (dist < laneThreshold || dist > MIN_ENGAGE_THRESHOLD) continue;

    float score = calculateThreatScore(lane);
    if (score > bestScore) {
      bestScore = score;
      bestLane = lane;
    }
  }

  // Now check all lanes for new targets - might be better than queued
  for (int i = 0; i < 4; i++) {
    // CRITICAL: NEVER target backward-moving zombies - they're retreating!
    if (ProxSensors[i].direction == BACKWARD) continue;
    // Skip non-forward targets
    if (ProxSensors[i].direction != FORWARD) continue;

    float dist = zombieDistances[i];

    // Skip if zombie is outside valid engagement range - use lane-specific threshold
    // NEW SEMANTICS: Valid range is dist > laneThreshold AND dist < MIN_ENGAGE_THRESHOLD
    float laneThreshold = getEarlyEngageThreshold(i);
    if (dist < laneThreshold || dist > MIN_ENGAGE_THRESHOLD) continue;
    
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
// BREAKS SEQUENCE if a better FORWARD target is farther along
//============================================
void chooseAndCommitTargetFromSequence() {
  // FIRST: Check if any FORWARD lane is farther along than our sequence target
  // If so, override the sequence and target that lane instead
  int bestOverrideLane = -1;
  float bestOverrideDist = -1.0;

  // Find the best FORWARD target by distance
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction == FORWARD) {
      // Check lane-specific threshold (lanes 2/3 have no threshold)
      float minThreshold = (i == 1 || i == 2) ? 0.0 : 0.30;
      if (zombieDistances[i] >= minThreshold &&
          zombieDistances[i] < MIN_ENGAGE_THRESHOLD &&
          zombieDistances[i] > bestOverrideDist) {
        // Check if not recently attempted
        bool recentlyAttempted = false;
        if (laneAttempted[i]) {
          unsigned long timeSinceAttempt = millis() - laneAttemptTime[i];
          if (timeSinceAttempt < ATTEMPT_COOLDOWN) {
            recentlyAttempted = true;
          }
        }
        if (!recentlyAttempted) {
          bestOverrideDist = zombieDistances[i];
          bestOverrideLane = i;
        }
      }
    }
  }

  // If we found a valid FORWARD target, use it (overrides sequence)
  if (bestOverrideLane >= 0) {
    // Recalculate sequence starting with this best target
    calculateNewSequence();
    commitToTarget(bestOverrideLane);
    if (isCommitted) {
      state = MOVE_TO_TARGET;
    }
    return;
  }

  // If sequence is locked, ONLY get targets from the sequence
  if (sequenceLocked && sequenceActive) {
    // Get next target from current sequence
    // Try up to SEQUENCE_SIZE times to find a valid target
    int attempts = 0;
    int nextLane = -1;
    bool foundValidTarget = false;

    while (attempts < SEQUENCE_SIZE && !foundValidTarget) {
      nextLane = getNextSequenceTarget();
      attempts++;

      if (nextLane >= 0) {
        // CRITICAL: NEVER target backward-moving zombies - they're retreating!
        if (ProxSensors[nextLane].direction == BACKWARD) {
          // Target is retreating - skip and get next from sequence
          continue;
        }

        // Check if lane has already been attempted
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

        // If recently attempted, skip this one and try next
        if (recentlyAttempted) {
          continue;
        }

        // Lane-specific threshold: lanes 2/3 no threshold, lanes 1/4 need 30%
        float minThreshold = (nextLane == 1 || nextLane == 2) ? 0.0 : 0.30;
        bool isValidStandard = (ProxSensors[nextLane].direction == FORWARD &&
                                zombieDistances[nextLane] >= minThreshold &&
                                zombieDistances[nextLane] < MIN_ENGAGE_THRESHOLD &&
                                !recentlyAttempted);

        if (isValidStandard) {
          // Target is valid - commit to it (sequence enforced)
          foundValidTarget = true;
        } else {
          // Target is invalid - continue loop to try next
          continue;
        }
      } else {
        // No more targets in sequence
        break;
      }
    }

    if (foundValidTarget && nextLane >= 0) {
      commitToTarget(nextLane);
      if (isCommitted) {
        state = MOVE_TO_TARGET;
      }
      return;
    }
    
    // Sequence exhausted but was locked - unlock and recalculate
    if (sequenceIndex >= SEQUENCE_SIZE) {
      sequenceLocked = false;
      sequenceActive = false;
    }
  }
  
  // Get next target from sequence (will calculate new sequence if needed)
  int nextLane = getNextSequenceTarget();
  
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
    sequenceActive = false;
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
        DBG_PRINT(F("~ARRIVE L"));
        DBG_PRINTLN(activeTargetIndex + 1);
      }
    }
  }
}

//============================================
// DWELL AT TARGET
//============================================
void dwellAtTarget() {
  // CRITICAL: Safety check - ensure activeTargetIndex is valid
  if (activeTargetIndex < 0 || activeTargetIndex > 3) {
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
    // CRITICAL: NEVER target backward-moving zombies - they're retreating!
    if (ProxSensors[i].direction == BACKWARD) continue;
    if (ProxSensors[i].direction != FORWARD) continue;  // Only forward-moving
    
    float dist = zombieDistances[i];
    // NEW SEMANTICS: Higher dist = closer to impact
    bool isExtremeEmergency = (dist > 0.92);  // Only extreme emergencies can override during cooldown
    
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
    if (sequenceLocked && sequenceActive) {
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
      // 2. Short lane getting very close (> 85%) while committed to long lane
      // 3. Extreme emergency (> 90% through)
      // 4. Both critical but new is significantly further through (at least 20% more)
      // NEW SEMANTICS: Higher dist = closer to impact
      if (isCritical && !laneIsCritical[activeTargetIndex]) {
        shouldOverride = true;  // Critical vs non-critical = always override
      } else if (isShortLane && committedIsLongLane && dist > 0.85) {
        shouldOverride = true;  // Short lane very close while at long lane
      } else if (dist > 0.90) {
        shouldOverride = true;  // Extreme emergency
      } else if (isCritical && laneIsCritical[activeTargetIndex] && dist > committedDist + 0.20) {
        shouldOverride = true;  // Both critical, new is 20%+ further through
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
      // NEW SEMANTICS: Higher dist = closer to impact (0% = at start, 100% = at impact)
      // So HIGH distance values mean close to impact (urgent!)
      // If L3 shows "83%" through lane, distance = 0.83 (urgent!)
      // We want to override when distance is HIGH (close to impact)

      // HIGHEST PRIORITY 0: ANY lane at 95%+ through lane = IMMEDIATE OVERRIDE
      // This is game-ending - zombie is about to impact!
      // NEW SEMANTICS: Higher dist = closer to impact
      if (dist > 0.95 && ProxSensors[i].direction == FORWARD && dist < 0.99) {
        // ANY lane at 95%+ through lane (about to impact!) - override immediately!
        shouldOverride = true;
      }
      // PRIORITY 1: L4 getting very close (90%+ through lane) = highest priority
      else if (i == 3 && dist > 0.90 && ProxSensors[i].direction == FORWARD && dist < 0.98) {
        // L4 at 90%+ through lane and moving forward - override immediately!
        shouldOverride = true;
      } else if (isShortLane && committedIsLongLane) {
        // Short lane at 50% through lane while at long lane = override immediately!
        // BUT: Check if L4 is also very close - if so, require short lane to be closer
        // NEW SEMANTICS: Higher dist = closer to impact
        bool l4AlsoClose = (zombieDistances[3] > 0.90 &&
                            ProxSensors[3].direction == FORWARD &&
                            zombieDistances[3] < 0.98);
        // NEW SEMANTICS: Higher dist = closer to impact
        if (l4AlsoClose && dist < 0.60) {
          shouldOverride = false;  // L4 is more urgent - short lane not close enough yet
        } else {
          if (isCritical || dist > 0.50) shouldOverride = true;
        }
      } else if (isShortLane && !committedIsLongLane) {
        // Short lane past 30% while at another lane = override
        // NEW SEMANTICS: Higher dist = closer to impact
        if (dist > 0.30) shouldOverride = true;
      } else if (isCritical && !laneIsCritical[activeTargetIndex]) {
        shouldOverride = true;
      } else if (dist > OVERRIDE_THRESHOLD[i]) {
        // NEW SEMANTICS: Higher dist = closer to impact
        shouldOverride = true;
      }
    }
    
    if (shouldOverride) {
      // PRIORITIZE BY TIME TO IMPACT (TTI) - which will hit first?
      // Use effective TTI to determine priority (lower TTI = will hit sooner = higher priority)
      float effectiveTTI = getEffectiveTTI(i);
      
      // If TTI is invalid, use a very high value (low priority)
      if (effectiveTTI >= 99999) {
        effectiveTTI = 999999;  // Very low priority
      }
      
      // Select the lane with the LOWEST TTI (will hit first)
      // If TTI is the same, use score as tiebreaker
      bool shouldSelect = false;
      if (overrideLane < 0) {
        // No override selected yet - select this one
        shouldSelect = true;
      } else {
        float currentTTI = getEffectiveTTI(overrideLane);
        if (currentTTI >= 99999) currentTTI = 999999;
        
        if (effectiveTTI < currentTTI) {
          // This lane will hit sooner - select it
          shouldSelect = true;
        } else if (effectiveTTI == currentTTI) {
          // Same TTI - use score as tiebreaker
          float thisScore = calculateThreatScore(i);
          float currentScore = calculateThreatScore(overrideLane);
          if (thisScore > currentScore) {
            shouldSelect = true;
          }
        }
      }
      
      if (shouldSelect) {
        overrideLane = i;
        bestOverrideScore = effectiveTTI;  // Store TTI for comparison
      }
    }
  }
  
  // EMERGENCY OVERRIDE while dwelling - but ONLY after MIN_DWELL_TIME has elapsed
  unsigned long earlyDwellTime = (arrivalTime > 0) ? (millis() - arrivalTime) : 0;
  if (overrideLane >= 0 && overrideLane <= 3 && earlyDwellTime >= MIN_DWELL_TIME && shouldOverride(overrideLane)) {
    int previousLane = activeTargetIndex;
    
    // CRITICAL: Clean up all dwell state variables before transitioning
    stoppedStartTime = 0;
    backwardStartTime = 0;
    peakZombieDistance = 0.0;  // Reset for next target (NEW SEMANTICS: 0 = at start)
    arrivalZombieDistance = 0.0;  // Reset for next target
    arrivalTime = 0;  // Reset for next target
    
    DBG_PRINT(F("!!! OVERRIDE L"));
    DBG_PRINT(overrideLane + 1);
    DBG_PRINT(F(" @"));
    if (overrideLane >= 0 && overrideLane <= 3) {
      DBG_PRINT((int)(zombieDistances[overrideLane] * 100));  // remaining % (consistent with LOCK)
    }
    DBG_PRINTLN(F("% !!!"));
    
    // Reset batch on emergency override
    resetSequence();
    
    releaseCommitment();
    commitToTarget(overrideLane);
    
    if (isCommitted) {
      state = MOVE_TO_TARGET;
      return;  // Exit dwell immediately to handle override
    }
  }
  
  // CRITICAL: Safety check - ensure arrivalTime is valid (avoid overflow)
  unsigned long dwellTime = (arrivalTime > 0) ? (millis() - arrivalTime) : 0;
  
  // CRITICAL: Safety check - ensure activeTargetIndex is valid before accessing arrays
  if (activeTargetIndex < 0 || activeTargetIndex > 3) {
    releaseCommitment();
    state = CHOOSE_TARGET;
    return;
  }
  
  float currentDist = zombieDistances[activeTargetIndex];
  int currentDir = ProxSensors[activeTargetIndex].direction;

  unsigned long maxDwell = (activeTargetIndex == 3) ? L4_DWELL_TIME : MAX_DWELL_TIME;

  // NEW SEMANTICS: Higher dist = closer to impact, so peak is the HIGHEST value
  if (currentDist > peakZombieDistance) {
    peakZombieDistance = currentDist;
  }
  
  // IMMEDIATE EXIT when target moves BACKWARD - cut dwell short!
  // No need to wait for MIN_DWELL_TIME - target retreating means hit registered
  if (currentDir == BACKWARD) {
    // Target has started moving backward - immediately record hit and move to next
    int previousLane = activeTargetIndex;  // Save for recordHit

    // CRITICAL: Clean up all dwell state variables before transitioning
    backwardStartTime = 0;
    stoppedStartTime = 0;
    peakZombieDistance = 0.0;  // Reset for next target (NEW SEMANTICS: 0 = at start)
    arrivalZombieDistance = 0.0;  // Reset for next target
    arrivalTime = 0;  // Reset for next target

    // Record hit with previous lane (before releaseCommitment clears it)
    if (previousLane >= 0 && previousLane <= 3) {
      zombiesKilled++;
      recordHit(previousLane);  // Analysis mode tracking
      Serial.print(F("HIT L"));
      Serial.print(previousLane + 1);
      Serial.print(F(" ["));
      Serial.print(zombiesKilled);
      Serial.println(F("]"));
    }

    releaseCommitment();

    // Advance sequence (will unlock if sequence complete)
    advanceSequence();

    // Get next target from sequence - IMMEDIATE transition, no delay
    int next = getNextSequenceTarget();
    if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
      commitToTarget(next);
      if (isCommitted) {
        state = MOVE_TO_TARGET;  // Immediately move to next target
      } else {
        // If commit failed, try chooseAndCommitTargetFromSequence for immediate retry
        state = CHOOSE_TARGET;
      }
    } else {
      // Sequence complete or no valid targets - unlock and immediately choose next
      if (sequenceLocked) {
        sequenceLocked = false;
      }
      // Immediately try to choose next target instead of waiting
      chooseAndCommitTargetFromSequence();
      if (isCommitted) {
        state = MOVE_TO_TARGET;
      } else {
        state = CHOOSE_TARGET;
      }
    }
    return;  // Exit immediately - no further dwell checks
  }
  
  // NEW SEMANTICS: Higher peakZombieDistance = got closer to impact
  if (dwellTime >= MIN_DWELL_TIME && peakZombieDistance > 0.85) {
    // Zombie got past 85% (close to impact)
    // Retreat = currentDist < peakZombieDistance (moved back toward start)
    float retreatAmount = peakZombieDistance - currentDist;
    if (retreatAmount > 0.04) {  // Moved back 4%+ from peak
      int previousLane = activeTargetIndex;  // Save for recordHit

      // CRITICAL: Clean up all dwell state variables before transitioning
      stoppedStartTime = 0;
      backwardStartTime = 0;
      peakZombieDistance = 0.0;  // Reset for next target (NEW SEMANTICS: 0 = at start)
      arrivalZombieDistance = 0.0;  // Reset for next target
      arrivalTime = 0;  // Reset for next target
      
      // Record hit with previous lane (before releaseCommitment clears it)
      if (previousLane >= 0 && previousLane <= 3) {
        zombiesKilled++;
        recordHit(previousLane);  // Analysis mode tracking
        DBG_PRINT(F("HIT L"));
        DBG_PRINT(previousLane + 1);
        DBG_PRINT(F(" ["));
        DBG_PRINT(zombiesKilled);
        DBG_PRINTLN(F("]"));
        
        if (!laneAttempted[previousLane]) {
          laneAttempted[previousLane] = true;
          laneAttemptTime[previousLane] = millis();
        }
      }
      
      releaseCommitment();
      advanceSequence();
      int next = getNextSequenceTarget();
      if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
        commitToTarget(next);
        if (isCommitted) state = MOVE_TO_TARGET;
        else {
          chooseAndCommitTargetFromSequence();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
        }
      } else {
        if (sequenceLocked) {
          sequenceLocked = false;
        }
        // Immediately try to choose next target
        chooseAndCommitTargetFromSequence();
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      }
      return;
    }
  }
  
  // NEW SEMANTICS: Lower dist = retreated toward start
  // Retreat confirmed when dist drops below threshold AND has retreated at least 15%
  if (dwellTime >= MIN_DWELL_TIME &&
      currentDist < RETREAT_CONFIRMED_DISTANCE &&
      currentDist < arrivalZombieDistance - 0.15) {
    int previousLane = activeTargetIndex;  // Save for recordHit
    
    // CRITICAL: Clean up all dwell state variables before transitioning
    stoppedStartTime = 0;
    backwardStartTime = 0;
    peakZombieDistance = 0.0;  // Reset for next target (NEW SEMANTICS: 0 = at start)
    arrivalZombieDistance = 0.0;  // Reset for next target
    arrivalTime = 0;  // Reset for next target
    
    // Record hit with previous lane (before releaseCommitment clears it)
    if (previousLane >= 0 && previousLane <= 3) {
      zombiesKilled++;
      recordHit(previousLane);  // Analysis mode tracking
      DBG_PRINT(F("HIT L"));
      DBG_PRINT(previousLane + 1);
      DBG_PRINT(F(" (far) ["));
      DBG_PRINT(zombiesKilled);
      DBG_PRINTLN(F("]"));
      
      if (!laneAttempted[previousLane]) {
        laneAttempted[previousLane] = true;
        laneAttemptTime[previousLane] = millis();
      }
    }
    
    releaseCommitment();
    int next = getNextSequenceTarget();
    if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
      commitToTarget(next);
      if (isCommitted) state = MOVE_TO_TARGET;
      else {
        chooseAndCommitTargetFromSequence();
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      }
    } else {
      chooseAndCommitTargetFromSequence();
      if (isCommitted) state = MOVE_TO_TARGET;
      else state = CHOOSE_TARGET;
    }
    return;
  }
  
  // Check for STOPPED direction OR very small velocity (pseudo-stopped)
  bool isEffectivelyStopped = (currentDir == STOPPED) || 
                               (abs(zombieVelocities[activeTargetIndex]) < 0.0001);
  
  // IMPROVED: Only check STOPPED exit after minimum persistent dwell time
  if (isEffectivelyStopped && dwellTime >= MIN_DWELL_TIME) {  // Use MIN_DWELL_TIME for consistency
    if (stoppedStartTime == 0) {
      stoppedStartTime = millis();
      stoppedStartDistance = currentDist;
    } else if (millis() - stoppedStartTime >= 300) {  // REDUCED: 300ms confirmed stopped
      // Check if distance barely changed - zombie truly stalled
      float distChange = abs(currentDist - stoppedStartDistance);
      if (distChange < 0.08) {  // Increased from 5% to 8% - more tolerant
        // If zombie was close (past 70%), probably a hit - be generous
        // NEW SEMANTICS: Higher peakZombieDistance = got closer to impact
        if (peakZombieDistance > 0.70) {
          int previousLane = activeTargetIndex;  // Save for recordHit

          // CRITICAL: Clean up all dwell state variables before transitioning
          stoppedStartTime = 0;
          backwardStartTime = 0;
          peakZombieDistance = 0.0;  // Reset for next target (NEW SEMANTICS: 0 = at start)
          arrivalZombieDistance = 0.0;  // Reset for next target
          arrivalTime = 0;  // Reset for next target
          
          // Record hit with previous lane (before releaseCommitment clears it)
          if (previousLane >= 0 && previousLane <= 3) {
            zombiesKilled++;
            recordHit(previousLane);
            DBG_PRINT(F("HIT L"));
            DBG_PRINT(previousLane + 1);
            DBG_PRINT(F(" (stop) ["));
            DBG_PRINT(zombiesKilled);
            DBG_PRINTLN(F("]"));
            
            // Mark as attempted and hit
            laneAttempted[previousLane] = true;
            laneAttemptTime[previousLane] = millis();
          }
          
          releaseCommitment();
          // IMMEDIATE transition - no delay
          int next = getNextSequenceTarget();
          if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
            commitToTarget(next);
            if (isCommitted) state = MOVE_TO_TARGET;
            else {
              chooseAndCommitTargetFromSequence();
              if (isCommitted) state = MOVE_TO_TARGET;
              else state = CHOOSE_TARGET;
            }
          } else {
            // Immediately try to choose next target
            chooseAndCommitTargetFromSequence();
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
            int previousLane = activeTargetIndex;  // Save for recordHit
            
            // CRITICAL: Clean up all dwell state variables before transitioning
            stoppedStartTime = 0;
            backwardStartTime = 0;
            peakZombieDistance = 1.0;  // Reset for next target
            arrivalZombieDistance = 1.0;  // Reset for next target
            arrivalTime = 0;  // Reset for next target
            
            DBG_PRINT(F("STOP L"));
            if (previousLane >= 0 && previousLane <= 3) {
              DBG_PRINTLN(previousLane + 1);
              
              // Mark as attempted - prevent immediate re-engagement
              laneAttempted[previousLane] = true;
              laneAttemptTime[previousLane] = millis();
            } else {
              DBG_PRINTLN(F("?"));
            }
            
            releaseCommitment();
            // IMMEDIATE transition - no delay
            int next = getNextSequenceTarget();
            if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
              commitToTarget(next);
              if (isCommitted) state = MOVE_TO_TARGET;
              else {
                chooseAndCommitTargetFromSequence();
                if (isCommitted) state = MOVE_TO_TARGET;
                else state = CHOOSE_TARGET;
              }
            } else {
              // Immediately try to choose next target
              chooseAndCommitTargetFromSequence();
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
      int previousLane = activeTargetIndex;  // Save for recordHit
      
      // CRITICAL: Clean up all dwell state variables before transitioning
      stoppedStartTime = 0;
      backwardStartTime = 0;
      peakZombieDistance = 1.0;  // Reset for next target
      arrivalZombieDistance = 1.0;  // Reset for next target
      arrivalTime = 0;  // Reset for next target
      
      DBG_PRINT(F("GONE L"));
      if (previousLane >= 0 && previousLane <= 3) {
        DBG_PRINT(previousLane + 1);
        DBG_PRINT(F(" @"));
        DBG_PRINT((int)((1.0 - currentDist) * 100));
        DBG_PRINTLN(F("%"));
        
        // CRITICAL FIX: Reset attempted flag when GONE - allow immediate re-engagement if target reappears
        // This allows the system to try again immediately if a new zombie appears in this lane
        laneAttempted[previousLane] = false;  // Clear attempted flag - allow re-engagement
      } else {
        DBG_PRINTLN(F("?"));
      }
      
      releaseCommitment();
      // IMMEDIATE transition - don't skip attempted lanes since we're not marking as attempted
      int next = getNextSequenceTarget();
      if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD) {
        commitToTarget(next);
        if (isCommitted) state = MOVE_TO_TARGET;
        else {
          chooseAndCommitTargetFromSequence();
          if (isCommitted) state = MOVE_TO_TARGET;
          else state = CHOOSE_TARGET;
        }
      } else {
        // Immediately try to choose next target
        chooseAndCommitTargetFromSequence();
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
    int previousLane = activeTargetIndex;  // Save for recordHit
    
    // CRITICAL: Clean up all dwell state variables before transitioning
    stoppedStartTime = 0;
    backwardStartTime = 0;
    peakZombieDistance = 0.0;  // Reset for next target (NEW SEMANTICS: 0 = at start)
    arrivalZombieDistance = 0.0;  // Reset for next target
    arrivalTime = 0;  // Reset for next target
    
    // IMPROVED: Mark lane as attempted - prevent immediate re-engagement
    if (previousLane >= 0 && previousLane <= 3) {
      laneAttempted[previousLane] = true;
      laneAttemptTime[previousLane] = millis();
      
      if (currentDir == FORWARD && currentDist < arrivalZombieDistance - 0.03) {
        zombiesMissed++;
        DBG_PRINT(F("MISS L"));
        DBG_PRINT(previousLane + 1);
        DBG_PRINT(F(" ["));
        DBG_PRINT(zombiesMissed);
        DBG_PRINTLN(F("]"));
      } else if (currentDir == STOPPED) {
        // STOPPED at very close range = likely hit wall = miss
        if (currentDist < 0.08) {
          // Could be a miss
        }
        DBG_PRINT(F("STOP L"));
        DBG_PRINTLN(previousLane + 1);
      } else {
        DBG_PRINT(F("TIME L"));
        DBG_PRINTLN(previousLane + 1);
      }
    }
    
    releaseCommitment();
    // IMMEDIATE transition - no delay (skip attempted lanes)
    int next = getNextSequenceTarget();
    if (next >= 0 && next <= 3 && ProxSensors[next].direction == FORWARD && !laneAttempted[next]) {
      commitToTarget(next);
      if (isCommitted) state = MOVE_TO_TARGET;
      else {
        chooseAndCommitTargetFromSequence();
        if (isCommitted) state = MOVE_TO_TARGET;
        else state = CHOOSE_TARGET;
      }
    } else {
      // Immediately try to choose next target (skip attempted lanes)
      chooseAndCommitTargetFromSequence();
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
      resetSequence();  // Reset sequence system for new game
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
      resetSequence();  // Reset batch system
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
        // CRITICAL: Prevent manual lane selection during calibration
        if (calibrationActive) {
          Serial.println(F("Calibration active - cannot change lane"));
          break;
        }
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
      // Show current calibration values and verify they're being used
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
        Serial.print(zombieDistances[i], 3);
        Serial.print(F(" ("));
        Serial.print((int)(zombieDistances[i] * 100));
        Serial.println(F("% through lane)"));
        
        // Verify the distance calculation is using the calibrated range
        float calcRange = (float)(ProxRange[i][0] - ProxRange[i][1]);
        if (calcRange > 0) {
          float expectedDist = (ProxSensors[i].currVal - ProxRange[i][1]) / calcRange;
          Serial.print(F("  -> Using range "));
          Serial.print(calcRange);
          Serial.print(F(" to calculate distance"));
          Serial.println();
        }
      }
      if (calibrationActive) {
        Serial.print(F("Calibrating... "));
        Serial.print((millis() - calibrationStartTime) / 1000);
        Serial.println(F("s"));
      } else {
        Serial.println(F("Calibration complete - ranges are active"));
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
// Calculates Time To Impact using velocity and REMAINING distance to impact
// TTI = (1.0 - currentPosition) / velocity
// Lower TTI = closer to impact = higher priority
//============================================
void updateTTI() {
  for (int i = 0; i < 4; i++) {
    // Calculate velocity: positive = moving TOWARD impact (distance increasing)
    // zombieDistances: 0% = start, 100% = impact
    float distChange = zombieDistances[i] - prevZombieDistances[i];  // Positive when approaching impact
    float instantVel = distChange / (float)TTI_UPDATE_INTERVAL;  // velocity in dist%/ms

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

    // Calculate TTI: REMAINING DISTANCE / VELOCITY
    // Remaining distance = 1.0 - zombieDistances[i] (how far until impact)
    // Only calculate if moving FORWARD (positive velocity) with meaningful speed
    // Minimum velocity threshold: 0.0001 dist/ms = 10% distance per second
    float remainingDist = 1.0 - zombieDistances[i];  // Distance left until impact

    if (zombieVelocities[i] > 0.0001 && ProxSensors[i].direction == FORWARD && remainingDist > 0) {
      // TTI in milliseconds = remaining distance / velocity
      timeToImpact[i] = remainingDist / zombieVelocities[i];
    } else if (zombieVelocities[i] <= 0 || ProxSensors[i].direction == BACKWARD) {
      // Moving backward or stopped - set very high TTI (not urgent)
      timeToImpact[i] = 99999;
    } else if (remainingDist <= 0) {
      // Already at or past impact - extremely urgent!
      timeToImpact[i] = 0;
    } else {
      // Very slow movement - estimate based on minimum expected velocity
      // Assume worst case: target will reach impact eventually
      timeToImpact[i] = remainingDist / 0.0001;  // Conservative estimate
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
// NEW SEMANTICS: Higher dist = closer to impact = higher threat
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
    // Only score stopped targets if they're close to impact (> 80%)
    if (dist < 0.80) return 0;
    return dist * 50 * LANE_PRIORITY[lane];
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
    // Distance-based scoring when TTI unavailable
    // NEW SEMANTICS: Higher dist = more urgent
    if (dist > 0.90) {
      score = 800 + (dist - 0.90) * 2000;
    } else if (dist > 0.80) {
      score = 400 + (dist - 0.80) * 4000;
    } else if (dist > 0.65) {
      score = 150 + (dist - 0.65) * 1700;
    } else {
      score = 30 + dist * 80;
    }
  }
  int dynamicTravel = getDynamicTravelTime(lane);
  score -= dynamicTravel * 0.15;
  if (!canReachInTime(lane) && score < 500) {
    score *= 0.5;
  }

  //============================================
  // Lane-specific bonuses
  // NEW SEMANTICS: Higher dist = closer to impact
  if (lane == 1 || lane == 2) {
    // Short lanes (L2/L3) - bonus for being further through
    if (dist > 0.70) {
      score += 300 + (dist - 0.70) * 667;
    } else if (dist > 0.40) {
      score += 150 + (dist - 0.40) * 500;
    } else {
      score += 50 + dist * 250;
    }
    // Critical distance bonus
    if (dist > SHORT_LANE_CRITICAL_DISTANCE) {
      score += (dist - SHORT_LANE_CRITICAL_DISTANCE) * 5000;
    } else if (dist > 0.65) {
      score += (dist - 0.65) * 2000;
    } else if (dist > 0.45) {
      score += (dist - 0.45) * 500;
    } else if (dist > 0.20) {
      score += (dist - 0.20) * 300;
    }
    // No bonus for early targets - prioritize targets closer to impact
  } else {
    // Long lanes (L1/L4)
    if (dist > LONG_LANE_CRITICAL_DISTANCE) {
      score += (dist - LONG_LANE_CRITICAL_DISTANCE) * 3000;
    } else if (dist > 0.70) {
      score += (dist - 0.70) * 1500;
    } else if (dist > 0.55) {
      score += (dist - 0.55) * 400;
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

    float currentDistance = 0.0f;
    // Distance calculation: Uses RAW sensor value for accurate calibration
    // ProxRange[0] = high reading = target at START (close to sensor)
    // ProxRange[1] = low reading = target at IMPACT (far from sensor)
    // Formula: 0% at start (rawVal=high), 100% at impact (rawVal=low)
    // normalized = (ProxRange[0] - rawVal) * proxRangeScaled (pre-multiplied with lane scaling)
    float normalized = (float)(ProxRange[i][0] - rawVal) * proxRangeScaled[i];
    currentDistance = constrain(normalized, 0.0f, 1.0f);
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
      
      // STABILITY FILTER: Require 4+ consecutive BACKWARD readings
      // This prevents F-B-F flicker from falsely marking as BACKWARD
      int backwardThreshold = 4;

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
  // CRITICAL: During calibration, force mechanism to stay at encoder home (position 0)
  if (calibrationActive) {
    desiredPosition = 0;
  }
  
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
  #ifdef DEBUG_SERIAL
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
    // Display: 0% = at start (beginning), 100% = at impact (end)
    // zombieDistances[i] = 0.0 at start, 1.0 at impact
    int pct = (int)(zombieDistances[i] * 100);
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
  #endif
}
