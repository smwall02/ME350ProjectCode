#include <Encoder.h>
#include <EEPROM.h>

// Per-lane state machine to reduce thrash and favor forward threats
enum LaneTargetState { LANE_SAFE = 0, LANE_APPROACHING, LANE_BEING_SERVICED, LANE_RECOVERING };
LaneTargetState laneTargetState[4] = {LANE_SAFE, LANE_SAFE, LANE_SAFE, LANE_SAFE};

//============================================
// DEBUG OPTIMIZATION - Comment out to save flash memory
//============================================
// Uncomment the line below to enable debug Serial output (disabled by default to save flash)
// #define DEBUG_SERIAL  // ENABLED to surface lane tracking logs

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

// Game modes
enum GameMode {
  MODE_STABLE,
  MODE_TURBO,
  MODE_HARDCORE,
  MODE_MANUAL
};

GameMode currentMode = MODE_STABLE;

// Secondary system state machine with explicit AIM/FIRE dwell handling
enum SystemState {
  ST_BOOT = 0,
  ST_HOME,
  ST_IDLE,
  ST_SELECT_LANE,
  ST_AIM,
  ST_FIRE_DWELL,
  ST_PAUSE,
  ST_ERROR
};

SystemState systemState = ST_BOOT;

// Core globals for stateful control
volatile long motorPositionCounts = 0;
float motorVelocityCPS = 0.0f;
float laneAnglesDeg[4] = {0, 0, 0, 0};
int currentLane = -1;
float targetAngleDeg = 0.0f;
float laneTTI[4] = {9999, 9999, 9999, 9999};
unsigned long lastVelUpdateMicros = 0;
unsigned long dwellStartMillis = 0;
const unsigned long SENSOR_DWELL_TIME_MS = 350;  // Ensure 350ms on-target dwell

// Hardware helpers
const int LASER_PIN = 4;
const float COUNTS_PER_DEGREE = 11.4f;  // Approximate conversion for motor linkage
bool pauseClearedByCommand = false;
bool gameRunning = false;
int requestedManualLane = -1;
bool homingStarted = false;

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

// HYSTERESIS for target selection to avoid rail thrash
const float SWITCH_TTI_MARGIN = 200.0f;           // Require 200 ms improvement to switch
const unsigned long MIN_SERVICE_TIME = 350;       // Stay on a lane briefly before switching
int lastSelectedLane = -1;                        // Track last chosen lane for hysteresis
float lastSelectedTTI = 99999;                    // Track last chosen lane's effective TTI
unsigned long lastSelectionTime = 0;
const float DANGER_ZONE_DISTANCE = 0.82;          // Emergency promotion threshold

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
const float MAX_ENGAGE_DISTANCE = 0.65;  // Do not plan hits past 65% through a lane

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
const float OVERRIDE_THRESHOLD[4] = {0.70, 0.45, 0.45, 0.70};  // Override when > this % through lane
const float ABSOLUTE_OVERRIDE_DISTANCE = 0.75;  // Critical when > 75% through lane
const float SHORT_LANE_CRITICAL_DISTANCE = 0.35;  // L2/L3 critical at 35% through lane
const float LONG_LANE_CRITICAL_DISTANCE = 0.60;  // L1/L4 critical at 60% through lane
const float TARGET_ZONE_MAX = 0.40;  // Desired "comfort" zone: keep zombies under 40%
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
unsigned long lastCalibrationLogTime = 0;

// Track min/max readings per lane during calibration
int calibrationMin[4] = {1023, 1023, 1023, 1023};  // Impact (low reading)
int calibrationMax[4] = {0, 0, 0, 0};              // Start (high reading)
int calibrationStart[4] = {0, 0, 0, 0};            // Initial reading at calibration start (= 0%)
int calibrationRaw[4] = {0, 0, 0, 0};              // Latest raw readings during calibration
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
const unsigned long MIN_VEL_COMP_TIME = 5000;  // micros between velocity updates
const int MIN_VEL_COMP_COUNT = 2;              // encoder counts needed for update

//============================================
// TTI TRACKING
//============================================
float zombieVelocities[4] = {0, 0, 0, 0};
float timeToImpact[4] = {99999, 99999, 99999, 99999};
unsigned long lastTTIUpdate = 0;
const unsigned long TTI_UPDATE_INTERVAL = 25;  // Lowered from 40ms for faster updates
float prevZombieDistances[4] = {1.0, 1.0, 1.0, 1.0};
float proxSpeeds[4] = {0, 0, 0, 0};            // Raw proximity delta per ms
float laneTTI[4] = {99999, 99999, 99999, 99999};
float prevSmoothProx[4] = {0, 0, 0, 0};
int lastRawReading[4] = {0, 0, 0, 0};          // Previous raw reading for speed
unsigned long lastRawUpdate[4] = {0, 0, 0, 0}; // Timestamp of last raw update

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

// ATTEMPTED LANES TRACKING - Prevent re-engaging lanes already attempted
bool laneAttempted[4] = {false, false, false, false};  // Track which lanes we've already attempted
unsigned long laneAttemptTime[4] = {0, 0, 0, 0};       // When we attempted each lane
const unsigned long ATTEMPT_COOLDOWN = 1000;           // Allow faster re-engagement

// Forward declarations for helpers used in forward targeting
float getEffectiveTTI(int lane);
float getEarlyEngageThreshold(int lane);

//============================================
// FORWARD PRIORITY HELPER
// Ensures we always pick the forward-moving target closest to impact by TTI
//============================================
int getMostAdvancedForwardLane(float &bestTTIOut, float &bestDistOut) {
  int bestLane = -1;
  float bestDist = -1.0f;
  float bestTTI = 99999.0f;
  long bestTravel = 32767;

  // Two-pass: prefer approaching/serviced lanes, then fall back to any forward lane if none found
  for (int pass = 0; pass < 2 && bestLane < 0; pass++) {
    for (int i = 0; i < 4; i++) {
      // HARD GUARD: never select backward or stopped lanes here
      if (ProxSensors[i].direction != FORWARD) continue;

      if (pass == 0) {
        if (laneTargetState[i] != LANE_APPROACHING && laneTargetState[i] != LANE_BEING_SERVICED) continue;
      }

      float dist = zombieDistances[i];
      float laneThreshold = getEarlyEngageThreshold(i);
      if (dist < laneThreshold || dist > MAX_ENGAGE_DISTANCE) continue;  // Ignore out-of-window targets

      // Respect recent attempt cooldown to avoid thrashing
      if (laneAttempted[i]) {
        unsigned long timeSinceAttempt = millis() - laneAttemptTime[i];
        if (timeSinceAttempt < ATTEMPT_COOLDOWN) continue;
      }

      float effectiveTTI = getEffectiveTTI(i);
      bool hasValidTTI = effectiveTTI < 99999;
      long travelTicks = labs(cachedEncoderPos - targetPositions[i]);

      bool choose = false;

      // PRIMARY: pick the target furthest forward (highest dist)
      if (dist > bestDist + 0.001f) {
        choose = true;
      } else if (abs(dist - bestDist) <= 0.001f) {
        // Tie-breaker: prefer lower TTI if both are similarly forward
        if (hasValidTTI && (effectiveTTI + 0.001f < bestTTI - 0.001f)) {
          choose = true;
        } else if (hasValidTTI && abs(effectiveTTI - bestTTI) < 0.001f) {
          // Final tie-breaker: shorter travel for faster slew
          choose = (travelTicks < bestTravel);
        }
      }

      if (choose) {
        bestTTI = hasValidTTI ? effectiveTTI : bestTTI;
        bestDist = dist;
        bestLane = i;
        bestTravel = travelTicks;
      }
    }
  }

  bestTTIOut = bestTTI;
  bestDistOut = bestDist;
  return bestLane;
}

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

const unsigned long MIN_DWELL_TIME = 350;  // Ensure at least 350ms on target
const unsigned long NORMAL_DWELL_TIME = 350;
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
void logSequence(const char *reason);
void updateCalibration();
void applyCalibration();
void stopMotor();
void setMotorVoltage(float voltage);
void recordHit(int lane);
void updateProxScaling();
void initHardware();
void encoderISR();
void setMotorPWM(int pwm);
void setMotorDirection(int dir);
bool isHomeSwitchActive();
void laserOn();
void laserOff();
float motorCountsToDegrees(long counts);
long motorDegreesToCounts(float deg);
void updateMotorVelocity(unsigned long nowMicros);
void updateThreatModel(unsigned long nowMillis);
int pickMostDangerousLane();
void readSerialCommands();
bool atTargetAngle();
void runStateMachine(unsigned long nowMillis);
void handleBootState();
void handleHomeState();
void handleIdleState();
void handleSelectLaneState();
void handleAimState();
void handleFireDwellState(unsigned long nowMillis);
void handlePauseState();
void handleErrorState();
void updateMotorControl(unsigned long nowMicros);

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
  pinMode(LASER_PIN, OUTPUT);
  initHardware();

  Serial.begin(115200);
  Serial.println(F("ME350 Zombie Defense v9"));
  
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
    lastRawReading[i] = ProxSensors[i].currVal;
    lastRawUpdate[i] = millis();
    prevSmoothProx[i] = ProxSensors[i].smoothVal;
    laneTTI[i] = 99999;

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
  Serial.println(F("Cmds: T/Y start, S stop, H home, M mode, 1-4 lane, C1-4 cap, X calib, P/W/R/D/?"));
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

  int previousLane = committedLane;
  float dist = zombieDistances[lane];

  if (ProxSensors[lane].direction == BACKWARD) return;
  // NEW SEMANTICS: Higher dist = more urgent (closer to impact)
  bool isEmergency = (dist > ABSOLUTE_OVERRIDE_DISTANCE && ProxSensors[lane].direction == FORWARD);
  if (!isEmergency) {
    float laneThreshold = getEarlyEngageThreshold(lane);
    // Valid range: dist > laneThreshold and below planned engage ceiling
    if (dist < laneThreshold || dist > MAX_ENGAGE_DISTANCE) return;
    if (ProxSensors[lane].direction != FORWARD) return;
    bool isCritical = laneIsCritical[lane];

    float closestDist = 0.0;  // Track highest dist (closest to impact)
    int closestLane = -1;

    for (int i = 0; i < 4; i++) {
      // Only consider forward-moving targets in valid range - use lane-specific threshold
      if (i != lane &&
          ProxSensors[i].direction == FORWARD &&
          zombieDistances[i] > getEarlyEngageThreshold(i) &&
          zombieDistances[i] < MAX_ENGAGE_DISTANCE) {
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
  }  // End of !isEmergency block

  isCommitted = true;
  // Track target switch for cooldown period
  if (committedLane >= 0 && committedLane != lane) {
    lastTargetSwitchTime = millis();  // Record when we switched targets
  }
  
  committedLane = lane;
  commitStartTime = millis();
  commitStartDistance = zombieDistances[lane];
  lastCommitTime = millis();
  lastSelectedLane = lane;
  lastSelectedTTI = getEffectiveTTI(lane);
  lastSelectionTime = millis();
  
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
// Target selection already handles cycling; overrides are intentionally disabled
//============================================
bool shouldOverride(int newLane) {
  // OVERRIDES DISABLED - always return false
  // User explicitly requested no overrides
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
    // Respect the planned engage ceiling to avoid flirting with impact
    float laneThreshold = getEarlyEngageThreshold(i);
    if (dist < laneThreshold || dist > MAX_ENGAGE_DISTANCE) continue;
    
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
    Serial.print(F("CAL DONE Ranges:"));
    for (int i = 0; i < 4; i++) {
      Serial.print(F(" L"));
      Serial.print(i + 1);
      Serial.print(F("=["));
      Serial.print(ProxRange[i][0]);
      Serial.print(F(","));
      Serial.print(ProxRange[i][1]);
      Serial.print(F("]"));
    }
    Serial.println();
    return;
  }
  
  // Update min/max for each lane based on RAW analog readings (not smoothed!)
  // CRITICAL: Must use raw analogRead() to capture true min/max values
  // Calibration mapping (sensor is mounted near lane start and reads HIGH there):
  // - Start (0%): HIGH sensor reading (target close to sensor) = calibrationMax
  // - Impact (100%): LOW sensor reading (target far from sensor) = calibrationMin
  for (int i = 0; i < 4; i++) {
    // Use raw analog reading directly - smoothed values won't capture true extremes
    int rawVal = analogRead(ProxSensors[i].pin);
    calibrationRaw[i] = rawVal;  // Capture latest raw value for logging

    // Track minimum (impact/100% = lowest sensor reading when target is far from the sensor)
    // Track maximum (start/0% = highest sensor reading when target is at the sensor)
    // Sensor behavior: HIGH value = target at start (near sensor), LOW value = target at impact (far from sensor)
    
    if (rawVal < calibrationMin[i]) {
      calibrationMin[i] = rawVal;  // Minimum = target at impact/100% (lowest reading)
      calibrationUpdated[i] = true;
    }
    if (rawVal > calibrationMax[i]) {
      calibrationMax[i] = rawVal;  // Maximum = target at start/0% (highest reading, may be slightly +/-)
      calibrationUpdated[i] = true;
    }
  }

  // Periodically log raw readings to verify orientation and movement during calibration
  const unsigned long CALIBRATION_LOG_INTERVAL = 1000;  // ms
  if (elapsed - lastCalibrationLogTime >= CALIBRATION_LOG_INTERVAL) {
    Serial.print(F("[CAL] t="));
    Serial.print(elapsed / 1000);
    Serial.print(F("s raw/min/max -> "));
    for (int i = 0; i < 4; i++) {
      Serial.print(F("L"));
      Serial.print(i + 1);
      Serial.print(F("("));
      Serial.print(calibrationRaw[i]);
      Serial.print(F("/"));
      Serial.print(calibrationMin[i]);
      Serial.print(F("/"));
      Serial.print(calibrationMax[i]);
      Serial.print(F(") "));
    }
    Serial.println();
    lastCalibrationLogTime = elapsed;
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

    // If readings are inverted (impact >= start) but we captured a span, swap so 0% stays near the sensor
    if (impactVal >= startVal && calibrationMax[i] != calibrationMin[i]) {
      Serial.print(F("  L"));
      Serial.print(i + 1);
      Serial.print(F(": detected inverted readings (impact>=start); swapping to keep 0% near sensor"));
      Serial.println();
      int swappedStart = impactVal;
      int swappedImpact = startVal;
      startVal = swappedStart;
      impactVal = swappedImpact;
      range = startVal - impactVal;
    }

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
      Serial.print(F(")"));
      if (impactVal >= startVal) {
        Serial.print(F(" [expected IMPACT < START; verify sensor orientation]"));
      }
      Serial.println(F(", keeping default"));
    }
  }

  updateProxScaling();

  // No additional normalization needed - we used calibrationStart (captured at begin) as 0%
  // This ensures targets at their initial position = exactly 0%

  Serial.println(F("Ranges in use (far,close):"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F(" L"));
    Serial.print(i + 1);
    Serial.print(F("=["));
    Serial.print(ProxRange[i][0]);
    Serial.print(F(","));
    Serial.print(ProxRange[i][1]);
    Serial.print(F("]"));
  }
  Serial.println();
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
  lastCalibrationLogTime = 0;

  // CRITICAL: Move mechanism to encoder home (position 0) and keep it there during calibration
  desiredPosition = 0;
  systemEnabled = true;  // Ensure PID controller is active to maintain position
  
  // Reset calibration tracking
  // CRITICAL: Calibration starts when targets are at 0% (beginning of lane)
  // At start (0%): sensor reads HIGH (target close to sensor) = calibrationStart
  // At impact (100%): sensor reads LOW (target far from sensor) = calibrationMin
  Serial.println(F("=== CALIBRATION STARTED (10s) ==="));
  Serial.println(F("Capturing initial RAW readings as 0% baseline..."));
  Serial.println(F("Expectation: 0% starts near the prox sensor (HIGH reading); move targets to impact for LOW readings."));
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
  unsigned long nowMicros = micros();
  unsigned long now = millis();
  executionDuration = nowMicros - lastExecutionTime;
  lastExecutionTime = nowMicros;

  processSerialCommands();
  readSerialCommands();
  
  // OPTIMIZATION: Cache encoder position to avoid multiple reads
  if (now - lastEncoderRead >= ENCODER_CACHE_INTERVAL) {
    cachedEncoderPos = encoder.read();
    lastEncoderRead = now;
  }

  updateMotorVelocity(nowMicros);
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

  updateLaneStates();
  
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

  updateThreatModel(now);
  
  //============================================
  // HIGH-LEVEL STATE MACHINE
  //============================================
  runStateMachine(now);

    //============================================
  // MOTOR CONTROL
  //============================================
  if (systemEnabled && digitalRead(ON_OFF_SWITCH_PIN) == HIGH) {
    updateMotorControl(nowMicros);
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

// End of main loop


//============================================
// CHOOSE AND COMMIT TO TARGET
//============================================
void chooseAndCommitTarget() {
  // Always prioritize smallest positive TTI, with emergency danger handling and hysteresis
  int bestLane = -1;
  float bestTTI = 99999.0f;
  float bestDist = 0.0f;
  int dangerLane = -1;
  float dangerDist = 0.0f;

  updatePendingQueue();  // Clean stale queue entries

  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction != FORWARD) continue;

    float dist = zombieDistances[i];
    float laneThreshold = getEarlyEngageThreshold(i);
    if (dist < laneThreshold || dist > MAX_ENGAGE_DISTANCE) continue;

    float effectiveTTI = getEffectiveTTI(i);
    laneTTI[i] = effectiveTTI;

    if (dist >= DANGER_ZONE_DISTANCE && effectiveTTI < 99999) {
      if (dist > dangerDist) {
        dangerLane = i;
        dangerDist = dist;
      }
    }

    if (effectiveTTI > 0 && effectiveTTI < bestTTI) {
      bestTTI = effectiveTTI;
      bestLane = i;
      bestDist = dist;
    } else if (effectiveTTI >= 0 && abs(effectiveTTI - bestTTI) < 1.0f && dist > bestDist) {
      // Tie-breaker: furthest forward
      bestLane = i;
      bestDist = dist;
    }
  }

  // Promote imminent danger regardless of minor TTI differences
  if (dangerLane >= 0) {
    bestLane = dangerLane;
    bestTTI = getEffectiveTTI(dangerLane);
  }

  if (bestLane >= 0 && lastSelectedLane >= 0 && lastSelectedLane != bestLane) {
    float lastTTI = getEffectiveTTI(lastSelectedLane);
    bool lastValid = ProxSensors[lastSelectedLane].direction == FORWARD &&
                     zombieDistances[lastSelectedLane] > getEarlyEngageThreshold(lastSelectedLane) &&
                     zombieDistances[lastSelectedLane] < MAX_ENGAGE_DISTANCE;
    bool servedRecently = (millis() - lastSelectionTime) < MIN_SERVICE_TIME;
    bool meaningfullyBetter = (lastTTI - bestTTI) > SWITCH_TTI_MARGIN;
    if (lastValid && (!meaningfullyBetter || servedRecently)) {
      bestLane = lastSelectedLane;
      bestTTI = lastTTI;
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
      systemState = ST_AIM;
      lastSelectedLane = bestLane;
      lastSelectedTTI = bestTTI;
      lastSelectionTime = millis();
      targetAngleDeg = motorCountsToDegrees(targetPositions[bestLane]);
    }
  } else {
    // No targets - go to wait position
    desiredPosition = WAIT_POSITION;
    WAIT_POS = true;
    activeTargetIndex = -1;
    releaseCommitment();
    systemState = ST_IDLE;
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
      laserOn();
      dwellStartMillis = millis();
      systemState = ST_FIRE_DWELL;
      targetAngleDeg = motorCountsToDegrees(desiredPosition);
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
  float committedDist = zombieDistances[activeTargetIndex];
  float distanceGap = abs(dist - committedDist);

  // Prevent switching when targets are at similar distances
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

    // Immediately pick the next best target
    state = CHOOSE_TARGET;
    chooseAndCommitTarget();
    if (isCommitted) {
      state = MOVE_TO_TARGET;
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
      // Immediately pick the next best target
      state = CHOOSE_TARGET;
      chooseAndCommitTarget();
      if (isCommitted) state = MOVE_TO_TARGET;
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
  state = CHOOSE_TARGET;
  chooseAndCommitTarget();
  if (isCommitted) state = MOVE_TO_TARGET;
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
          state = CHOOSE_TARGET;
          chooseAndCommitTarget();
          if (isCommitted) state = MOVE_TO_TARGET;
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
            state = CHOOSE_TARGET;
            chooseAndCommitTarget();
            if (isCommitted) state = MOVE_TO_TARGET;
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
      state = CHOOSE_TARGET;
      chooseAndCommitTarget();
      if (isCommitted) state = MOVE_TO_TARGET;
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
  state = CHOOSE_TARGET;
  chooseAndCommitTarget();
  if (isCommitted) state = MOVE_TO_TARGET;
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
  lastSelectedLane = -1;
  lastSelectedTTI = 99999;
  lastSelectionTime = 0;
  for (int i = 0; i < 4; i++) laneTargetState[i] = LANE_SAFE;
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
    proxSpeeds[i] = 0;
  }
  lastTTIUpdate = millis();
}

//============================================
// PER-LANE STATE TRACKING
// Keeps lightweight intent per rail to cut thrash
//============================================
void updateLaneStates() {
  for (int i = 0; i < 4; i++) {
    float dist = zombieDistances[i];
    int dir = ProxSensors[i].direction;
    float safeThreshold = (i == 3) ? L4_GONE_DISTANCE : ((i == 1 || i == 2) ? L2_L3_GONE_DISTANCE : ZOMBIE_GONE_DISTANCE);

    LaneTargetState nextState = laneTargetState[i];

    if (isCommitted && committedLane == i) {
      nextState = LANE_BEING_SERVICED;
    } else if (dir == BACKWARD) {
      nextState = LANE_RECOVERING;
    } else if (dir == FORWARD) {
      nextState = LANE_APPROACHING;
    } else if (dist <= safeThreshold) {
      nextState = LANE_SAFE;
    }

    laneTargetState[i] = nextState;
  }
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
    float instantVel = distChange / (float)TTI_UPDATE_INTERVAL;      // velocity in dist%/ms

    // Derive velocity directly from raw sensor delta to improve TTI accuracy
    float spanCounts = max(1.0f, (float)(ProxRange[i][0] - ProxRange[i][1]));
    float railVel = -(proxSpeeds[i]) / spanCounts;  // Normalize and flip sign so forward = +

    // Use filtered proximity deltas per rail to stabilize speed estimation
    float filteredCountsDelta = prevSmoothProx[i] - ProxSensors[i].smoothVal;  // forward => positive
    float filteredRailVel = (filteredCountsDelta * proxRangeScaled[i]) / (float)TTI_UPDATE_INTERVAL;
    prevSmoothProx[i] = ProxSensors[i].smoothVal;

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

    // Fuse distance-derived median velocity with filtered sensor velocity for robustness
    float fusedVel = 0.5f * filteredRailVel + 0.3f * railVel + 0.2f * medianVel;

    // Smooth the velocity (EMA filter)
    zombieVelocities[i] = velocityAlpha * zombieVelocities[i] + (1 - velocityAlpha) * fusedVel;

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
    laneTTI[i] = timeToImpact[i];
    prevZombieDistances[i] = zombieDistances[i];
  }
}

//============================================
// GET EFFECTIVE TTI
// Uses DYNAMIC travel time based on current motor position
// L2/L3 get BOOST (lower effective TTI = more urgent)
//============================================
float getEffectiveTTI(int lane) {
  // Immediately de-prioritize backward or stopped movement
  if (ProxSensors[lane].direction != FORWARD) return 99999;

  float baseTTI = (laneTTI[lane] < 99999) ? laneTTI[lane] : timeToImpact[lane];
  if (baseTTI >= 99999) return 99999;
  int dynamicTravel = getDynamicTravelTime(lane);
  float effectiveTTI = baseTTI - dynamicTravel;
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
  float zoneError = max(0.0f, dist - TARGET_ZONE_MAX);  // Pressure once past the comfort zone
  float zoneWeight = (lane == 1 || lane == 2) ? 1600.0f : 1200.0f;
  score += zoneError * zoneWeight;
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
    unsigned long rawNow = now;
    unsigned long rawDeltaT = rawNow - lastRawUpdate[i];
    if (rawDeltaT == 0) rawDeltaT = 1;  // Prevent division by zero
    proxSpeeds[i] = (float)(rawVal - lastRawReading[i]) / (float)rawDeltaT;  // counts per ms
    lastRawReading[i] = rawVal;
    lastRawUpdate[i] = rawNow;

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
      // Moving forward toward impact (sensor value decreasing = target moving away from the sensor)
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
      // Moving backward toward the sensor (sensor value increasing)
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

  // Softened hop for adjacent lanes to reduce gear stress
  bool adjacentHop = (isCommitted && committedLane >= 0 && previousTargetIndex >= 0 &&
                      abs(committedLane - previousTargetIndex) == 1);
  float voltageLimit = SUPPLY_VOLTAGE;
  float frictionScale = 1.0f;
  if (adjacentHop) {
    long hopDistance = labs(targetPositions[committedLane] - targetPositions[previousTargetIndex]);
    // For small adjacent moves, trim voltage and friction compensation
    if (hopDistance < 350) {
      voltageLimit = min(voltageLimit, SUPPLY_VOLTAGE * 0.75f);
      frictionScale = 0.75f;

      // As we get close to the target, soften further to avoid overshoot and tooth skipping
      if (abs(positionError) < 200) {
        voltageLimit = min(voltageLimit, SUPPLY_VOLTAGE * 0.60f);
        frictionScale = 0.65f;
      }
    }
  }

  int effectiveBand = (activeTargetIndex == 3) ? 12 : TARGET_BAND;
  int effectiveVelThreshold = (activeTargetIndex == 3) ? 50 : 80;

  if (abs(positionError) <= effectiveBand && abs(motorVelocity) < effectiveVelThreshold) {
    stopMotor(); errorIntegral = 0; return;
  }
  
  errorIntegral += positionError * (float)executionDuration / 1000000.0;
  errorIntegral = constrain(errorIntegral, -1000, 1000);
  
  float velocityError = 0 - motorVelocity;
  float voltage = KP * positionError + KI * errorIntegral + KD * velocityError;

  if (positionError < -5) voltage -= FRICTION_LEFT * frictionScale;
  else if (positionError > 5) voltage += FRICTION_RIGHT * frictionScale;

  if (abs(voltage) > voltageLimit) {
    errorIntegral -= positionError * (float)executionDuration / 1000000.0;
    voltage = constrain(voltage, -voltageLimit, voltageLimit);
  }

  setMotorVoltage(voltage);
}

// Helper wrappers for state-machine driven control
void initHardware() {
  laserOff();
}

void encoderISR() {
  // Encoder library handles updates; placeholder provided for completeness
}

void setMotorPWM(int pwm) {
  analogWrite(MOTOR_ENA, constrain(pwm, 0, 255));
}

void setMotorDirection(int dir) {
  if (dir > 0) {
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (dir < 0) {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
  }
}

bool isHomeSwitchActive() {
  return digitalRead(LIMIT_LEFT) == HIGH;
}

void laserOn() { digitalWrite(LASER_PIN, HIGH); }
void laserOff() { digitalWrite(LASER_PIN, LOW); }

float motorCountsToDegrees(long counts) { return (float)counts / COUNTS_PER_DEGREE; }
long motorDegreesToCounts(float deg) { return (long)(deg * COUNTS_PER_DEGREE); }

void updateMotorVelocity(unsigned long nowMicros) {
  static long lastPos = 0;
  static unsigned long lastTime = 0;

  if (nowMicros - lastTime < MIN_VEL_COMP_TIME) return;

  long pos = encoder.read();
  motorPositionCounts = pos;
  long deltaCounts = pos - lastPos;
  unsigned long deltaMicros = nowMicros - lastTime;

  if (abs(deltaCounts) >= MIN_VEL_COMP_COUNT && deltaMicros > 0) {
    motorVelocity = (float)deltaCounts * 1e6 / (float)deltaMicros;
    motorVelocityCPS = motorVelocity;
    lastPos = pos;
    lastTime = nowMicros;
  }
}

void updateMotorControl(unsigned long nowMicros) {
  (void)nowMicros;
  runPIDController();
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

  // Show configured lane target positions on every auto-mode status log
  Serial.print(F(" |Lanes:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F(" L"));
    Serial.print(i + 1);
    Serial.print(F("="));
    Serial.print(targetPositions[i]);
    Serial.print(F(" "));
  }
  
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

//============================================
// HIGH-LEVEL STATE MACHINE HELPERS
//============================================
void runStateMachine(unsigned long nowMillis) {
  switch (systemState) {
    case ST_BOOT:
      handleBootState();
      break;
    case ST_HOME:
      handleHomeState();
      break;
    case ST_IDLE:
      handleIdleState();
      break;
    case ST_SELECT_LANE:
      handleSelectLaneState();
      break;
    case ST_AIM:
      handleAimState();
      break;
    case ST_FIRE_DWELL:
      handleFireDwellState(nowMillis);
      break;
    case ST_PAUSE:
      handlePauseState();
      break;
    case ST_ERROR:
      handleErrorState();
      break;
  }
}

void handleBootState() {
  initHardware();
  homingStarted = false;
  systemState = ST_HOME;
}

void handleHomeState() {
  if (!homingStarted) {
    homingStarted = true;
    homeToLeftLimit();
    motorPositionCounts = encoder.read();
    systemState = ST_IDLE;
  }
}

void handleIdleState() {
  if (gameRunning && currentMode != MODE_MANUAL) {
    systemState = ST_SELECT_LANE;
    return;
  }

  if (currentMode == MODE_MANUAL && requestedManualLane >= 0) {
    currentLane = requestedManualLane;
    desiredPosition = targetPositions[currentLane];
    targetAngleDeg = motorCountsToDegrees(desiredPosition);
    systemState = ST_AIM;
    requestedManualLane = -1;
  }
}

void handleSelectLaneState() {
  if (!gameRunning) {
    systemState = ST_IDLE;
    return;
  }

  int bestLane = pickMostDangerousLane();
  if (bestLane < 0) {
    systemState = ST_IDLE;
    return;
  }

  currentLane = bestLane;
  targetAngleDeg = motorCountsToDegrees(targetPositions[bestLane]);
  commitToTarget(bestLane);
  state = MOVE_TO_TARGET;
  systemState = ST_AIM;
}

//============================================
// Threat model helpers for per-rail TTI selection
//============================================
void updateThreatModel(unsigned long nowMillis) {
  if (nowMillis - lastTTIUpdate >= TTI_UPDATE_INTERVAL) {
    updateTTI();
    lastTTIUpdate = nowMillis;
  }
  for (int i = 0; i < 4; i++) {
    laneTTI[i] = getEffectiveTTI(i);
  }
}

int pickMostDangerousLane() {
  int bestIdx = -1;
  float bestTTI = 999999.0f;
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction != FORWARD) continue;
    float tti = laneTTI[i];
    if (tti > 0 && tti < bestTTI) {
      bestTTI = tti;
      bestIdx = i;
    }
  }
  return bestIdx;
}

//============================================
// Command handling
//============================================
void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'G':
        currentMode = MODE_STABLE;
        autoMode = true;
        gameRunning = true;
        systemState = ST_SELECT_LANE;
        break;
      case 'T':
        currentMode = MODE_TURBO;
        autoMode = true;
        gameRunning = true;
        systemState = ST_SELECT_LANE;
        break;
      case 'Y':
        currentMode = MODE_HARDCORE;
        autoMode = true;
        gameRunning = true;
        systemState = ST_SELECT_LANE;
        break;
      case 'S':
        gameRunning = false;
        autoMode = false;
        systemState = ST_IDLE;
        break;
      case 'H':
        gameRunning = false;
        homingStarted = false;
        systemState = ST_HOME;
        break;
      case 'M':
        gameRunning = false;
        autoMode = false;
        if (currentMode == MODE_STABLE) currentMode = MODE_TURBO;
        else if (currentMode == MODE_TURBO) currentMode = MODE_HARDCORE;
        else currentMode = MODE_STABLE;
        systemState = ST_IDLE;
        break;
      case '1': case '2': case '3': case '4':
        gameRunning = false;
        autoMode = false;
        currentMode = MODE_MANUAL;
        requestedManualLane = (int)(c - '1');
        systemState = ST_IDLE;
        break;
      default:
        break;
    }
  }
}

//============================================
// AIM and FIRE dwell helpers
//============================================
bool atTargetAngle() {
  float currentDeg = motorCountsToDegrees(encoder.read());
  return fabs(currentDeg - targetAngleDeg) <= 1.0f;
}

void handleAimState() {
  bool manualActive = (currentMode == MODE_MANUAL);
  if (!gameRunning && !autoMode && !manualActive) {
    systemState = ST_IDLE;
    return;
  }

  if (atTargetAngle()) {
    laserOn();
    dwellStartMillis = millis();
    systemState = ST_FIRE_DWELL;
  }
}

void handleFireDwellState(unsigned long nowMillis) {
  unsigned long elapsed = nowMillis - dwellStartMillis;
  if (elapsed >= SENSOR_DWELL_TIME_MS) {
    laserOff();
    if (!gameRunning) {
      releaseCommitment();
      systemState = ST_IDLE;
    } else {
      releaseCommitment();
      systemState = ST_SELECT_LANE;
    }
  }
}

void handlePauseState() {
  stopMotor();
  laserOff();
  if (pauseClearedByCommand) {
    systemState = ST_IDLE;
    pauseClearedByCommand = false;
  }
}

void handleErrorState() {
  stopMotor();
  laserOff();
}
