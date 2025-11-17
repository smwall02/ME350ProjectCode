/*
 * ME350 Competition State Machine Code
 * Plants vs Zombies - Target Interception System
 *
 * Features:
 * - Three-state machine (CALIBRATE, CHOOSE_ACTIVE_TARGET, MOVE_TO_TARGET)
 * - Three-round competition support with different rules per round
 * - Adaptive target selection (prioritizes closest forward-moving zombie)
 * - Low-pass filtered sensor readings with direction detection
 * - PID position control with friction compensation
 * - Comprehensive scoring and status display
 *
 * Hardware: Standard ME350 configuration
 * - Encoder: Pins 2, 3
 * - Motor: Pins 11 (PWM), 12, 13 (direction)
 * - Limit Switches: Pins 8 (left), 9 (right)
 * - Proximity Sensors: A0, A1, A2, A3
 *
 * Competition Rules:
 * Round 1: 40s, normal speed, LED or limit counts
 * Round 2: 40s, faster, LED or limit counts
 * Round 3: Until fail, LED only, increasing speed
 */

#include <Encoder.h>
#include <EEPROM.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// Encoder
#define ENCODER_A 2
#define ENCODER_B 3

// Motor Control (H-Bridge)
#define MOTOR_ENA 11  // PWM
#define MOTOR_IN2 12  // Direction 1
#define MOTOR_IN3 13  // Direction 2

// Flip (enable) switch
#define ON_OFF_SWITCH_PIN 5  // HIGH = enabled, LOW = override/stop

// Limit Switches (active HIGH with INPUT_PULLUP based on hardware wiring)
#define LIMIT_LEFT 8   // Zero position
#define LIMIT_RIGHT 9  // Maximum range

// Proximity Sensors
#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================

enum State {
  IDLE = 0,
  CALIBRATE = 1,
  CHOOSE_ACTIVE_TARGET = 2,
  MOVE_TO_TARGET = 3
};

State currentState = IDLE;

// ============================================================================
// ROUND MANAGEMENT
// ============================================================================

enum Round {
  ROUND_1 = 1,
  ROUND_2 = 2,
  ROUND_3 = 3,
  COMPLETE = 4
};

Round currentRound = ROUND_1;
unsigned long roundStartTime = 0;

const unsigned long ROUND_1_DURATION = 40000;  // 40 seconds
const unsigned long ROUND_2_DURATION = 40000;  // 40 seconds
// Round 3 has no time limit - runs until zombie hits front limit

int round1Score = 0;
int round2Score = 0;
int round3Score = 0;
int totalScore = 0;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Encoder motorEncoder(ENCODER_A, ENCODER_B);

// ============================================================================
// TARGET POSITIONS
// ============================================================================

// Encoder positions for each target (from calibration)
// NOTE: Adjust these values after running calibration!
long TARGET_1_POSITION = -74;
long TARGET_2_POSITION = -307;
long TARGET_3_POSITION = -547;
long TARGET_4_POSITION = -1080;

long targetPositions[4] = {
  TARGET_1_POSITION,
  TARGET_2_POSITION,
  TARGET_3_POSITION,
  TARGET_4_POSITION
};

long WAIT_POSITION = TARGET_3_POSITION;  // Default waiting position

long currentTargetPosition = WAIT_POSITION;
int activeTarget = -1;  // Index of current target (-1 = none)

// ============================================================================
// PID CONTROLLER PARAMETERS
// ============================================================================

// NOTE: Replace these with values from PID auto-tune!
float KP = 0.020;  // Proportional gain
float KI = 0.005;  // Integral gain
float KD = 0.004;  // Derivative gain

// PID state variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

const long DEADBAND = 5;  // Encoder counts - don't correct for small errors
const unsigned long CONTROL_PERIOD = 10;  // ms (100 Hz update rate)
float controlDtSeconds = CONTROL_PERIOD / 1000.0;  // Actual loop dt for PID

// ============================================================================
// FRICTION COMPENSATION
// ============================================================================

// NOTE: Replace these with values from friction characterization!
float FRICTION_LEFT = 2.2;   // Voltage to overcome friction moving RIGHT (stored as positive, applied as negative)
float FRICTION_RIGHT = 0.25; // Voltage to overcome friction moving LEFT (stored and applied as positive)

// Adaptive friction boost (increases if target not reached)
float adaptiveFrictionLeft = FRICTION_LEFT;
float adaptiveFrictionRight = FRICTION_RIGHT;
const float FRICTION_BOOST_AMOUNT = 0.2;

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// Direction detection constants
const int FORWARD = 1;    // Zombie moving toward sensor (approaching)
const int BACKWARD = -1;  // Zombie moving away from sensor
const int STOPPED = 0;    // Zombie not moving

// Low-pass filter coefficient for sensor smoothing
const float ALPHA = 0.925;  // Higher = more filtering (0-1)

// Sensor activation threshold
const int ACTIVATION_THRESHOLD = 400;  // Raw sensor value (0-1023)
const int ACTIVATION_THRESHOLD_HIGH = 430;  // Hysteresis high
const int ACTIVATION_THRESHOLD_LOW = 370;   // Hysteresis low

// Sensor data structure
struct SensorData {
  int rawValue;           // Current raw analog reading
  float filteredValue;    // Low-pass filtered value
  int direction;          // FORWARD, BACKWARD, or STOPPED
  float lastFilteredValue; // Previous filtered value for derivative
  unsigned long lastUpdate; // Timestamp of last update
  float velocity;         // Change per update (filteredValue/sec)
  bool active;            // Above hysteresis threshold
  bool justActivated;     // Rising edge detection
};

SensorData sensors[4];
bool competitionEnabled = false;  // Must be started via command

// EEPROM layout shared with PID auto-tune sketch
const int EEPROM_FLAG = 0;
const int EEPROM_KP = 1;
const int EEPROM_KI = 5;
const int EEPROM_KD = 9;
const int EEPROM_FRICTION_LEFT = 13;
const int EEPROM_FRICTION_RIGHT = 17;
const int EEPROM_LEFT_LIMIT = 21;
const int EEPROM_RIGHT_LIMIT = 25;
const int EEPROM_LANES_BASE = 29;  // 4 lanes * 4 bytes each

// ============================================================================
// CALIBRATION STATE VARIABLES
// ============================================================================

long lastCalibrationPos = 0;
unsigned long calibrationStartTime = 0;
bool isCalibrated = false;
const unsigned long CALIBRATION_TIMEOUT = 15000;  // 15s safety timeout

// ============================================================================
// MOVE TO TARGET STATE VARIABLES
// ============================================================================

unsigned long positionReachedTime = 0;
const unsigned long WAIT_TIME = 1000;  // Wait 1s at position before giving up

unsigned long lastTargetSwitchTime = 0;
const unsigned long TARGET_SWITCH_COOLDOWN = 200;  // 200ms min between switches

// ============================================================================
// TIMING
// ============================================================================

unsigned long lastControlUpdate = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastStatusPrint = 0;

const unsigned long STATUS_PRINT_INTERVAL = 2000;  // Print status every 2s

// Homing softness
const float CALIBRATE_EXTRA_VOLTAGE = 0.6;          // added to overcome friction
const float CALIBRATE_MIN_VOLTAGE = 2.5;            // minimum drive during homing
const unsigned long CALIBRATE_HOLD_TIME = 300;      // ms hold on limit before zeroing
const int CALIBRATE_STABLE_TICKS = 3;               // stable readings before zeroing

// Nudge offsets if no hit at a lane
const int NUDGE_OFFSETS[] = {5, -5, 10, -10};
const int NUDGE_COUNT = sizeof(NUDGE_OFFSETS) / sizeof(NUDGE_OFFSETS[0]);
int nudgeIndex = 0;

// Velocity stop detection
const float VEL_STOP_THRESH = 2.0;  // counts/sec considered stopped

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // Configure motor pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

  // Flip switch
  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);

  // Configure limit switches
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);

  // Initialize sensors
  for (int i = 0; i < 4; i++) {
    sensors[i].rawValue = 0;
    sensors[i].filteredValue = 0;
    sensors[i].lastFilteredValue = 0;
    sensors[i].direction = STOPPED;
    sensors[i].lastUpdate = 0;
    sensors[i].active = false;
    sensors[i].justActivated = false;
  }

  // Stop motor initially
  setMotorVoltage(0);

  // Load PID, friction, limits, and lane positions from EEPROM (if present)
  loadCalibrationFromEEPROM();

  // Initialize timing
  lastControlUpdate = millis();
  lastSensorUpdate = millis();
  roundStartTime = millis();

  Serial.println(F("========================================"));
  Serial.println(F("  ME350 COMPETITION STATE MACHINE"));
  Serial.println(F("  Plants vs Zombies"));
  Serial.println(F("========================================"));
  Serial.println(F("Starting in IDLE. Use serial commands to calibrate/start."));
  Serial.println(F("========================================\n"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();

  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    handleCommand(c);
  }

  // Only update sensors/state machine when enabled or calibrating
  if (currentState == IDLE && !competitionEnabled) {
    // keep idle
    return;
  }

  // Update sensors at 100 Hz
  if (currentTime - lastSensorUpdate >= CONTROL_PERIOD) {
    updateSensors();
    lastSensorUpdate = currentTime;
  }

  // Run state machine at 100 Hz
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    controlDtSeconds = (currentTime - lastControlUpdate) / 1000.0;  // Use actual dt for accurate PID derivative
    runStateMachine();
    lastControlUpdate = currentTime;
  }

  // Check round transitions
  checkRoundTransition();

  // Print status periodically
  if (currentTime - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    printStatus();
    lastStatusPrint = currentTime;
  }
}

// ============================================================================
// STATE MACHINE
// ============================================================================

void runStateMachine() {
  if (currentState == IDLE && !competitionEnabled) {
    return;
  }

  switch (currentState) {
    case CALIBRATE:
      stateCalibrate();
      break;

    case CHOOSE_ACTIVE_TARGET:
      stateChooseActiveTarget();
      break;

    case MOVE_TO_TARGET:
      stateMoveToTarget();
      break;
  }
}

// ============================================================================
// STATE: CALIBRATE
// ============================================================================

void stateCalibrate() {
  /*
   * Purpose: Find left limit switch and zero encoder position
   *
   * Actions:
   * 1. Apply constant positive voltage to move LEFT
   * 2. Wait for left limit switch activation
   * 3. Check for zero velocity (motor has stopped)
   * 4. Zero encoder position
   * 5. Transition to CHOOSE_ACTIVE_TARGET
   */

  if (digitalRead(LIMIT_LEFT) == HIGH) {  // Limit switch pressed (active HIGH)
    // Check if motor has stopped (velocity near zero)
    long currentPos = motorEncoder.read();
    long movement = abs(currentPos - lastCalibrationPos);

    if (movement < 2) {
      // Motor has stopped at limit; hold gently to remove bounce
      unsigned long holdStart = millis();
      long lastPos = currentPos;
      int stableTicks = 0;
      float holdVoltage = max(FRICTION_RIGHT, CALIBRATE_MIN_VOLTAGE - 0.5);
      while (millis() - holdStart < CALIBRATE_HOLD_TIME || stableTicks < CALIBRATE_STABLE_TICKS) {
        setMotorVoltage(holdVoltage);
        delay(10);
        long pos = motorEncoder.read();
        if (abs(pos - lastPos) <= 1) {
          stableTicks++;
        } else {
          stableTicks = 0;
          lastPos = pos;
        }
      }

      motorEncoder.write(0);  // Zero the encoder
      setMotorVoltage(0);     // Stop motor

      isCalibrated = true;
      calibrationStartTime = 0;

      Serial.println(F("=== CALIBRATION COMPLETE ==="));
      Serial.println(F("Encoder zeroed at left limit."));
      Serial.println(F("Transitioning to CHOOSE_ACTIVE_TARGET state.\n"));

      // Move to wait position
      currentTargetPosition = WAIT_POSITION;
      activeTarget = -1;

      // If competition not enabled, return to IDLE; otherwise continue
      currentState = competitionEnabled ? CHOOSE_ACTIVE_TARGET : IDLE;
    }

    lastCalibrationPos = currentPos;
  }
  else {
    // Continue moving to left limit with softer voltage
    float driveVoltage = max(FRICTION_RIGHT + CALIBRATE_EXTRA_VOLTAGE, CALIBRATE_MIN_VOLTAGE);
    setMotorVoltage(driveVoltage);  // Constant voltage LEFT

    // Start calibration timer on first move
    if (calibrationStartTime == 0) {
      calibrationStartTime = millis();
    }

    // Safety: stop if limit switch never triggers
    if (calibrationStartTime > 0 && millis() - calibrationStartTime > CALIBRATION_TIMEOUT) {
      setMotorVoltage(0);
      Serial.println(F("Calibration timeout! Check left limit switch wiring."));
      currentState = CALIBRATE;  // Stay in CALIBRATE but motor stopped
      calibrationStartTime = millis();  // Reset timer so we retry without hammering the stop
    }
  }
}

// ============================================================================
// STATE: CHOOSE_ACTIVE_TARGET
// ============================================================================

void stateChooseActiveTarget() {
  /*
   * Purpose: Determine which zombie to target based on proximity and direction
   *
   * Logic:
   * 1. Read all 4 proximity sensors
   * 2. Determine direction for each zombie (FORWARD/BACKWARD/STOPPED)
   * 3. Calculate distance from current position to each target
   * 4. Priority: Select closest FORWARD-moving zombie
   * 5. If no forward-moving zombies, go to wait position
   * 6. Set target position and transition to MOVE_TO_TARGET
   */

  int chosenTarget = -1;
  float bestVelocity = -1;
  float bestProx = 1e9;

  // Prefer highest approaching velocity; if tie/none, prefer farthest (lowest prox)
  for (int i = 0; i < 4; i++) {
    // Check if sensor detects a zombie
    if (sensors[i].active) {
      // Check if zombie is moving FORWARD (toward the sensor)
      if (sensors[i].direction == FORWARD) {
        float v = sensors[i].velocity;
        if (v > bestVelocity + 0.01) {
          bestVelocity = v;
          bestProx = sensors[i].filteredValue;
          chosenTarget = i;
        } else if (fabs(v - bestVelocity) <= 0.01 && sensors[i].filteredValue < bestProx) {
          // tie on velocity -> pick farthest
          bestProx = sensors[i].filteredValue;
          chosenTarget = i;
        }
      }
    }
  }

  if (chosenTarget >= 0) {
    // Found a target
    currentTargetPosition = targetPositions[chosenTarget];
    activeTarget = chosenTarget;

    Serial.print(F("Target selected (farthest forward): "));
    Serial.print(chosenTarget + 1);
    Serial.print(F(" at position "));
    Serial.println(currentTargetPosition);
  }
  else {
    // No forward-moving zombies, go to wait position
    currentTargetPosition = WAIT_POSITION;
    activeTarget = -1;

    // Don't spam serial, only print once
    static bool waitMessagePrinted = false;
    if (!waitMessagePrinted) {
      Serial.println(F("No active targets. Moving to wait position."));
      waitMessagePrinted = true;
    }
  }

  // Reset PID state for new target
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
  nudgeIndex = 0;

  // Transition to MOVE_TO_TARGET
  currentState = MOVE_TO_TARGET;
  positionReachedTime = millis();
}

// ============================================================================
// STATE: MOVE_TO_TARGET
// ============================================================================

void stateMoveToTarget() {
  /*
   * Purpose: Execute PID control to reach target position
   *
   * Actions:
   * 1. Run PID control loop
   * 2. Monitor proximity sensor for zombie activation
   * 3. If zombie detected (LED lights):
   *    - Increment score
   *    - Return to CHOOSE_ACTIVE_TARGET
   * 4. Allow dynamic target switching if closer forward-moving zombie appears
   * 5. If position reached and held for WAIT_TIME with no activation:
   *    - Return to CHOOSE_ACTIVE_TARGET
   */

  // Run PID controller
  float voltage = updatePID(currentTargetPosition);
  setMotorVoltage(voltage);

  // Check if zombie activated LED (hit detection)
  if (activeTarget >= 0) {
    // Consider a hit when we see a rising edge OR a backward direction (zombie leaving after being close)
    bool directionFlip = (sensors[activeTarget].direction == BACKWARD && sensors[activeTarget].rawValue > ACTIVATION_THRESHOLD_LOW);
    if (sensors[activeTarget].justActivated || directionFlip) {
      recordHit();

      Serial.print(F("*** HIT! Target "));
      Serial.print(activeTarget + 1);
      Serial.println(F(" ***"));

      // Return to target selection
      currentState = CHOOSE_ACTIVE_TARGET;
      return;
    }
  }

  // Check for closer forward-moving zombie (dynamic switching)
  if (millis() - lastTargetSwitchTime > TARGET_SWITCH_COOLDOWN) {
    checkForBetterTarget();
  }

  // Check if position is stable
  if (abs(error) < DEADBAND) {
    // Position reached, check if we've waited long enough
    if (millis() - positionReachedTime > WAIT_TIME) {
      // Try nudges before giving up
      if (activeTarget >= 0 && nudgeIndex < NUDGE_COUNT) {
        currentTargetPosition = targetPositions[activeTarget] + NUDGE_OFFSETS[nudgeIndex];
        nudgeIndex++;
        Serial.print(F("No activation, nudging to "));
        Serial.println(currentTargetPosition);
        positionReachedTime = millis();
        return;
      }

      // Position held but no activation - zombie missed or passed
      Serial.println(F("Position reached, no activation. Choosing new target."));

      // Boost friction to overcome stiction on next move
      adaptiveFrictionLeft += FRICTION_BOOST_AMOUNT;
      adaptiveFrictionRight += FRICTION_BOOST_AMOUNT;

      currentState = CHOOSE_ACTIVE_TARGET;
    }
  }
  else {
    // Still moving, reset timer
    positionReachedTime = millis();

    // Reset adaptive friction when actively moving
    adaptiveFrictionLeft = FRICTION_LEFT;
    adaptiveFrictionRight = FRICTION_RIGHT;
  }
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

float updatePID(long targetPosition) {
  long currentPosition = motorEncoder.read();

  // Calculate error
  error = targetPosition - currentPosition;

  // Apply deadband
  if (abs(error) < DEADBAND) {
    error = 0;
    integral = 0;  // Reset integral when at target
  }
  else {
    // Calculate integral (with anti-windup)
    integral += error * controlDtSeconds;
    integral = constrain(integral, -1000, 1000);
  }

  // Calculate derivative
  derivative = (error - lastError) / controlDtSeconds;

  // Calculate PID output
  float pidOutput = KP * error + KI * integral + KD * derivative;

  // Add friction compensation
  float frictionComp = 0;

  if (error < -DEADBAND) {
    // Need to move RIGHT (negative direction)
    frictionComp = -adaptiveFrictionLeft;
  }
  else if (error > DEADBAND) {
    // Need to move LEFT (positive direction)
    frictionComp = adaptiveFrictionRight;
  }

  // Calculate total voltage
  float voltage = pidOutput + frictionComp;

  // Store for next iteration
  lastError = error;

  return voltage;
}

// ============================================================================
// SENSOR UPDATES
// ============================================================================

void updateSensors() {
  for (int i = 0; i < 4; i++) {
    bool previousActive = sensors[i].active;

    // Read raw analog value
    int raw = analogRead(A0 + i);

    // Apply low-pass filter
    // filtered = alpha * filtered_old + (1 - alpha) * raw
    sensors[i].filteredValue = ALPHA * sensors[i].filteredValue +
                               (1.0 - ALPHA) * raw;

    // Calculate derivative (rate of change)
    float derivative = sensors[i].filteredValue - sensors[i].lastFilteredValue;
    float dt = (millis() - sensors[i].lastUpdate) / 1000.0;
    if (dt <= 0) dt = CONTROL_PERIOD / 1000.0;
    sensors[i].velocity = derivative / dt;

    // Determine direction based on derivative
    if (derivative > 2.0) {
      sensors[i].direction = FORWARD;   // Value increasing = zombie approaching
    }
    else if (derivative < -2.0) {
      sensors[i].direction = BACKWARD;  // Value decreasing = zombie leaving
    }
    else {
      sensors[i].direction = STOPPED;   // Stable = zombie stopped or not present
    }

    // Store values for next iteration
    sensors[i].rawValue = raw;
    sensors[i].lastFilteredValue = sensors[i].filteredValue;
    sensors[i].lastUpdate = millis();

    // Hysteresis-based activation using filtered value
    if (sensors[i].filteredValue >= ACTIVATION_THRESHOLD_HIGH) {
      sensors[i].active = true;
    }
    else if (sensors[i].filteredValue <= ACTIVATION_THRESHOLD_LOW) {
      sensors[i].active = false;
    }

    sensors[i].justActivated = sensors[i].active && !previousActive;
  }
}

// ============================================================================
// TARGET SWITCHING
// ============================================================================

void checkForBetterTarget() {
  /*
   * Check if there's a closer forward-moving zombie than current target
   * This allows dynamic target switching during movement
   */

  if (activeTarget < 0) {
    return;  // No active target, nothing to switch from
  }

  long currentPos = motorEncoder.read();
  long currentTargetDistance = abs(currentPos - currentTargetPosition);

  for (int i = 0; i < 4; i++) {
    if (i == activeTarget) {
      continue;  // Skip current target
    }

    // Check if this sensor detects a forward-moving zombie
    if (sensors[i].active &&
        sensors[i].direction == FORWARD) {

      long newTargetDistance = abs(currentPos - targetPositions[i]);

      // Switch if new target is significantly closer (>20% closer)
      if (newTargetDistance < currentTargetDistance * 0.8) {
        Serial.print(F("Switching target: "));
        Serial.print(activeTarget + 1);
        Serial.print(F(" -> "));
        Serial.println(i + 1);

        // Switch to new target
        currentTargetPosition = targetPositions[i];
        activeTarget = i;

        // Reset PID state
        integral = 0;

        lastTargetSwitchTime = millis();

        break;  // Only switch once per check
      }
    }
  }
}

// ============================================================================
// ROUND MANAGEMENT
// ============================================================================

void checkRoundTransition() {
  unsigned long elapsed = millis() - roundStartTime;
  static unsigned long allStoppedSince = 0;

  auto allSensorsStopped = [&]() {
    for (int i = 0; i < 4; i++) {
      if (fabs(sensors[i].velocity) > VEL_STOP_THRESH) return false;
    }
    return true;
  };

  bool sensorsStopped = allSensorsStopped();
  if (sensorsStopped) {
    if (allStoppedSince == 0) allStoppedSince = millis();
  } else {
    allStoppedSince = 0;
  }
  bool stoppedFor5s = (allStoppedSince > 0) && (millis() - allStoppedSince >= 5000);

  switch (currentRound) {
    case ROUND_1:
      // Transition if: (time up AND sensors quiet for 5s) OR (time up + 10s grace period)
      if ((elapsed >= ROUND_1_DURATION && stoppedFor5s) || (elapsed >= ROUND_1_DURATION + 10000)) {
        // Round 1 complete
        if (elapsed >= ROUND_1_DURATION + 10000) {
          Serial.println(F("Round 1 timeout - forcing transition"));
        }
        Serial.println(F("\n========================================"));
        Serial.println(F("=== ROUND 1 COMPLETE ==="));
        Serial.print(F("Score: "));
        Serial.println(round1Score);
        Serial.println(F("========================================\n"));

        currentRound = ROUND_2;
        roundStartTime = millis();
        allStoppedSince = 0;  // Reset for next round
      }
      break;

    case ROUND_2:
      // Transition if: (time up AND sensors quiet for 5s) OR (time up + 10s grace period)
      if ((elapsed >= ROUND_2_DURATION && stoppedFor5s) || (elapsed >= ROUND_2_DURATION + 10000)) {
        // Round 2 complete
        if (elapsed >= ROUND_2_DURATION + 10000) {
          Serial.println(F("Round 2 timeout - forcing transition"));
        }
        Serial.println(F("\n========================================"));
        Serial.println(F("=== ROUND 2 COMPLETE ==="));
        Serial.print(F("Score: "));
        Serial.println(round2Score);
        Serial.println(F("========================================\n"));

        currentRound = ROUND_3;
        roundStartTime = millis();
        allStoppedSince = 0;  // Reset for next round

        Serial.println(F("=== ROUND 3 STARTING ==="));
        Serial.println(F("LED ONLY - Limit switch ends round!"));
        Serial.println(F("========================================\n"));
      }
      break;

    case ROUND_3:
      // Round 3 ends when zombie hits front limit switch
      if (digitalRead(LIMIT_RIGHT) == HIGH) {
        // Zombie hit front limit - GAME OVER
        Serial.println(F("\n========================================"));
        Serial.println(F("=== ROUND 3 FAILED ==="));
        Serial.println(F("Zombie reached front limit!"));
        Serial.print(F("Score: "));
        Serial.println(round3Score);
        Serial.println(F("========================================\n"));

        currentRound = COMPLETE;
        setMotorVoltage(0);  // Stop motor

        printFinalScore();
      }
      break;

    case COMPLETE:
      // Game over, do nothing
      setMotorVoltage(0);
      break;
  }
}

void recordHit() {
  /*
   * Record a zombie hit based on current round rules
   */

  switch (currentRound) {
    case ROUND_1:
      round1Score++;
      totalScore++;
      break;

    case ROUND_2:
      round2Score++;
      totalScore++;
      break;

    case ROUND_3:
      // In Round 3, ONLY LED activation counts
      // (limit switch would end the round, not add to score)
      round3Score++;
      totalScore++;
      break;

    case COMPLETE:
      // Game over, no scoring
      break;
  }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

bool isSwitchEnabled() {
  return digitalRead(ON_OFF_SWITCH_PIN) == HIGH;
}

void setMotorVoltage(float voltage) {
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
  int pwmValue = abs(voltage) * 25.5;  // 10V -> 255

  // Safety: Check limit switches and prevent movement into limits
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwmValue = 0;
  }
  if (digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
    // In Round 3, hitting right limit ends the game
    if (currentRound == ROUND_3) {
      // Let checkRoundTransition handle this
    }
    voltage = 0;
    pwmValue = 0;
  }

  // Apply voltage to H-bridge
  if (voltage > 0) {
    // Move LEFT (toward position 0)
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, pwmValue);
  }
  else if (voltage < 0) {
    // Move RIGHT (toward negative positions)
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
    analogWrite(MOTOR_ENA, pwmValue);
  }
  else {
    // STOP
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, 0);
  }
}

// ============================================================================
// STATUS DISPLAY
// ============================================================================

void printStatus() {
  Serial.print(F("Round "));
  Serial.print(currentRound);
  Serial.print(F(" | State: "));

  switch (currentState) {
    case IDLE:
      Serial.print(F("IDLE"));
      break;
    case CALIBRATE:
      Serial.print(F("CALIBRATE"));
      break;
    case CHOOSE_ACTIVE_TARGET:
      Serial.print(F("CHOOSE_TARGET"));
      break;
    case MOVE_TO_TARGET:
      Serial.print(F("MOVE_TO_TARGET"));
      break;
  }

  Serial.print(F(" | Pos: "));
  Serial.print(motorEncoder.read());

  if (activeTarget >= 0) {
    Serial.print(F(" | Target: "));
    Serial.print(activeTarget + 1);
    Serial.print(F(" ("));
    Serial.print(currentTargetPosition);
    Serial.print(F(")"));
  }

  Serial.print(F(" | Score: "));
  Serial.print(totalScore);

  Serial.print(F(" | Sensors: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(sensors[i].rawValue);
    if (i < 3) Serial.print(F(","));
  }

  Serial.println();
}

void printFinalScore() {
  Serial.println(F("\n========================================"));
  Serial.println(F("=== FINAL SCORE ==="));
  Serial.println(F("========================================"));
  Serial.print(F("Round 1: "));
  Serial.println(round1Score);
  Serial.print(F("Round 2: "));
  Serial.println(round2Score);
  Serial.print(F("Round 3: "));
  Serial.println(round3Score);
  Serial.println(F("----------------------------------------"));
  Serial.print(F("TOTAL:   "));
  Serial.println(totalScore);
  Serial.println(F("========================================\n"));
}

// ============================================================================
// EEPROM LOADING
// ============================================================================

void loadCalibrationFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) {
    Serial.println(F("EEPROM flag not set; using defaults."));
    return;
  }

  EEPROM.get(EEPROM_KP, KP);
  EEPROM.get(EEPROM_KI, KI);
  EEPROM.get(EEPROM_KD, KD);
  EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.get(EEPROM_LEFT_LIMIT, TARGET_1_POSITION);   // not directly used, but keep compatible
  EEPROM.get(EEPROM_RIGHT_LIMIT, TARGET_4_POSITION);  // not directly used, but keep compatible

  for (int i = 0; i < 4; i++) {
    long v;
    EEPROM.get(EEPROM_LANES_BASE + i * sizeof(long), v);
    targetPositions[i] = v;
  }
  WAIT_POSITION = targetPositions[2];

  Serial.println(F("Loaded PID/friction/lanes from EEPROM:"));
  Serial.print(F("  Kp=")); Serial.print(KP, 6);
  Serial.print(F(" Ki=")); Serial.print(KI, 6);
  Serial.print(F(" Kd=")); Serial.println(KD, 6);
  Serial.print(F("  Fric L=")); Serial.print(FRICTION_LEFT, 3);
  Serial.print(F(" R=")); Serial.println(FRICTION_RIGHT, 3);
  Serial.print(F("  Lanes: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(targetPositions[i]);
    if (i < 3) Serial.print(F(", "));
  }
  Serial.println();
}

// ============================================================================
// COMMAND HANDLING & EEPROM helpers
// ============================================================================

void loadTargets() {
  byte flag = EEPROM.read(EEPROM_FLAG);
  if (flag != 0xAA) return;
  for (int i = 0; i < 4; i++) {
    long val;
    EEPROM.get(EEPROM_LANES_BASE + i * sizeof(long), val);
    targetPositions[i] = val;
  }
  WAIT_POSITION = targetPositions[2];
}

void saveTargets() {
  EEPROM.write(EEPROM_FLAG, 0xAA);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * sizeof(long), targetPositions[i]);
  }
  Serial.println(F("Lane positions saved to EEPROM."));
}

void printConfig() {
  Serial.println(F("\n=== CONFIG ==="));
  Serial.print(F("Lane 1: ")); Serial.println(targetPositions[0]);
  Serial.print(F("Lane 2: ")); Serial.println(targetPositions[1]);
  Serial.print(F("Lane 3: ")); Serial.println(targetPositions[2]);
  Serial.print(F("Lane 4: ")); Serial.println(targetPositions[3]);
  Serial.print(F("Wait pos: ")); Serial.println(WAIT_POSITION);
  Serial.print(F("Competition enabled: "));
  Serial.println(competitionEnabled ? F("YES") : F("NO"));
}

void handleCommand(char c) {
  c = toupper(c);
  switch (c) {
    case 'H':
      Serial.println(F("\nCommands:"));
      Serial.println(F("  C - Calibrate to left limit"));
      Serial.println(F("  1-4 - Set lane position to current encoder reading"));
      Serial.println(F("  P - Print config"));
      Serial.println(F("  S - Stop/IDLE"));
      Serial.println(F("  G - Start competition (Round 1)"));
      Serial.println(F("  L - Load lanes from EEPROM"));
      Serial.println(F("  W - Save lanes to EEPROM"));
      break;

    case 'C':
      currentState = CALIBRATE;
      break;

    case '1': case '2': case '3': case '4': {
      int lane = c - '1';
      long val = motorEncoder.read();
      targetPositions[lane] = val;
      WAIT_POSITION = targetPositions[2];
      Serial.print(F("Lane ")); Serial.print(lane + 1);
      Serial.print(F(" set to ")); Serial.println(val);
      break;
    }

    case 'P':
      printConfig();
      break;

    case 'S':
      competitionEnabled = false;
      currentState = IDLE;
      setMotorVoltage(0);
      Serial.println(F("Stopped. State=IDLE."));
      break;

    case 'G':
      // Start competition
      competitionEnabled = true;
      currentRound = ROUND_1;
      round1Score = round2Score = round3Score = totalScore = 0;
      roundStartTime = millis();
      currentState = isCalibrated ? CHOOSE_ACTIVE_TARGET : CALIBRATE;
      Serial.println(F("Competition started (Round 1)."));
      break;

    case 'L':
      loadTargets();
      Serial.println(F("Loaded lane positions from EEPROM."));
      printConfig();
      break;

    case 'W':
      saveTargets();
      break;

    default:
      break;
  }
}
