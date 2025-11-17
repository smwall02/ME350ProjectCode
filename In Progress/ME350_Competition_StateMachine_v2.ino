/*
 * ME350 Competition State Machine Code - Version 2
 * Plants vs Zombies - Target Interception System
 *
 * IMPORTANT SENSOR LOGIC CLARIFICATION:
 * ======================================
 * Zombies move from BACK → FRONT (toward RIGHT limit switch/goal)
 *
 * Sensor value behavior as zombie passes:
 *   1. Zombie far behind sensor → LOW value (~0-100)
 *   2. Zombie approaching sensor → INCREASING value (200, 300, 400...)
 *   3. Zombie AT sensor position → MAXIMUM value (500-800)
 *   4. Zombie passing sensor (toward goal) → DECREASING value (400, 300, 200...)
 *   5. Zombie far past sensor → LOW value (~0-100)
 *
 * Direction Detection:
 *   - APPROACHING: sensor value INCREASING (derivative > 0) → SHOULD TARGET
 *   - LEAVING: sensor value DECREASING (derivative < 0) → TOO LATE, DON'T TARGET
 *   - STOPPED/STABLE: sensor value constant (derivative ≈ 0)
 *
 * We target zombies that are APPROACHING or AT the sensor, NOT ones that are LEAVING
 *
 * Features:
 * - Setup mode to configure target positions before competition
 * - Three-state machine (CALIBRATE, CHOOSE_ACTIVE_TARGET, MOVE_TO_TARGET)
 * - Three-round competition support
 * - Direction detection with low-pass filtering
 * - Dynamic target switching
 * - PID control with friction compensation
 */

#include <Encoder.h>

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

// Limit Switches (active LOW with INPUT_PULLUP)
#define LIMIT_LEFT 8   // Zero position (back)
#define LIMIT_RIGHT 9  // Maximum range (front/goal)

// Proximity Sensors
#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================

enum State {
  SETUP = 0,                   // NEW: Configure target positions
  CALIBRATE = 1,
  CHOOSE_ACTIVE_TARGET = 2,
  MOVE_TO_TARGET = 3
};

State currentState = SETUP;  // Start in SETUP mode

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

// Default positions (ADJUST THESE or use SETUP mode)
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

bool setupComplete = false;

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

const long DEADBAND = 5;  // Encoder counts
const unsigned long CONTROL_PERIOD = 10;  // ms (100 Hz)

// ============================================================================
// FRICTION COMPENSATION
// ============================================================================

// NOTE: Replace these with values from friction characterization!
float FRICTION_LEFT = 2.2;   // Voltage to overcome friction moving LEFT
float FRICTION_RIGHT = 0.25; // Voltage to overcome friction moving RIGHT

// Adaptive friction boost
float adaptiveFrictionLeft = FRICTION_LEFT;
float adaptiveFrictionRight = FRICTION_RIGHT;
const float FRICTION_BOOST_AMOUNT = 0.2;

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// Direction constants
// CRITICAL: This is from the SENSOR'S perspective, not the zombie's game direction!
const int APPROACHING = 1;   // Sensor value INCREASING (zombie coming toward sensor)
const int LEAVING = -1;      // Sensor value DECREASING (zombie moving past sensor)
const int STOPPED = 0;       // Sensor value STABLE

// Low-pass filter coefficient
const float ALPHA = 0.925;  // Higher = more filtering

// Sensor activation threshold
const int ACTIVATION_THRESHOLD = 400;  // Raw sensor value (0-1023)

// Minimum derivative to detect movement
const float MOVEMENT_THRESHOLD = 2.0;

// Sensor data structure
struct SensorData {
  int rawValue;           // Current raw analog reading
  float filteredValue;    // Low-pass filtered value
  int direction;          // APPROACHING, LEAVING, or STOPPED
  float lastFilteredValue; // Previous filtered value for derivative
  unsigned long lastUpdate; // Timestamp of last update
};

SensorData sensors[4];

// ============================================================================
// CALIBRATION STATE VARIABLES
// ============================================================================

long lastCalibrationPos = 0;
unsigned long calibrationStartTime = 0;
bool isCalibrated = false;

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

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // Configure motor pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

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
  }

  // Stop motor initially
  setMotorVoltage(0);

  // Initialize timing
  lastControlUpdate = millis();
  lastSensorUpdate = millis();

  printWelcome();
  printSetupInstructions();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();

  // Update sensors at 100 Hz
  if (currentTime - lastSensorUpdate >= CONTROL_PERIOD) {
    updateSensors();
    lastSensorUpdate = currentTime;
  }

  // Handle setup mode commands
  if (currentState == SETUP) {
    if (Serial.available() > 0) {
      handleSetupCommand();
    }
    return;  // Don't run state machine until setup complete
  }

  // Run state machine at 100 Hz
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    runStateMachine();
    lastControlUpdate = currentTime;
  }

  // Check round transitions (only after setup)
  if (setupComplete) {
    checkRoundTransition();
  }

  // Print status periodically
  if (currentTime - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    printStatus();
    lastStatusPrint = currentTime;
  }
}

// ============================================================================
// SETUP MODE
// ============================================================================

void handleSetupCommand() {
  char cmd = Serial.read();

  switch (toupper(cmd)) {
    case 'S':
      // Start competition
      if (validateSetup()) {
        currentState = CALIBRATE;
        setupComplete = true;
        roundStartTime = millis();
        Serial.println(F("\n=== COMPETITION STARTING ==="));
        Serial.println(F("Entering CALIBRATE state...\n"));
      }
      break;

    case 'P':
      // Set position for target
      setTargetPosition();
      break;

    case 'W':
      // Set wait position
      setWaitPosition();
      break;

    case 'V':
      // View current configuration
      printConfiguration();
      break;

    case 'G':
      // Set PID gains
      setPIDGains();
      break;

    case 'F':
      // Set friction values
      setFrictionValues();
      break;

    case '?':
      printSetupInstructions();
      break;

    default:
      // Ignore whitespace and newlines
      break;
  }
}

void setTargetPosition() {
  Serial.println(F("\nEnter target number (1-4):"));
  while (!Serial.available()) { }
  int targetNum = Serial.parseInt();

  if (targetNum < 1 || targetNum > 4) {
    Serial.println(F("ERROR: Invalid target number. Must be 1-4."));
    return;
  }

  Serial.println(F("Enter encoder position for this target:"));
  while (!Serial.available()) { }
  long position = Serial.parseInt();

  targetPositions[targetNum - 1] = position;

  Serial.print(F("Target "));
  Serial.print(targetNum);
  Serial.print(F(" set to position: "));
  Serial.println(position);
}

void setWaitPosition() {
  Serial.println(F("\nEnter wait position (encoder value):"));
  while (!Serial.available()) { }
  long position = Serial.parseInt();

  WAIT_POSITION = position;

  Serial.print(F("Wait position set to: "));
  Serial.println(position);
}

void setPIDGains() {
  Serial.println(F("\nEnter Kp value:"));
  while (!Serial.available()) { }
  KP = Serial.parseFloat();

  Serial.println(F("Enter Ki value:"));
  while (!Serial.available()) { }
  KI = Serial.parseFloat();

  Serial.println(F("Enter Kd value:"));
  while (!Serial.available()) { }
  KD = Serial.parseFloat();

  Serial.println(F("PID gains updated:"));
  Serial.print(F("Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("Kd = "));
  Serial.println(KD, 6);
}

void setFrictionValues() {
  Serial.println(F("\nEnter FRICTION_LEFT value (volts):"));
  while (!Serial.available()) { }
  FRICTION_LEFT = Serial.parseFloat();

  Serial.println(F("Enter FRICTION_RIGHT value (volts):"));
  while (!Serial.available()) { }
  FRICTION_RIGHT = Serial.parseFloat();

  adaptiveFrictionLeft = FRICTION_LEFT;
  adaptiveFrictionRight = FRICTION_RIGHT;

  Serial.println(F("Friction values updated:"));
  Serial.print(F("FRICTION_LEFT = "));
  Serial.println(FRICTION_LEFT, 3);
  Serial.print(F("FRICTION_RIGHT = "));
  Serial.println(FRICTION_RIGHT, 3);
}

bool validateSetup() {
  // Basic validation
  if (targetPositions[0] == 0 && targetPositions[1] == 0 &&
      targetPositions[2] == 0 && targetPositions[3] == 0) {
    Serial.println(F("ERROR: All target positions are zero. Please configure positions first."));
    return false;
  }

  Serial.println(F("Setup validation passed."));
  return true;
}

void printSetupInstructions() {
  Serial.println(F("\n========================================"));
  Serial.println(F("        SETUP MODE COMMANDS"));
  Serial.println(F("========================================"));
  Serial.println(F("P - Set target Position (1-4)"));
  Serial.println(F("W - Set Wait position"));
  Serial.println(F("G - Set PID Gains (Kp, Ki, Kd)"));
  Serial.println(F("F - Set Friction values"));
  Serial.println(F("V - View current configuration"));
  Serial.println(F("S - Start competition"));
  Serial.println(F("? - Show this help"));
  Serial.println(F("========================================\n"));
}

void printConfiguration() {
  Serial.println(F("\n=== CURRENT CONFIGURATION ==="));
  Serial.println(F("\nTarget Positions:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Target "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(targetPositions[i]);
  }
  Serial.print(F("  Wait Position: "));
  Serial.println(WAIT_POSITION);

  Serial.println(F("\nPID Gains:"));
  Serial.print(F("  Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("  Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("  Kd = "));
  Serial.println(KD, 6);

  Serial.println(F("\nFriction Compensation:"));
  Serial.print(F("  FRICTION_LEFT = "));
  Serial.print(FRICTION_LEFT, 3);
  Serial.println(F(" V"));
  Serial.print(F("  FRICTION_RIGHT = "));
  Serial.print(FRICTION_RIGHT, 3);
  Serial.println(F(" V"));

  Serial.println(F("========================================\n"));
}

void printWelcome() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  ME350 COMPETITION STATE MACHINE"));
  Serial.println(F("  Plants vs Zombies - Version 2"));
  Serial.println(F("========================================"));
  Serial.println(F("SETUP MODE - Configure before starting"));
  Serial.println(F("========================================\n"));
}

// ============================================================================
// STATE MACHINE
// ============================================================================

void runStateMachine() {
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

    default:
      break;
  }
}

// ============================================================================
// STATE: CALIBRATE
// ============================================================================

void stateCalibrate() {
  /*
   * Purpose: Find left limit switch and zero encoder position
   */

  if (digitalRead(LIMIT_LEFT) == LOW) {  // Limit switch pressed (active LOW)
    // Check if motor has stopped (velocity near zero)
    long currentPos = motorEncoder.read();
    long movement = abs(currentPos - lastCalibrationPos);

    if (movement < 2) {
      // Motor has stopped at limit
      motorEncoder.write(0);  // Zero the encoder
      setMotorVoltage(0);     // Stop motor

      isCalibrated = true;

      Serial.println(F("=== CALIBRATION COMPLETE ==="));
      Serial.println(F("Encoder zeroed at left limit."));
      Serial.println(F("Transitioning to CHOOSE_ACTIVE_TARGET state.\n"));

      // Move to wait position
      currentTargetPosition = WAIT_POSITION;
      activeTarget = -1;

      currentState = CHOOSE_ACTIVE_TARGET;
    }

    lastCalibrationPos = currentPos;
  }
  else {
    // Continue moving to left limit
    setMotorVoltage(5.0);  // Constant voltage LEFT
  }
}

// ============================================================================
// STATE: CHOOSE_ACTIVE_TARGET
// ============================================================================

void stateChooseActiveTarget() {
  /*
   * Purpose: Determine which zombie to target
   *
   * Priority: Select closest APPROACHING zombie
   * - Zombie must be detected (sensor value > threshold)
   * - Zombie must be APPROACHING (sensor value increasing)
   * - Among qualifying zombies, choose closest to current position
   */

  int closestTarget = -1;
  long minDistance = 999999;

  // Find closest APPROACHING zombie
  for (int i = 0; i < 4; i++) {
    // Check if sensor detects a zombie
    if (sensors[i].rawValue > ACTIVATION_THRESHOLD) {
      // Check if zombie is APPROACHING the sensor (value increasing)
      // This means the zombie is coming toward us and we can still intercept it
      if (sensors[i].direction == APPROACHING) {
        // Calculate distance from current position to this target
        long currentPos = motorEncoder.read();
        long distance = abs(currentPos - targetPositions[i]);

        if (distance < minDistance) {
          minDistance = distance;
          closestTarget = i;
        }
      }
    }
  }

  if (closestTarget >= 0) {
    // Found an approaching zombie
    currentTargetPosition = targetPositions[closestTarget];
    activeTarget = closestTarget;

    Serial.print(F("Target selected: "));
    Serial.print(closestTarget + 1);
    Serial.print(F(" at position "));
    Serial.println(currentTargetPosition);
  }
  else {
    // No approaching zombies, go to wait position
    currentTargetPosition = WAIT_POSITION;
    activeTarget = -1;
  }

  // Reset PID state for new target
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;

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
   */

  // Run PID controller
  float voltage = updatePID(currentTargetPosition);
  setMotorVoltage(voltage);

  // Check if zombie activated LED (hit detection)
  if (activeTarget >= 0) {
    if (sensors[activeTarget].rawValue > ACTIVATION_THRESHOLD) {
      // Zombie has been hit!
      recordHit();

      Serial.print(F("*** HIT! Target "));
      Serial.print(activeTarget + 1);
      Serial.println(F(" ***"));

      // Return to target selection
      currentState = CHOOSE_ACTIVE_TARGET;
      return;
    }
  }

  // Check for closer approaching zombie (dynamic switching)
  if (millis() - lastTargetSwitchTime > TARGET_SWITCH_COOLDOWN) {
    checkForBetterTarget();
  }

  // Check if position is stable
  if (abs(error) < DEADBAND) {
    // Position reached, check if we've waited long enough
    if (millis() - positionReachedTime > WAIT_TIME) {
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
    integral += error * (CONTROL_PERIOD / 1000.0);
    integral = constrain(integral, -1000, 1000);
  }

  // Calculate derivative
  derivative = (error - lastError) / (CONTROL_PERIOD / 1000.0);

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
  /*
   * CRITICAL SENSOR LOGIC:
   * ======================
   * When zombie APPROACHES sensor:
   *   - Sensor value INCREASES (50 → 100 → 200 → 400 → 600)
   *   - Derivative is POSITIVE
   *   - Direction = APPROACHING
   *   - We SHOULD target this zombie
   *
   * When zombie LEAVES sensor (moves toward goal):
   *   - Sensor value DECREASES (600 → 400 → 200 → 100 → 50)
   *   - Derivative is NEGATIVE
   *   - Direction = LEAVING
   *   - We should NOT target this zombie (too late!)
   */

  for (int i = 0; i < 4; i++) {
    // Read raw analog value
    int raw = analogRead(A0 + i);

    // Apply low-pass filter to reduce noise
    sensors[i].filteredValue = ALPHA * sensors[i].filteredValue +
                               (1.0 - ALPHA) * raw;

    // Calculate derivative (rate of change)
    float derivative = sensors[i].filteredValue - sensors[i].lastFilteredValue;

    // Determine direction based on derivative
    if (derivative > MOVEMENT_THRESHOLD) {
      // Value INCREASING = zombie APPROACHING sensor
      sensors[i].direction = APPROACHING;
    }
    else if (derivative < -MOVEMENT_THRESHOLD) {
      // Value DECREASING = zombie LEAVING sensor (moving toward goal)
      sensors[i].direction = LEAVING;
    }
    else {
      // Value STABLE = zombie stopped or not present
      sensors[i].direction = STOPPED;
    }

    // Store values for next iteration
    sensors[i].rawValue = raw;
    sensors[i].lastFilteredValue = sensors[i].filteredValue;
    sensors[i].lastUpdate = millis();
  }
}

// ============================================================================
// TARGET SWITCHING
// ============================================================================

void checkForBetterTarget() {
  /*
   * Check if there's a closer approaching zombie than current target
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

    // Check if this sensor detects an approaching zombie
    if (sensors[i].rawValue > ACTIVATION_THRESHOLD &&
        sensors[i].direction == APPROACHING) {

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

        break;
      }
    }
  }
}

// ============================================================================
// ROUND MANAGEMENT
// ============================================================================

void checkRoundTransition() {
  unsigned long elapsed = millis() - roundStartTime;

  switch (currentRound) {
    case ROUND_1:
      if (elapsed >= ROUND_1_DURATION) {
        Serial.println(F("\n========================================"));
        Serial.println(F("=== ROUND 1 COMPLETE ==="));
        Serial.print(F("Score: "));
        Serial.println(round1Score);
        Serial.println(F("========================================\n"));

        currentRound = ROUND_2;
        roundStartTime = millis();
      }
      break;

    case ROUND_2:
      if (elapsed >= ROUND_2_DURATION) {
        Serial.println(F("\n========================================"));
        Serial.println(F("=== ROUND 2 COMPLETE ==="));
        Serial.print(F("Score: "));
        Serial.println(round2Score);
        Serial.println(F("========================================\n"));

        currentRound = ROUND_3;
        roundStartTime = millis();

        Serial.println(F("=== ROUND 3 STARTING ==="));
        Serial.println(F("LED ONLY - Limit switch ends round!"));
        Serial.println(F("========================================\n"));
      }
      break;

    case ROUND_3:
      // Round 3 ends when zombie hits front limit switch
      if (digitalRead(LIMIT_RIGHT) == LOW) {
        Serial.println(F("\n========================================"));
        Serial.println(F("=== ROUND 3 FAILED ==="));
        Serial.println(F("Zombie reached front limit!"));
        Serial.print(F("Score: "));
        Serial.println(round3Score);
        Serial.println(F("========================================\n"));

        currentRound = COMPLETE;
        setMotorVoltage(0);

        printFinalScore();
      }
      break;

    case COMPLETE:
      setMotorVoltage(0);
      break;
  }
}

void recordHit() {
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
      round3Score++;
      totalScore++;
      break;

    case COMPLETE:
      break;
  }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setMotorVoltage(float voltage) {
  // Constrain voltage to safe range
  voltage = constrain(voltage, -10.0, 10.0);

  // Convert voltage to PWM (0-255)
  int pwmValue = abs(voltage) * 25.5;  // 10V -> 255

  // Safety: Check limit switches
  if (digitalRead(LIMIT_LEFT) == LOW && voltage > 0) {
    voltage = 0;
    pwmValue = 0;
  }
  if (digitalRead(LIMIT_RIGHT) == LOW && voltage < 0) {
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
  if (!setupComplete) return;

  Serial.print(F("R"));
  Serial.print(currentRound);
  Serial.print(F(" | "));

  switch (currentState) {
    case CALIBRATE:
      Serial.print(F("CAL"));
      break;
    case CHOOSE_ACTIVE_TARGET:
      Serial.print(F("CHOOSE"));
      break;
    case MOVE_TO_TARGET:
      Serial.print(F("MOVE"));
      break;
    default:
      Serial.print(F("SETUP"));
      break;
  }

  Serial.print(F(" | Pos:"));
  Serial.print(motorEncoder.read());

  if (activeTarget >= 0) {
    Serial.print(F(" | Tgt:"));
    Serial.print(activeTarget + 1);
    Serial.print(F("("));
    Serial.print(currentTargetPosition);
    Serial.print(F(")"));
  }

  Serial.print(F(" | Score:"));
  Serial.print(totalScore);

  Serial.print(F(" | Sensors:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(sensors[i].rawValue);
    Serial.print(sensors[i].direction == APPROACHING ? "↑" :
                 sensors[i].direction == LEAVING ? "↓" : "•");
    if (i < 3) Serial.print(",");
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
