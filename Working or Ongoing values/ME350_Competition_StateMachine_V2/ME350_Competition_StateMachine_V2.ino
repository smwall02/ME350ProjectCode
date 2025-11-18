// ============================================================================
// ME 350 - Plants vs Zombies - STATE MACHINE V2
// Built from ground up based on Lab 11 Tutorial
// Clean, simple, robust implementation
// ============================================================================
//
// This code implements a 3-state machine as specified in Lab 11:
//   1. CALIBRATE - Home to left limit switch and zero encoder
//   2. CHOOSE_ACTIVE_TARGET - Determine which zombie to target
//   3. MOVE_TO_TARGET - Move flashlight to target and dwell
//
// Key improvements over V1:
//   - Follows Lab 11 tutorial structure exactly
//   - Consistent limit switch logic (active LOW with INPUT_PULLUP)
//   - Simplified target selection
//   - Proper dwell time to ensure zombies are pushed back
//   - Clear comments and maintainable code
//
// ============================================================================

#include <Encoder.h>

// ============================================
// PIN DEFINITIONS (Per Lab 11 Tutorial)
// ============================================
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_ENA 11
#define MOTOR_IN2 12
#define MOTOR_IN3 13
#define LIMIT_LEFT 8
#define LIMIT_RIGHT 9

#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

Encoder encoder(ENCODER_A, ENCODER_B);

// ============================================
// STATE MACHINE DEFINITIONS
// ============================================
enum State {
  CALIBRATE = 0,
  CHOOSE_ACTIVE_TARGET = 1,
  MOVE_TO_TARGET = 2
};

State currentState = CALIBRATE;

// Direction constants for zombie movement
const int FORWARD = 1;   // Moving toward photosensor (THREAT!)
const int BACKWARD = -1; // Moving away from photosensor
const int STOPPED = 0;   // Not moving

// ============================================
// TARGET POSITION CALIBRATION
// ============================================
// IMPORTANT: Calibrate these values for your specific setup!
// Use the manual calibration mode ('C' command) to find correct positions
long TARGET_1_POSITION = -80;
long TARGET_2_POSITION = -330;
long TARGET_3_POSITION = -590;
long TARGET_4_POSITION = -1230;

long targetPositions[4] = {
  TARGET_1_POSITION,
  TARGET_2_POSITION,
  TARGET_3_POSITION,
  TARGET_4_POSITION
};

// Wait position (center position when no targets active)
long WAIT_POSITION = TARGET_2_POSITION;

// Travel limits (calibrated during homing)
long LOWER_BOUND = 0;           // Left limit (home position)
long UPPER_BOUND = -1400;       // Right limit (approximate)

// ============================================
// PROXIMITY SENSOR CONFIGURATION
// ============================================
struct ProximitySensor {
  int pin;                    // Analog pin
  float currVal;              // Current filtered reading
  float prevVal;              // Previous value for change detection
  int direction;              // FORWARD, BACKWARD, or STOPPED
  unsigned long lastChangeTime; // Time of last significant change

  // Calibration values (min/max observed)
  int minObserved;
  int maxObserved;
};

ProximitySensor ProxSensors[4];

// Sensor filtering and noise parameters
const float SENSOR_ALPHA = 0.85;              // Smoothing factor (higher = more smoothing)
const int DIRECTION_THRESHOLD = 15;            // Minimum change to detect direction
const unsigned long STOP_TIMEOUT = 300;        // ms until considered stopped
const int MIN_SENSOR_RANGE = 80;               // Minimum range for valid calibration

// ============================================
// PID CONTROL PARAMETERS
// ============================================
// These gains are tuned for smooth, accurate positioning
// Adjust if your mechanism behaves differently
float KP = 0.020;
float KI = 0.005;
float KD = 0.004;

const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 0.8;
const int TARGET_BAND = 8;                     // "Close enough" tolerance (encoder counts)
const float MAX_INTEGRAL = 800.0;
const unsigned long CONTROL_PERIOD = 10;       // ms between control updates

// ============================================
// FRICTION COMPENSATION
// ============================================
float FRICTION_COMP_VOLTAGE = 2.2;            // Voltage to overcome static friction
float FRICTION_BIAS = 0.25;                    // Directional asymmetry compensation

// ============================================
// MOTION CONTROL STATE
// ============================================
long desiredPosition = 0;
float errorIntegral = 0;
float lastError = 0;

unsigned long lastControlTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastPrintTime = 0;

// ============================================
// TARGET SELECTION
// ============================================
int activeTargetIndex = -1;                    // Currently targeted lane (-1 = none)
long activeTargetPosition = WAIT_POSITION;
float closestZombieDist = 2.0;                 // Normalized distance (0=closest, 1=farthest)
float zombieDistances[4];                      // Normalized distance for each lane

// ============================================
// DWELL TIME (per Lab 11 tutorial requirement)
// ============================================
unsigned long targetReachedTime = 0;
const unsigned long MIN_DWELL_TIME = 500;      // Minimum time to keep light on target (ms)

// ============================================
// SYSTEM STATE
// ============================================
bool systemEnabled = false;
bool autoMode = false;
bool sensorsCalibrated = false;

const float CALIBRATION_VOLTAGE = 5.0;         // Voltage for homing

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);

  // Configure motor control pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

// Configure limit switches (wired to GND, active LOW with INPUT_PULLUP)
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);

  // Configure proximity sensors
  pinMode(PROX_SENSOR_1, INPUT);
  pinMode(PROX_SENSOR_2, INPUT);
  pinMode(PROX_SENSOR_3, INPUT);
  pinMode(PROX_SENSOR_4, INPUT);

  // Initialize proximity sensor data structures
  ProxSensors[0].pin = PROX_SENSOR_1;
  ProxSensors[1].pin = PROX_SENSOR_2;
  ProxSensors[2].pin = PROX_SENSOR_3;
  ProxSensors[3].pin = PROX_SENSOR_4;

  for (int i = 0; i < 4; i++) {
    int reading = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = reading;
    ProxSensors[i].prevVal = reading;
    ProxSensors[i].direction = STOPPED;
    ProxSensors[i].lastChangeTime = millis();
    ProxSensors[i].minObserved = 1023;
    ProxSensors[i].maxObserved = 0;
  }

  stopMotor();
  delay(500);

  printWelcome();
  printHelp();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();

  // Process serial commands
  if (Serial.available() > 0) {
    processCommand();
  }

  // Update sensors at regular intervals
  if (currentTime - lastSensorTime >= 20) {
    lastSensorTime = currentTime;
    updateAllSensors();
  }

  // Run control loop
  if (currentTime - lastControlTime >= CONTROL_PERIOD) {
    lastControlTime = currentTime;

    if (autoMode) {
      runStateMachine();
    }

    if (autoMode || systemEnabled) {
      runMotionControl();
    }

    // Periodic status output
    if (autoMode && (currentTime - lastPrintTime >= 500)) {
      lastPrintTime = currentTime;
      printCompactStatus();
    }
  }

  // Safety: Check limit switches
  checkLimitSwitches();
}

// ============================================
// STATE MACHINE (Per Lab 11 Tutorial)
// ============================================
void runStateMachine() {

  switch (currentState) {

    // ========================================
    // STATE: CALIBRATE
    // Purpose: Home to left limit and zero encoder
    // Transition: When limit switch pressed AND velocity ~0
    // ========================================
    case CALIBRATE:
      desiredPosition = LOWER_BOUND;

      // Check if we've reached the limit switch and stopped
      if (leftPressed() && abs(encoder.read()) < 10) {
        // Zero the encoder
        encoder.write(0);
        LOWER_BOUND = 0;

        // Reset state
        errorIntegral = 0;
        lastError = 0;

        Serial.println(F("✓ Calibrated! → CHOOSE_ACTIVE_TARGET"));

        // Transition to next state
        currentState = CHOOSE_ACTIVE_TARGET;
      }
      break;

    // ========================================
    // STATE: CHOOSE_ACTIVE_TARGET
    // Purpose: Determine which zombie to target
    // Method:
    //   1. Compute normalized distance for each sensor
    //   2. Find closest FORWARD-moving zombie
    //   3. Set target position
    // Transition: Immediately to MOVE_TO_TARGET
    // ========================================
    case CHOOSE_ACTIVE_TARGET:
      activeTargetIndex = -1;
      closestZombieDist = 2.0;  // Start with impossible value

      // Find the closest forward-moving zombie
      for (int i = 0; i < 4; i++) {
        // Calculate normalized distance (0 = at photosensor, 1 = far away)
        int range = ProxSensors[i].maxObserved - ProxSensors[i].minObserved;

        if (range > MIN_SENSOR_RANGE) {
          // We have good calibration data
          zombieDistances[i] = (ProxSensors[i].currVal - ProxSensors[i].minObserved) /
                               (float)range;
          zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);

          // Invert: lower sensor reading = closer to photosensor = more dangerous
          float threatLevel = 1.0 - zombieDistances[i];

          // Only target forward-moving zombies
          if (ProxSensors[i].direction == FORWARD) {
            if (threatLevel > (1.0 - closestZombieDist)) {
              closestZombieDist = 1.0 - threatLevel;
              activeTargetIndex = i;
            }
          }
        }
      }

      // Set target position
      if (activeTargetIndex >= 0) {
        activeTargetPosition = targetPositions[activeTargetIndex];

        Serial.print(F("🎯 Target Lane "));
        Serial.print(activeTargetIndex + 1);
        Serial.print(F(" (threat: "));
        Serial.print((int)((1.0 - closestZombieDist) * 100));
        Serial.println(F("%)"));
      } else {
        // No active targets, go to wait position
        activeTargetPosition = WAIT_POSITION;
        Serial.println(F("→ Wait position (no threats)"));
      }

      // Reset for new target
      desiredPosition = activeTargetPosition;
      errorIntegral = 0;
      targetReachedTime = 0;

      // Transition to MOVE_TO_TARGET
      currentState = MOVE_TO_TARGET;
      break;

    // ========================================
    // STATE: MOVE_TO_TARGET
    // Purpose: Move to target and dwell
    // Method:
    //   - PID controller moves to activeTargetPosition
    //   - Once within TARGET_BAND, start dwell timer
    //   - After MIN_DWELL_TIME, return to CHOOSE_ACTIVE_TARGET
    // ========================================
    case MOVE_TO_TARGET:
      desiredPosition = activeTargetPosition;

      long currentPos = encoder.read();
      long error = desiredPosition - currentPos;

      // Check if we're at the target
      if (abs(error) <= TARGET_BAND) {
        // Start dwell timer if not already started
        if (targetReachedTime == 0) {
          targetReachedTime = millis();
          Serial.println(F("✓ On target, dwelling..."));
        }

        // Check if we've dwelled long enough
        if (millis() - targetReachedTime >= MIN_DWELL_TIME) {
          Serial.println(F("✓ Dwell complete → CHOOSE_ACTIVE_TARGET"));

          // Transition back to choose next target
          currentState = CHOOSE_ACTIVE_TARGET;
          targetReachedTime = 0;
        }
      } else {
        // Not at target yet, reset dwell timer
        targetReachedTime = 0;
      }

      // Safety check: don't go past right limit
      if (currentPos < UPPER_BOUND + 50) {
        Serial.println(F("⚠️ Approaching right limit!"));
        currentState = CHOOSE_ACTIVE_TARGET;
      }
      break;
  }
}

// ============================================
// SENSOR UPDATE
// ============================================
void updateAllSensors() {
  for (int i = 0; i < 4; i++) {
    // Store previous value
    float previousReading = ProxSensors[i].currVal;

    // Read and filter sensor
    int rawReading = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = (SENSOR_ALPHA * ProxSensors[i].currVal) +
                             ((1.0 - SENSOR_ALPHA) * rawReading);

    // Update calibration bounds
    if (rawReading < ProxSensors[i].minObserved) {
      ProxSensors[i].minObserved = rawReading;
    }
    if (rawReading > ProxSensors[i].maxObserved) {
      ProxSensors[i].maxObserved = rawReading;
    }

    // Detect direction of movement
    float change = ProxSensors[i].currVal - previousReading;

    if (abs(change) > DIRECTION_THRESHOLD) {
      // Significant change detected
      if (change < 0) {
        // Sensor value decreasing = zombie moving toward photosensor
        ProxSensors[i].direction = FORWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].lastChangeTime = millis();
      } else {
        // Sensor value increasing = zombie moving away
        ProxSensors[i].direction = BACKWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].lastChangeTime = millis();
      }
    } else {
      // No significant change - check for timeout
      if (millis() - ProxSensors[i].lastChangeTime > STOP_TIMEOUT) {
        ProxSensors[i].direction = STOPPED;
      }
    }
  }

  // Mark sensors as calibrated after we've seen some variation
  if (!sensorsCalibrated) {
    bool allCalibrated = true;
    for (int i = 0; i < 4; i++) {
      int range = ProxSensors[i].maxObserved - ProxSensors[i].minObserved;
      if (range < MIN_SENSOR_RANGE) {
        allCalibrated = false;
        break;
      }
    }
    if (allCalibrated) {
      sensorsCalibrated = true;
      Serial.println(F("✓ Sensors calibrated!"));
    }
  }
}

// ============================================
// MOTION CONTROL (PID)
// ============================================
void runMotionControl() {
  long currentPosition = encoder.read();
  float error = desiredPosition - currentPosition;

  // If in calibrate state, override PID with constant voltage
  if (currentState == CALIBRATE) {
    // Apply constant voltage to move toward left limit until the switch closes
    if (!leftPressed()) {
      setMotor(CALIBRATION_VOLTAGE);
    } else {
      stopMotor();
    }
    return;
  }

  // Check if we're close enough to stop
  if (abs(error) <= TARGET_BAND) {
    stopMotor();
    errorIntegral = 0;
    return;
  }

  // Compute PID terms
  float dt = CONTROL_PERIOD / 1000.0;

  // Proportional
  float pTerm = KP * error;

  // Integral (with anti-windup)
  if (abs(error) < 200) {
    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  } else {
    errorIntegral *= 0.95;  // Decay integral when far from target
  }
  float iTerm = KI * errorIntegral;

  // Derivative
  float dTerm = KD * (error - lastError) / dt;
  lastError = error;

  // Combine PID terms
  float pidVoltage = pTerm + iTerm + dTerm;

  // Add friction compensation
  float frictionComp = 0;
  if (abs(error) > TARGET_BAND) {
    // Scale friction compensation based on error magnitude
    float frictionScale = 1.0;
    if (abs(error) < 30) {
      frictionScale = 0.6;
    } else if (abs(error) < 100) {
      frictionScale = 0.85;
    }

    if (error < 0) {
      // Moving toward more negative (right)
      frictionComp = -(FRICTION_COMP_VOLTAGE + FRICTION_BIAS) * frictionScale;
    } else {
      // Moving toward zero (left/home)
      frictionComp = FRICTION_COMP_VOLTAGE * frictionScale;
    }
  }

  // Compute total voltage
  float totalVoltage = pidVoltage + frictionComp;
  totalVoltage = constrain(totalVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);

  // Apply voltage (with deadband)
  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
  }
}

// ============================================
// MOTOR CONTROL
// ============================================
void setMotor(float voltage) {
  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;

  if (voltage > 0) {
    // Positive voltage = move LEFT (toward home)
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (voltage < 0) {
    // Negative voltage = move RIGHT (away from home)
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
  }

  analogWrite(MOTOR_ENA, pwm);
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
  // Left limit switch (active LOW with INPUT_PULLUP)
  if (leftPressed()) {
    // Only recalibrate if not already in calibrate state and not moving much
    if (currentState != CALIBRATE) {
      delay(50);  // Debounce
      if (leftPressed()) {
        encoder.write(0);
        LOWER_BOUND = 0;
        errorIntegral = 0;
        Serial.println(F("⚠️ Left limit - recalibrated"));
      }
    }
  }

  // Right limit switch (active LOW with INPUT_PULLUP)
  if (rightPressed()) {
    stopMotor();
    Serial.println(F("⚠️ RIGHT LIMIT - EMERGENCY STOP!"));

    if (autoMode) {
      // Back off and return to safe position
      delay(200);
      setMotor(3.0);  // Move left
      delay(300);
      stopMotor();

      desiredPosition = WAIT_POSITION;
      currentState = CHOOSE_ACTIVE_TARGET;
    }
  }
}

bool leftPressed() {
  // INPUT_PULLUP keeps the pin HIGH when open; pressed = LOW
  return digitalRead(LIMIT_LEFT) == LOW;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == LOW;
}

// ============================================
// HOMING ROUTINE
// ============================================
bool homeToLeftLimit() {
  Serial.println(F("\n🏠 HOMING TO LEFT LIMIT..."));

  // If already at limit, we're done
  if (leftPressed()) {
    Serial.println(F("Already at limit, zeroing encoder..."));
    stopMotor();
    delay(100);
    encoder.write(0);
    LOWER_BOUND = 0;
    Serial.println(F("✓ Homed\n"));
    return true;
  }

  // Move toward left limit
  unsigned long startTime = millis();
  setMotor(CALIBRATION_VOLTAGE);

  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }

  stopMotor();
  delay(200);

  if (leftPressed()) {
    encoder.write(0);
    LOWER_BOUND = 0;
    Serial.println(F("✓ Homed\n"));
    return true;
  } else {
    Serial.println(F("✗ Homing timeout\n"));
    return false;
  }
}

// ============================================
// MANUAL POSITION CALIBRATION
// ============================================
void manualCalibration() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   MANUAL TARGET POSITION CALIBRATION       ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();

  Serial.println(F("Commands:"));
  Serial.println(F("  R = Move Right"));
  Serial.println(F("  L = Move Left"));
  Serial.println(F("  S = Stop"));
  Serial.println(F("  1-4 = Save position as lane 1-4"));
  Serial.println(F("  Q = Quit\n"));

  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();

      cmd = toupper(cmd);
      long pos = encoder.read();

      switch (cmd) {
        case 'R':
          setMotor(-3.5);
          Serial.println(F("Moving RIGHT..."));
          break;

        case 'L':
          setMotor(3.5);
          Serial.println(F("Moving LEFT..."));
          break;

        case 'S':
          stopMotor();
          Serial.print(F("Stopped at: "));
          Serial.println(pos);
          break;

        case '1':
          stopMotor();
          TARGET_1_POSITION = pos;
          targetPositions[0] = pos;
          Serial.print(F("✓ Lane 1 = "));
          Serial.println(TARGET_1_POSITION);
          break;

        case '2':
          stopMotor();
          TARGET_2_POSITION = pos;
          targetPositions[1] = pos;
          Serial.print(F("✓ Lane 2 = "));
          Serial.println(TARGET_2_POSITION);
          break;

        case '3':
          stopMotor();
          TARGET_3_POSITION = pos;
          targetPositions[2] = pos;
          WAIT_POSITION = TARGET_3_POSITION;
          Serial.print(F("✓ Lane 3 = "));
          Serial.println(TARGET_3_POSITION);
          break;

        case '4':
          stopMotor();
          TARGET_4_POSITION = pos;
          targetPositions[3] = pos;
          Serial.print(F("✓ Lane 4 = "));
          Serial.println(TARGET_4_POSITION);
          break;

        case 'Q':
          stopMotor();
          Serial.println(F("\n✓ Calibration complete!\n"));
          Serial.println(F("Target positions:"));
          for (int i = 0; i < 4; i++) {
            Serial.print(F("  Lane "));
            Serial.print(i + 1);
            Serial.print(F(": "));
            Serial.println(targetPositions[i]);
          }
          Serial.println();
          systemEnabled = wasEnabled;
          autoMode = wasAuto;
          return;
      }
    }

    checkLimitSwitches();
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
    case 'G':  // GO - Start autonomous mode
      if (!autoMode) {
        Serial.println(F("\n🎮 STARTING AUTONOMOUS MODE"));

        if (homeToLeftLimit()) {
          autoMode = true;
          systemEnabled = true;
          currentState = CALIBRATE;
          errorIntegral = 0;
          lastError = 0;
          lastPrintTime = 0;

          Serial.println(F("✓ AUTO MODE ACTIVE\n"));
        } else {
          Serial.println(F("✗ Homing failed\n"));
        }
      }
      break;

    case 'S':  // STOP
      Serial.println(F("\n⏹ STOP\n"));
      autoMode = false;
      systemEnabled = false;
      stopMotor();
      errorIntegral = 0;
      break;

    case '1':
    case '2':
    case '3':
    case '4':  // Manual lane selection
      if (!autoMode) {
        int lane = cmd - '0';
        desiredPosition = targetPositions[lane - 1];
        systemEnabled = true;
        errorIntegral = 0;
        lastError = 0;

        Serial.print(F("\n→ Lane "));
        Serial.print(lane);
        Serial.print(F(" ("));
        Serial.print(desiredPosition);
        Serial.println(F(")"));
      } else {
        Serial.println(F("\n⚠️ Stop AUTO mode first (press 'S')\n"));
      }
      break;

    case 'P':  // Print status
      printStatus();
      break;

    case 'H':  // Help
      printHelp();
      break;

    case 'Z':  // Home (zero)
      homeToLeftLimit();
      break;

    case 'C':  // Calibrate positions
      manualCalibration();
      break;

    case 'D':  // Display sensors
      printSensorData();
      break;

    case 'M':  // Monitor mode
      continuousMonitor();
      break;
  }
}

// ============================================
// DISPLAY FUNCTIONS
// ============================================
void printWelcome() {
  Serial.println(F("\n\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   ME 350 - STATE MACHINE V2                ║"));
  Serial.println(F("║   Clean implementation per Lab 11          ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("COMMANDS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("  C      Calibrate target positions"));
  Serial.println(F("  Z      Home to left limit"));
  Serial.println(F("  G      Start AUTO mode"));
  Serial.println(F("  S      Stop"));
  Serial.println(F("  1-4    Manual lane control"));
  Serial.println(F("  P      Print status"));
  Serial.println(F("  D      Display sensors"));
  Serial.println(F("  M      Continuous monitor"));
  Serial.println(F("  H      Help"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void printCompactStatus() {
  long pos = encoder.read();

  Serial.print(F("State:"));
  switch(currentState) {
    case CALIBRATE: Serial.print(F("CAL")); break;
    case CHOOSE_ACTIVE_TARGET: Serial.print(F("CHOOSE")); break;
    case MOVE_TO_TARGET: Serial.print(F("MOVE")); break;
  }

  Serial.print(F(" | Pos:"));
  Serial.print(pos);
  Serial.print(F("→"));
  Serial.print(desiredPosition);

  if (activeTargetIndex >= 0) {
    Serial.print(F(" | Lane:"));
    Serial.print(activeTargetIndex + 1);
  }

  Serial.print(F(" | Zombies:["));
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction == FORWARD) {
      Serial.print(F("▶"));
    } else if (ProxSensors[i].direction == BACKWARD) {
      Serial.print(F("◀"));
    } else {
      Serial.print(F("■"));
    }
  }
  Serial.println(F("]"));
}

void printStatus() {
  long currentPos = encoder.read();

  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("SYSTEM STATUS"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  Serial.print(F("Mode:      "));
  Serial.println(autoMode ? F("AUTONOMOUS") : F("Manual"));

  Serial.print(F("State:     "));
  switch(currentState) {
    case CALIBRATE: Serial.println(F("CALIBRATE")); break;
    case CHOOSE_ACTIVE_TARGET: Serial.println(F("CHOOSE_ACTIVE_TARGET")); break;
    case MOVE_TO_TARGET: Serial.println(F("MOVE_TO_TARGET")); break;
  }

  Serial.print(F("Position:  "));
  Serial.println(currentPos);
  Serial.print(F("Target:    "));
  Serial.println(desiredPosition);
  Serial.print(F("Error:     "));
  Serial.println(desiredPosition - currentPos);

  Serial.println(F("\n━━ SENSORS ━━"));
  Serial.print(F("Calibrated: "));
  Serial.println(sensorsCalibrated ? F("YES") : F("NO"));

  for (int i = 0; i < 4; i++) {
    Serial.print(F("Lane "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print((int)ProxSensors[i].currVal);
    Serial.print(F(" ["));
    Serial.print(ProxSensors[i].minObserved);
    Serial.print(F("-"));
    Serial.print(ProxSensors[i].maxObserved);
    Serial.print(F("] "));

    switch(ProxSensors[i].direction) {
      case FORWARD: Serial.println(F("FORWARD ⚠️")); break;
      case BACKWARD: Serial.println(F("BACKWARD")); break;
      case STOPPED: Serial.println(F("STOPPED")); break;
    }
  }

  Serial.println(F("\n━━ PID GAINS ━━"));
  Serial.print(F("Kp = "));
  Serial.println(KP, 4);
  Serial.print(F("Ki = "));
  Serial.println(KI, 4);
  Serial.print(F("Kd = "));
  Serial.println(KD, 4);

  Serial.println(F("\n━━ TARGET POSITIONS ━━"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("Lane "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(targetPositions[i]);
  }

  Serial.println(F("\n━━ LIMIT SWITCHES ━━"));
  Serial.print(F("Left:  "));
  Serial.println(leftPressed() ? "PRESSED" : "open");
  Serial.print(F("Right: "));
  Serial.println(rightPressed() ? "PRESSED" : "open");

  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void printSensorData() {
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("SENSOR DATA"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  for (int i = 0; i < 4; i++) {
    Serial.print(F("Lane "));
    Serial.print(i + 1);
    Serial.print(F(": "));

    // Raw reading
    int raw = analogRead(ProxSensors[i].pin);
    Serial.print(F("Raw="));
    Serial.print(raw);

    // Filtered value
    Serial.print(F(", Filt="));
    Serial.print((int)ProxSensors[i].currVal);

    // Range
    Serial.print(F(", Range=["));
    Serial.print(ProxSensors[i].minObserved);
    Serial.print(F("-"));
    Serial.print(ProxSensors[i].maxObserved);
    Serial.print(F("]"));

    // Direction
    Serial.print(F(", Dir="));
    switch(ProxSensors[i].direction) {
      case FORWARD: Serial.println(F("FWD")); break;
      case BACKWARD: Serial.println(F("BACK")); break;
      case STOPPED: Serial.println(F("STOP")); break;
    }
  }

  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void continuousMonitor() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   CONTINUOUS MONITOR                       ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.println(F("Press any key to stop\n"));

  bool wasEnabled = systemEnabled;
  bool wasAuto = autoMode;
  systemEnabled = false;
  autoMode = false;
  stopMotor();

  while (!Serial.available()) {
    updateAllSensors();

    Serial.print(F("Enc:"));
    Serial.print(encoder.read());
    Serial.print(F(" | L:"));
    Serial.print(leftPressed() ? "1" : "0");
    Serial.print(F(" R:"));
    Serial.print(rightPressed() ? "1" : "0");
    Serial.print(F(" | Prox:["));

    for (int i = 0; i < 4; i++) {
      Serial.print((int)ProxSensors[i].currVal);
      if (i < 3) Serial.print(F(","));
    }
    Serial.print(F("] Dir:["));

    for (int i = 0; i < 4; i++) {
      if (ProxSensors[i].direction == FORWARD) Serial.print(F("▶"));
      else if (ProxSensors[i].direction == BACKWARD) Serial.print(F("◀"));
      else Serial.print(F("■"));
    }
    Serial.println(F("]"));

    delay(100);
  }

  while (Serial.available()) Serial.read();
  Serial.println(F("\n✓ Monitor stopped\n"));
  systemEnabled = wasEnabled;
  autoMode = wasAuto;
}
