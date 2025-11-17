/*
 * ME350 PID Auto-Tune System
 *
 * Comprehensive PID tuning tool for ME350 mechatronics project
 * Implements multiple industry-standard tuning methods
 *
 * Features:
 * - Automatic range calibration
 * - Directional friction characterization
 * - Ziegler-Nichols relay oscillation method
 * - Multiple tuning presets (Conservative, Classic, Aggressive)
 * - EEPROM persistent storage
 * - Step response analysis
 * - Manual position testing
 *
 * Compatible with standard ME350 hardware configuration
 */

#include <Encoder.h>
#include <EEPROM.h>

// ============================================================================
// PIN CONFIGURATION (Standard ME350 Setup)
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

// Limit Switches
#define LIMIT_LEFT 8   // Zero position
#define LIMIT_RIGHT 9  // Maximum range

// Proximity Sensors (optional for this tuning code)
#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Encoder motorEncoder(ENCODER_A, ENCODER_B);

// ============================================================================
// PID CONTROLLER PARAMETERS
// ============================================================================

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

float FRICTION_LEFT = 2.2;   // Voltage to overcome friction moving LEFT
float FRICTION_RIGHT = 2.9;  // Voltage to overcome friction moving RIGHT

// ============================================================================
// CALIBRATION DATA
// ============================================================================

long LEFT_LIMIT_POSITION = 0;
long RIGHT_LIMIT_POSITION = -1800;
long TOTAL_RANGE = 0;

bool isCalibrated = false;

// ============================================================================
// EEPROM STORAGE
// ============================================================================

const int EEPROM_CALIBRATION_FLAG = 0;
const int EEPROM_KP = 1;
const int EEPROM_KI = 5;
const int EEPROM_KD = 9;
const int EEPROM_FRICTION_LEFT = 13;
const int EEPROM_FRICTION_RIGHT = 17;
const int EEPROM_LEFT_LIMIT = 21;
const int EEPROM_RIGHT_LIMIT = 25;

// ============================================================================
// AUTO-TUNE RESULTS
// ============================================================================

struct TuneResults {
  float Ku;           // Ultimate gain
  float Tu;           // Ultimate period
  float amplitude;    // Oscillation amplitude
  int peakCount;      // Number of peaks collected
  bool valid;         // Results validity flag
};

TuneResults lastTuneResults = {0, 0, 0, 0, false};

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

   // Configure flip switch with pull-up so HIGH = on by default
  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);

  // Stop motor initially
  setMotorVoltage(0);

  // Load calibration from EEPROM
  loadCalibration();

  // Print welcome message
  printWelcome();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    processCommand(command);
  }
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(char cmd) {
  switch (toupper(cmd)) {
    case 'R':
      calibrateRange();
      break;

    case 'F':
      characterizeFriction();
      break;

    case 'Z':
      autoTuneZieglerNichols();
      break;

    case 'S':
      stepResponse();
      break;

    case 'T':
      manualPositionTest();
      break;

    case 'H':
      homeToLeft();
      break;

    case 'P':
      printStatus();
      break;

    case 'C':
      clearCalibration();
      break;

    case '?':
    case '/':
      printHelp();
      break;

    default:
      Serial.println(F("Unknown command. Press '?' for help."));
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
  // Master override
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

  // Check limit switches and stop if hit
  if (digitalRead(LIMIT_LEFT) == LOW && voltage > 0) {
    voltage = 0;
    pwmValue = 0;
  }
  if (digitalRead(LIMIT_RIGHT) == LOW && voltage < 0) {
    voltage = 0;
    pwmValue = 0;
  }

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
    // Moving RIGHT (negative direction)
    frictionComp = -FRICTION_LEFT;
  }
  else if (error > DEADBAND) {
    // Moving LEFT (positive direction)
    frictionComp = FRICTION_RIGHT;
  }

  // Calculate total voltage
  float voltage = pidOutput + frictionComp;

  // Store for next iteration
  lastError = error;

  return voltage;
}

// ============================================================================
// RANGE CALIBRATION
// ============================================================================

void calibrateRange() {
  Serial.println(F("\n=== RANGE CALIBRATION ==="));
  Serial.println(F("Finding left and right limit positions..."));

  // Move to left limit
  Serial.println(F("Moving to left limit..."));
  setMotorVoltage(5.0);

  unsigned long startTime = millis();
  long lastPosition = motorEncoder.read();

  while (digitalRead(LIMIT_LEFT) == HIGH) {
    if (millis() - startTime > 10000) {
      Serial.println(F("ERROR: Timeout waiting for left limit"));
      setMotorVoltage(0);
      return;
    }
    delay(10);
  }

  // Wait for motor to stop
  delay(200);
  setMotorVoltage(0);
  delay(100);

  // Zero the encoder
  motorEncoder.write(0);
  LEFT_LIMIT_POSITION = 0;
  Serial.println(F("Left limit found, encoder zeroed."));

  // Move to right limit
  Serial.println(F("Moving to right limit..."));
  delay(500);
  setMotorVoltage(-5.0);

  startTime = millis();
  lastPosition = 0;

  while (digitalRead(LIMIT_RIGHT) == HIGH) {
    if (millis() - startTime > 10000) {
      Serial.println(F("ERROR: Timeout waiting for right limit"));
      setMotorVoltage(0);
      return;
    }
    delay(10);
  }

  // Wait for motor to stop
  delay(200);
  setMotorVoltage(0);
  delay(100);

  // Record right limit position
  RIGHT_LIMIT_POSITION = motorEncoder.read();
  TOTAL_RANGE = abs(RIGHT_LIMIT_POSITION - LEFT_LIMIT_POSITION);

  Serial.print(F("Right limit found at: "));
  Serial.print(RIGHT_LIMIT_POSITION);
  Serial.println(F(" counts"));

  Serial.print(F("Total range: "));
  Serial.print(TOTAL_RANGE);
  Serial.println(F(" encoder counts"));

  if (TOTAL_RANGE < 100) {
    Serial.println(F("ERROR: Range too small. Check limit switches."));
    return;
  }

  isCalibrated = true;
  saveCalibration();

  Serial.println(F("Range calibration complete!"));

  // Return to center
  homeToLeft();
}

// ============================================================================
// FRICTION CHARACTERIZATION
// ============================================================================

void characterizeFriction() {
  Serial.println(F("\n=== FRICTION CHARACTERIZATION ==="));

  if (!isCalibrated) {
    Serial.println(F("ERROR: Must calibrate range first (command 'R')"));
    return;
  }

  // Home to left limit first
  homeToLeft();
  delay(1000);

  // Characterize RIGHT friction (moving from left to right)
  Serial.println(F("\nMeasuring friction for RIGHT movement..."));
  Serial.println(F("Finding minimum voltage to overcome static friction..."));

  float testVoltage = 0.5;
  bool motionDetected = false;

  while (!motionDetected && testVoltage < 8.0) {
    long startPos = motorEncoder.read();
    setMotorVoltage(-testVoltage);  // Negative = RIGHT
    delay(300);
    long endPos = motorEncoder.read();
    setMotorVoltage(0);
    delay(200);

    long movement = abs(endPos - startPos);

    if (movement > 10) {
      motionDetected = true;
      FRICTION_LEFT = testVoltage - 0.2;  // Subtract safety margin
      Serial.print(F("RIGHT friction voltage: "));
      Serial.print(FRICTION_LEFT, 2);
      Serial.println(F(" V"));
    }
    else {
      testVoltage += 0.25;
    }
  }

  if (!motionDetected) {
    Serial.println(F("ERROR: Could not detect motion. Check motor connection."));
    return;
  }

  // Move to right limit
  Serial.println(F("Moving to right limit..."));
  setMotorVoltage(-6.0);
  while (digitalRead(LIMIT_RIGHT) == HIGH) {
    delay(10);
  }
  setMotorVoltage(0);
  delay(500);

  // Characterize LEFT friction (moving from right to left)
  Serial.println(F("\nMeasuring friction for LEFT movement..."));

  testVoltage = 0.5;
  motionDetected = false;

  while (!motionDetected && testVoltage < 8.0) {
    long startPos = motorEncoder.read();
    setMotorVoltage(testVoltage);  // Positive = LEFT
    delay(300);
    long endPos = motorEncoder.read();
    setMotorVoltage(0);
    delay(200);

    long movement = abs(endPos - startPos);

    if (movement > 10) {
      motionDetected = true;
      FRICTION_RIGHT = testVoltage - 0.2;  // Subtract safety margin
      Serial.print(F("LEFT friction voltage: "));
      Serial.print(FRICTION_RIGHT, 2);
      Serial.println(F(" V"));
    }
    else {
      testVoltage += 0.25;
    }
  }

  saveCalibration();

  Serial.println(F("\n=== Friction Characterization Complete ==="));
  Serial.print(F("FRICTION_LEFT (moving RIGHT): "));
  Serial.print(FRICTION_LEFT, 2);
  Serial.println(F(" V"));
  Serial.print(F("FRICTION_RIGHT (moving LEFT): "));
  Serial.print(FRICTION_RIGHT, 2);
  Serial.println(F(" V"));

  // Return home
  homeToLeft();
}

// ============================================================================
// ZIEGLER-NICHOLS AUTO-TUNE
// ============================================================================

void autoTuneZieglerNichols() {
  Serial.println(F("\n=== ZIEGLER-NICHOLS AUTO-TUNE ==="));

  if (!isCalibrated) {
    Serial.println(F("ERROR: Must calibrate range first (command 'R')"));
    return;
  }

  Serial.println(F("This will perform relay oscillation method to find Ku and Tu."));
  Serial.println(F("Ensure system is clear and ready to move."));
  Serial.println(F("Press 'Y' to continue or any other key to cancel..."));

  while (!Serial.available()) { }
  char response = Serial.read();
  if (toupper(response) != 'Y') {
    Serial.println(F("Auto-tune cancelled."));
    return;
  }

  // Move to center position
  long centerPosition = (LEFT_LIMIT_POSITION + RIGHT_LIMIT_POSITION) / 2;
  Serial.print(F("Moving to center position: "));
  Serial.println(centerPosition);

  // Use simple proportional control to get to center
  unsigned long moveStart = millis();
  while (abs(motorEncoder.read() - centerPosition) > 20 && millis() - moveStart < 10000) {
    long currentPos = motorEncoder.read();
    long err = centerPosition - currentPos;
    float voltage = constrain(err * 0.01, -6.0, 6.0);
    setMotorVoltage(voltage);
    delay(10);
  }
  setMotorVoltage(0);
  delay(500);

  // Relay parameters
  const float TEST_VOLTAGE = 5.0;  // Relay amplitude
  const long HYSTERESIS = TOTAL_RANGE / 6;  // Oscillation band
  const int REQUIRED_PEAKS = 16;
  const unsigned long TIMEOUT = 60000;  // 60 seconds

  Serial.println(F("\nStarting relay oscillation test..."));
  Serial.print(F("Test voltage: "));
  Serial.println(TEST_VOLTAGE);
  Serial.print(F("Hysteresis band: ±"));
  Serial.println(HYSTERESIS);

  // Peak detection
  long peaks[REQUIRED_PEAKS];
  unsigned long peakTimes[REQUIRED_PEAKS];
  int peakCount = 0;
  bool lastAboveCenter = false;
  long lastCrossPosition = centerPosition;
  unsigned long lastCrossTime = millis();

  // Start oscillation
  unsigned long testStart = millis();
  bool relayState = true;  // Start moving up

  while (peakCount < REQUIRED_PEAKS && millis() - testStart < TIMEOUT) {
    long currentPos = motorEncoder.read();
    long deviation = currentPos - centerPosition;

    // Relay logic with hysteresis
    if (deviation > HYSTERESIS) {
      relayState = false;  // Switch to negative voltage
    }
    else if (deviation < -HYSTERESIS) {
      relayState = true;   // Switch to positive voltage
    }

    // Apply relay voltage
    if (relayState) {
      setMotorVoltage(TEST_VOLTAGE);
    }
    else {
      setMotorVoltage(-TEST_VOLTAGE);
    }

    // Detect center crossings
    bool currentAboveCenter = (currentPos > centerPosition);

    if (currentAboveCenter != lastAboveCenter) {
      // Center crossing detected
      unsigned long crossTime = millis();

      // Record peak (skip first 4 for settling)
      if (peakCount >= 4 && peakCount < REQUIRED_PEAKS + 4) {
        peaks[peakCount - 4] = abs(lastCrossPosition - centerPosition);
        peakTimes[peakCount - 4] = crossTime - lastCrossTime;
      }

      peakCount++;
      lastCrossPosition = currentPos;
      lastCrossTime = crossTime;

      if (peakCount % 4 == 0) {
        Serial.print(F("Peaks collected: "));
        Serial.println(peakCount);
      }
    }

    lastAboveCenter = currentAboveCenter;

    delay(5);  // Short delay for control loop
  }

  setMotorVoltage(0);

  if (peakCount < REQUIRED_PEAKS + 4) {
    Serial.println(F("ERROR: Insufficient peaks collected for reliable tuning."));
    Serial.print(F("Collected "));
    Serial.print(peakCount);
    Serial.print(F(" peaks, needed "));
    Serial.println(REQUIRED_PEAKS + 4);
    lastTuneResults.valid = false;
    return;
  }

  // Calculate average amplitude
  long sumAmplitude = 0;
  for (int i = 0; i < REQUIRED_PEAKS; i++) {
    sumAmplitude += peaks[i];
  }
  float avgAmplitude = sumAmplitude / (float)REQUIRED_PEAKS;

  // Calculate average period
  unsigned long sumPeriod = 0;
  for (int i = 0; i < REQUIRED_PEAKS - 1; i++) {
    sumPeriod += peakTimes[i];
  }
  float avgPeriod = (sumPeriod / (float)(REQUIRED_PEAKS - 1)) / 1000.0;  // Convert to seconds

  // Calculate Ku (ultimate gain)
  // Ku = 4 * V / (π * amplitude)
  float Ku = (4.0 * TEST_VOLTAGE) / (PI * avgAmplitude);

  // Store results
  lastTuneResults.Ku = Ku;
  lastTuneResults.Tu = avgPeriod * 2;  // Full period is 2 half-periods
  lastTuneResults.amplitude = avgAmplitude;
  lastTuneResults.peakCount = REQUIRED_PEAKS;
  lastTuneResults.valid = true;

  // Display results
  Serial.println(F("\n=== AUTO-TUNE RESULTS ==="));
  Serial.print(F("Peaks collected: "));
  Serial.println(REQUIRED_PEAKS);
  Serial.print(F("Average amplitude: "));
  Serial.print(avgAmplitude);
  Serial.println(F(" encoder counts"));
  Serial.print(F("Average half-period: "));
  Serial.print(avgPeriod, 3);
  Serial.println(F(" seconds"));
  Serial.print(F("Ultimate period (Tu): "));
  Serial.print(lastTuneResults.Tu, 3);
  Serial.println(F(" seconds"));
  Serial.print(F("Ultimate gain (Ku): "));
  Serial.println(Ku, 4);

  // Offer tuning options
  Serial.println(F("\n=== TUNING OPTIONS ==="));
  Serial.println(F("Select a tuning method:"));
  Serial.println(F("1. Conservative (30% of ZN) - RECOMMENDED for stability"));
  Serial.println(F("2. Classic Ziegler-Nichols (Full ZN) - Balanced"));
  Serial.println(F("3. Aggressive (80% of ZN) - Fast response"));
  Serial.println(F("4. PD-Only (No integral) - Prevents windup"));
  Serial.println(F("5. Cancel - Don't apply gains"));

  Serial.println(F("\nEnter selection (1-5):"));

  while (!Serial.available()) { }
  char selection = Serial.read();

  float newKp, newKi, newKd;

  switch (selection) {
    case '1':  // Conservative
      newKp = 0.3 * 0.6 * Ku;
      newKi = 0.3 * 1.2 * Ku / lastTuneResults.Tu;
      newKd = 0.3 * 0.075 * Ku * lastTuneResults.Tu;
      Serial.println(F("Applying CONSERVATIVE gains (30% of ZN)..."));
      break;

    case '2':  // Classic ZN
      newKp = 0.6 * Ku;
      newKi = 1.2 * Ku / lastTuneResults.Tu;
      newKd = 0.075 * Ku * lastTuneResults.Tu;
      Serial.println(F("Applying CLASSIC ZIEGLER-NICHOLS gains..."));
      break;

    case '3':  // Aggressive
      newKp = 0.8 * 0.6 * Ku;
      newKi = 0.8 * 1.2 * Ku / lastTuneResults.Tu;
      newKd = 0.8 * 0.075 * Ku * lastTuneResults.Tu;
      Serial.println(F("Applying AGGRESSIVE gains (80% of ZN)..."));
      break;

    case '4':  // PD-Only
      newKp = 0.6 * Ku;
      newKi = 0.0;
      newKd = 0.125 * Ku * lastTuneResults.Tu;
      Serial.println(F("Applying PD-ONLY gains (no integral)..."));
      break;

    default:
      Serial.println(F("Auto-tune cancelled. Gains not changed."));
      return;
  }

  KP = newKp;
  KI = newKi;
  KD = newKd;

  Serial.println(F("\n=== NEW PID GAINS ==="));
  Serial.print(F("Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("Kd = "));
  Serial.println(KD, 6);

  saveCalibration();

  Serial.println(F("\nGains saved to EEPROM."));
  Serial.println(F("Use 'T' command to test these gains."));
}

// ============================================================================
// STEP RESPONSE ANALYSIS
// ============================================================================

void stepResponse() {
  Serial.println(F("\n=== STEP RESPONSE ANALYSIS ==="));

  if (!isCalibrated) {
    Serial.println(F("ERROR: Must calibrate range first (command 'R')"));
    return;
  }

  Serial.println(F("This will apply a step voltage and record the response."));
  Serial.println(F("Press 'Y' to continue..."));

  while (!Serial.available()) { }
  char response = Serial.read();
  if (toupper(response) != 'Y') {
    Serial.println(F("Cancelled."));
    return;
  }

  // Move to a known starting position
  homeToLeft();
  delay(1000);

  Serial.println(F("\nApplying step voltage of 5.0V..."));
  Serial.println(F("Time(ms),Position(counts)"));

  unsigned long startTime = millis();
  long startPosition = motorEncoder.read();

  setMotorVoltage(5.0);

  // Record for 3 seconds or until right limit
  while (millis() - startTime < 3000 && digitalRead(LIMIT_RIGHT) == HIGH) {
    unsigned long elapsed = millis() - startTime;
    long position = motorEncoder.read();

    Serial.print(elapsed);
    Serial.print(",");
    Serial.println(position);

    delay(50);  // Sample every 50ms
  }

  setMotorVoltage(0);

  long finalPosition = motorEncoder.read();
  long totalMovement = abs(finalPosition - startPosition);

  Serial.println(F("\n=== STEP RESPONSE COMPLETE ==="));
  Serial.print(F("Total movement: "));
  Serial.print(totalMovement);
  Serial.println(F(" encoder counts"));

  // Return home
  delay(1000);
  homeToLeft();
}

// ============================================================================
// MANUAL POSITION TEST
// ============================================================================

void manualPositionTest() {
  Serial.println(F("\n=== MANUAL POSITION TEST ==="));

  if (!isCalibrated) {
    Serial.println(F("ERROR: Must calibrate range first (command 'R')"));
    return;
  }

  Serial.println(F("Current PID gains:"));
  Serial.print(F("Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("Kd = "));
  Serial.println(KD, 6);

  Serial.print(F("\nEnter target position ("));
  Serial.print(RIGHT_LIMIT_POSITION);
  Serial.print(F(" to "));
  Serial.print(LEFT_LIMIT_POSITION);
  Serial.println(F("):"));

  while (!Serial.available()) { }
  long targetPosition = Serial.parseInt();

  // Clear serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  // Validate target
  if (targetPosition < RIGHT_LIMIT_POSITION || targetPosition > LEFT_LIMIT_POSITION) {
    Serial.println(F("ERROR: Target position out of range."));
    return;
  }

  Serial.print(F("Moving to position: "));
  Serial.println(targetPosition);
  Serial.println(F("Press any key to stop test early."));
  Serial.println(F("\nTime(s),Position,Error,Integral,Voltage"));

  // Reset PID state
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;

  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  unsigned long settledTime = 0;
  bool hasSettled = false;

  while (millis() - startTime < 10000) {  // 10 second test
    // Update PID
    float voltage = updatePID(targetPosition);
    setMotorVoltage(voltage);

    // Print status every 100ms
    if (millis() - lastPrint >= 100) {
      float elapsedSec = (millis() - startTime) / 1000.0;
      long currentPos = motorEncoder.read();

      Serial.print(elapsedSec, 2);
      Serial.print(",");
      Serial.print(currentPos);
      Serial.print(",");
      Serial.print(error);
      Serial.print(",");
      Serial.print(integral, 2);
      Serial.print(",");
      Serial.println(voltage, 3);

      lastPrint = millis();
    }

    // Check if settled
    if (abs(error) < DEADBAND && !hasSettled) {
      settledTime = millis();
      hasSettled = true;
    }

    // Check for early termination
    if (Serial.available()) {
      break;
    }

    delay(CONTROL_PERIOD);
  }

  setMotorVoltage(0);

  Serial.println(F("\n=== TEST COMPLETE ==="));
  Serial.print(F("Final position: "));
  Serial.println(motorEncoder.read());
  Serial.print(F("Final error: "));
  Serial.println(error);

  if (hasSettled) {
    float settleTime = (settledTime - startTime) / 1000.0;
    Serial.print(F("Settling time: "));
    Serial.print(settleTime, 2);
    Serial.println(F(" seconds"));
  }
  else {
    Serial.println(F("Warning: Did not settle within test period."));
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void homeToLeft() {
  Serial.println(F("Homing to left limit..."));

  setMotorVoltage(6.0);

  unsigned long startTime = millis();
  while (digitalRead(LIMIT_LEFT) == HIGH) {
    if (millis() - startTime > 10000) {
      Serial.println(F("ERROR: Timeout during homing"));
      setMotorVoltage(0);
      return;
    }
    delay(10);
  }

  setMotorVoltage(0);
  delay(200);

  motorEncoder.write(0);
  Serial.println(F("Homed. Encoder zeroed."));
}

void printStatus() {
  Serial.println(F("\n=== SYSTEM STATUS ==="));

  Serial.print(F("Calibrated: "));
  Serial.println(isCalibrated ? "YES" : "NO");

  Serial.print(F("Current position: "));
  Serial.println(motorEncoder.read());

  Serial.print(F("Left limit: "));
  Serial.println(LEFT_LIMIT_POSITION);
  Serial.print(F("Right limit: "));
  Serial.println(RIGHT_LIMIT_POSITION);
  Serial.print(F("Total range: "));
  Serial.println(TOTAL_RANGE);

  Serial.println(F("\n=== PID GAINS ==="));
  Serial.print(F("Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("Kd = "));
  Serial.println(KD, 6);

  Serial.println(F("\n=== FRICTION COMPENSATION ==="));
  Serial.print(F("LEFT (moving RIGHT): "));
  Serial.print(FRICTION_LEFT, 3);
  Serial.println(F(" V"));
  Serial.print(F("RIGHT (moving LEFT): "));
  Serial.print(FRICTION_RIGHT, 3);
  Serial.println(F(" V"));

  if (lastTuneResults.valid) {
    Serial.println(F("\n=== LAST AUTO-TUNE RESULTS ==="));
    Serial.print(F("Ku = "));
    Serial.println(lastTuneResults.Ku, 4);
    Serial.print(F("Tu = "));
    Serial.print(lastTuneResults.Tu, 3);
    Serial.println(F(" s"));
    Serial.print(F("Amplitude = "));
    Serial.println(lastTuneResults.amplitude);
    Serial.print(F("Peaks = "));
    Serial.println(lastTuneResults.peakCount);
  }
}

void printHelp() {
  Serial.println(F("\n=== ME350 PID AUTO-TUNE - COMMAND REFERENCE ==="));
  Serial.println(F("R - Calibrate Range (find left/right limits)"));
  Serial.println(F("F - Friction Characterization (measure breakaway voltages)"));
  Serial.println(F("Z - Ziegler-Nichols Auto-Tune (relay oscillation method)"));
  Serial.println(F("S - Step Response Analysis (voltage step test)"));
  Serial.println(F("T - Manual Position Test (test current PID gains)"));
  Serial.println(F("H - Home (return to left limit and zero encoder)"));
  Serial.println(F("P - Print Status (show all current settings)"));
  Serial.println(F("C - Clear Calibration (erase EEPROM data)"));
  Serial.println(F("? - Help (show this menu)"));

  Serial.println(F("\n=== RECOMMENDED WORKFLOW ==="));
  Serial.println(F("1. R - Calibrate range"));
  Serial.println(F("2. F - Characterize friction"));
  Serial.println(F("3. Z - Auto-tune PID (select Conservative)"));
  Serial.println(F("4. T - Test with manual position"));
  Serial.println(F("5. Copy gains to your main project code"));
}

void printWelcome() {
  Serial.println(F("\n========================================"));
  Serial.println(F("   ME350 PID AUTO-TUNE SYSTEM"));
  Serial.println(F("========================================"));
  Serial.println(F("Comprehensive PID tuning tool"));
  Serial.println(F("Press '?' for command list"));
  Serial.println(F("========================================\n"));

  if (isCalibrated) {
    Serial.println(F("Calibration loaded from EEPROM."));
    Serial.println(F("Use 'P' to view current settings."));
  }
  else {
    Serial.println(F("System not calibrated."));
    Serial.println(F("Run 'R' command to calibrate range."));
  }
}

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================

void saveCalibration() {
  EEPROM.write(EEPROM_CALIBRATION_FLAG, 0xAA);  // Magic number
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.put(EEPROM_LEFT_LIMIT, LEFT_LIMIT_POSITION);
  EEPROM.put(EEPROM_RIGHT_LIMIT, RIGHT_LIMIT_POSITION);

  Serial.println(F("Calibration saved to EEPROM."));
}

void loadCalibration() {
  byte flag = EEPROM.read(EEPROM_CALIBRATION_FLAG);

  if (flag == 0xAA) {
    EEPROM.get(EEPROM_KP, KP);
    EEPROM.get(EEPROM_KI, KI);
    EEPROM.get(EEPROM_KD, KD);
    EEPROM.get(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
    EEPROM.get(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
    EEPROM.get(EEPROM_LEFT_LIMIT, LEFT_LIMIT_POSITION);
    EEPROM.get(EEPROM_RIGHT_LIMIT, RIGHT_LIMIT_POSITION);

    TOTAL_RANGE = abs(RIGHT_LIMIT_POSITION - LEFT_LIMIT_POSITION);
    isCalibrated = (TOTAL_RANGE > 100);
  }
}

void clearCalibration() {
  Serial.println(F("Clearing EEPROM calibration data..."));

  EEPROM.write(EEPROM_CALIBRATION_FLAG, 0x00);

  // Reset to defaults
  KP = 0.020;
  KI = 0.005;
  KD = 0.004;
  FRICTION_LEFT = 2.2;
  FRICTION_RIGHT = 2.9;
  LEFT_LIMIT_POSITION = 0;
  RIGHT_LIMIT_POSITION = -1800;
  TOTAL_RANGE = 0;
  isCalibrated = false;

  Serial.println(F("Calibration cleared. Defaults restored."));
  Serial.println(F("Run 'R' to recalibrate."));
}
