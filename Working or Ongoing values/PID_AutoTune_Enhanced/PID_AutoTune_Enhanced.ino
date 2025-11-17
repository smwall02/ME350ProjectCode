// ============================================================================
// ME 350 - ENHANCED PID AUTO-TUNING SYSTEM
// Advanced PID Parameter Identification and Tuning
// ============================================================================
// This code provides comprehensive PID tuning using multiple methods:
// 1. Ziegler-Nichols Relay Method (oscillation-based)
// 2. Step Response Analysis
// 3. Friction Characterization
// 4. System Identification
// ============================================================================

#include <Encoder.h>
#include <EEPROM.h>

// ============================================
// PIN DEFINITIONS (Standardized across project)
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
// EEPROM ADDRESSES FOR PERSISTENT STORAGE
// ============================================
#define EEPROM_KP_ADDR 0
#define EEPROM_KI_ADDR 4
#define EEPROM_KD_ADDR 8
#define EEPROM_FRICTION_LEFT_ADDR 12
#define EEPROM_FRICTION_RIGHT_ADDR 16
#define EEPROM_CALIBRATED_FLAG 20
#define EEPROM_LEFT_LIMIT_ADDR 24
#define EEPROM_RIGHT_LIMIT_ADDR 28

// ============================================
// PID PARAMETERS
// ============================================
float KP = 0.020;
float KI = 0.005;
float KD = 0.004;

// ============================================
// FRICTION COMPENSATION
// ============================================
float FRICTION_LEFT = 1.6;   // Moving toward more negative positions
float FRICTION_RIGHT = 2.9;  // Moving toward less negative positions

// ============================================
// CONTROL PARAMETERS
// ============================================
const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 0.5;
const int DEADBAND = 5;
const float MAX_INTEGRAL = 500.0;
const unsigned long CONTROL_PERIOD = 10;  // 10ms = 100Hz

// ============================================
// SYSTEM LIMITS
// ============================================
long LEFT_LIMIT = 0;
long RIGHT_LIMIT = -1300;
bool limitsCalibrated = false;

// ============================================
// CONTROL STATE VARIABLES
// ============================================
long targetPosition = 0;
float errorIntegral = 0;
float lastError = 0;
unsigned long lastControlTime = 0;
bool systemEnabled = false;

// ============================================
// AUTO-TUNE PARAMETERS
// ============================================
struct TuneResults {
  float Ku;              // Ultimate gain
  float Tu;              // Ultimate period
  float amplitude;       // Oscillation amplitude
  int peakCount;         // Number of peaks detected
  float frictionLeft;    // Friction voltage (moving left)
  float frictionRight;   // Friction voltage (moving right)
  float systemDelay;     // Transport delay
  float timeConstant;    // System time constant
  bool valid;            // Results validity flag
};

TuneResults lastTuneResults;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);

  pinMode(PROX_SENSOR_1, INPUT);
  pinMode(PROX_SENSOR_2, INPUT);
  pinMode(PROX_SENSOR_3, INPUT);
  pinMode(PROX_SENSOR_4, INPUT);

  stopMotor();
  delay(500);

  printWelcome();
  loadCalibration();
  printHelp();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();

  if (Serial.available() > 0) {
    processCommand();
  }

  if (systemEnabled && (currentTime - lastControlTime >= CONTROL_PERIOD)) {
    lastControlTime = currentTime;
    runPIDControl();
  }

  checkLimitSwitches();
}

// ============================================
// PID CONTROL
// ============================================
void runPIDControl() {
  long currentPosition = encoder.read();
  float error = targetPosition - currentPosition;

  if (abs(error) <= DEADBAND) {
    stopMotor();
    errorIntegral = 0;
    return;
  }

  float dt = CONTROL_PERIOD / 1000.0;
  errorIntegral += error * dt;
  errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);

  float errorDerivative = (error - lastError) / dt;

  float pidVoltage = (KP * error) + (KI * errorIntegral) + (KD * errorDerivative);

  // Adaptive friction compensation based on direction
  float frictionComp = 0;
  if (abs(error) > DEADBAND) {
    if (error < 0) {
      // Moving toward more negative position (RIGHT)
      frictionComp = -FRICTION_LEFT;
    } else {
      // Moving toward less negative position (LEFT)
      frictionComp = FRICTION_RIGHT;
    }
  }

  float totalVoltage = pidVoltage + frictionComp;
  totalVoltage = constrain(totalVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);

  if (abs(totalVoltage) >= MIN_CONTROL_VOLTAGE) {
    setMotor(totalVoltage);
  } else {
    stopMotor();
  }

  lastError = error;
}

// ============================================
// MOTOR CONTROL
// ============================================
void setMotor(float voltage) {
  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;

  if (voltage > 0) {
    // Positive voltage = Move LEFT (toward 0)
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (voltage < 0) {
    // Negative voltage = Move RIGHT (away from 0)
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
// LIMIT SWITCH HANDLING
// ============================================
void checkLimitSwitches() {
  if (digitalRead(LIMIT_LEFT) == LOW) {
    stopMotor();
    encoder.write(0);
    LEFT_LIMIT = 0;
  }

  if (digitalRead(LIMIT_RIGHT) == LOW) {
    stopMotor();
  }
}

bool leftPressed() {
  return digitalRead(LIMIT_LEFT) == LOW;
}

bool rightPressed() {
  return digitalRead(LIMIT_RIGHT) == LOW;
}

// ============================================
// HOMING
// ============================================
bool homeToLeftLimit() {
  Serial.println(F("\n=== HOMING TO LEFT LIMIT ==="));

  if (leftPressed()) {
    Serial.println(F("Already at limit, backing off..."));
    setMotor(-3.0);
    delay(300);
    stopMotor();
    delay(200);
  }

  Serial.println(F("Moving to left limit..."));
  setMotor(5.0);

  unsigned long startTime = millis();
  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }

  if (leftPressed()) {
    stopMotor();
    delay(200);
    encoder.write(0);
    delay(50);
    encoder.write(0);
    LEFT_LIMIT = 0;

    Serial.println(F("✓ Homed successfully"));
    Serial.print(F("Encoder position: "));
    Serial.println(encoder.read());
    return true;
  } else {
    stopMotor();
    Serial.println(F("✗ Homing timeout"));
    return false;
  }
}

// ============================================
// RANGE CALIBRATION
// ============================================
bool calibrateRange() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   RANGE CALIBRATION                        ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  systemEnabled = false;
  stopMotor();
  delay(200);

  // Home to left
  if (!homeToLeftLimit()) {
    Serial.println(F("✗ Range calibration failed at homing"));
    return false;
  }

  LEFT_LIMIT = encoder.read();
  Serial.print(F("Left limit: "));
  Serial.println(LEFT_LIMIT);
  delay(500);

  // Move to right limit
  Serial.println(F("\nMoving to right limit..."));
  setMotor(-5.5);

  unsigned long startTime = millis();
  long lastPos = encoder.read();
  unsigned long stuckTime = 0;

  while (!rightPressed() && (millis() - startTime) < 20000) {
    delay(50);
    long currentPos = encoder.read();

    if (abs(currentPos - lastPos) < 3) {
      stuckTime += 50;
      if (stuckTime > 2000) {
        Serial.println(F("⚠ Movement stopped, assuming limit"));
        break;
      }
    } else {
      stuckTime = 0;
      lastPos = currentPos;
    }
  }

  stopMotor();
  delay(300);
  RIGHT_LIMIT = encoder.read();

  Serial.print(F("Right limit: "));
  Serial.println(RIGHT_LIMIT);

  long range = abs(RIGHT_LIMIT - LEFT_LIMIT);
  Serial.print(F("Total range: "));
  Serial.print(range);
  Serial.println(F(" encoder counts"));

  if (range > 100) {
    limitsCalibrated = true;
    saveCalibration();
    homeToLeftLimit();
    Serial.println(F("\n✓ Range calibration complete\n"));
    return true;
  } else {
    Serial.println(F("\n✗ Range too small, calibration failed\n"));
    return false;
  }
}

// ============================================
// FRICTION CHARACTERIZATION
// ============================================
bool characterizeFriction() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   FRICTION CHARACTERIZATION                ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  if (!limitsCalibrated) {
    Serial.println(F("⚠ Please calibrate range first (command 'R')"));
    return false;
  }

  systemEnabled = false;
  stopMotor();

  if (!homeToLeftLimit()) {
    return false;
  }

  delay(500);

  // Test LEFT friction (moving toward more negative)
  Serial.println(F("Testing LEFT friction (→ negative positions)..."));

  float testVoltage = 0.5;
  const float voltageStep = 0.15;
  const int movementThreshold = 20;
  bool motionDetected = false;

  FRICTION_LEFT = 0;

  while (testVoltage <= 6.0 && !motionDetected) {
    long startPos = encoder.read();

    Serial.print(F("  Testing "));
    Serial.print(testVoltage, 2);
    Serial.print(F("V... "));

    setMotor(-testVoltage);
    delay(400);

    long endPos = encoder.read();
    long movement = abs(endPos - startPos);

    stopMotor();
    delay(150);

    Serial.print(movement);
    Serial.println(F(" counts"));

    if (movement >= movementThreshold) {
      FRICTION_LEFT = testVoltage;
      motionDetected = true;
      Serial.print(F("  ✓ FRICTION_LEFT = "));
      Serial.print(FRICTION_LEFT, 2);
      Serial.println(F("V\n"));
    } else {
      testVoltage += voltageStep;
    }
  }

  if (!motionDetected) {
    Serial.println(F("  ⚠ Using default 1.6V\n"));
    FRICTION_LEFT = 1.6;
  }

  delay(500);

  // Move to middle for RIGHT friction test
  long middlePos = (LEFT_LIMIT + RIGHT_LIMIT) / 2;
  Serial.print(F("Moving to middle position ("));
  Serial.print(middlePos);
  Serial.println(F(")..."));

  systemEnabled = true;
  targetPosition = middlePos;
  errorIntegral = 0;
  lastError = 0;

  unsigned long moveStart = millis();
  while (abs(encoder.read() - targetPosition) > 20 && (millis() - moveStart) < 10000) {
    runPIDControl();
    delay(10);
  }

  stopMotor();
  systemEnabled = false;
  delay(500);

  Serial.print(F("Current position: "));
  Serial.println(encoder.read());

  // Test RIGHT friction (moving toward less negative / toward 0)
  Serial.println(F("\nTesting RIGHT friction (→ less negative/toward 0)..."));

  testVoltage = 0.5;
  motionDetected = false;
  FRICTION_RIGHT = 0;

  while (testVoltage <= 6.0 && !motionDetected) {
    long startPos = encoder.read();

    Serial.print(F("  Testing "));
    Serial.print(testVoltage, 2);
    Serial.print(F("V... "));

    setMotor(testVoltage);
    delay(400);

    long endPos = encoder.read();
    long movement = abs(endPos - startPos);

    stopMotor();
    delay(150);

    Serial.print(movement);
    Serial.println(F(" counts"));

    if (movement >= movementThreshold) {
      FRICTION_RIGHT = testVoltage;
      motionDetected = true;
      Serial.print(F("  ✓ FRICTION_RIGHT = "));
      Serial.print(FRICTION_RIGHT, 2);
      Serial.println(F("V\n"));
    } else {
      testVoltage += voltageStep;
    }
  }

  if (!motionDetected) {
    Serial.println(F("  ⚠ Using default 2.9V\n"));
    FRICTION_RIGHT = 2.9;
  }

  Serial.println(F("╔════════════════════════════════════════════╗"));
  Serial.println(F("║   FRICTION CHARACTERIZATION COMPLETE       ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.print(F("  LEFT (→neg):  "));
  Serial.print(FRICTION_LEFT, 2);
  Serial.println(F("V"));
  Serial.print(F("  RIGHT (→0):   "));
  Serial.print(FRICTION_RIGHT, 2);
  Serial.println(F("V\n"));

  saveCalibration();
  homeToLeftLimit();

  return true;
}

// ============================================
// ZIEGLER-NICHOLS RELAY AUTO-TUNE
// ============================================
bool autoTuneZN() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   ZIEGLER-NICHOLS RELAY AUTO-TUNE          ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  if (!limitsCalibrated) {
    Serial.println(F("⚠ Please calibrate range first (command 'R')"));
    return false;
  }

  Serial.println(F("This will perform oscillation-based PID tuning."));
  Serial.println(F("Press ENTER to continue, or any other key to cancel..."));

  while (!Serial.available()) {}
  char response = Serial.read();
  while (Serial.available()) Serial.read();

  if (response != '\n' && response != '\r') {
    Serial.println(F("\n✗ Cancelled\n"));
    return false;
  }

  systemEnabled = false;
  stopMotor();
  delay(500);

  // Home and find range
  if (!homeToLeftLimit()) {
    return false;
  }

  delay(500);

  long leftPos = encoder.read();
  long fullRange = abs(RIGHT_LIMIT - leftPos);

  Serial.print(F("Using range: "));
  Serial.print(fullRange);
  Serial.println(F(" counts\n"));

  if (fullRange < 100) {
    Serial.println(F("✗ Range too small!"));
    return false;
  }

  // Move to center
  Serial.println(F("Moving to center position..."));
  long centerPosition = (leftPos + RIGHT_LIMIT) / 2;

  unsigned long moveTimeout = millis();
  while (abs(encoder.read() - centerPosition) > 50 && (millis() - moveTimeout) < 15000) {
    long pos = encoder.read();
    long error = centerPosition - pos;

    if (error < 0) {
      setMotor(-5.0);
    } else {
      setMotor(5.0);
    }

    delay(10);
  }

  stopMotor();
  delay(1500);

  long actualCenter = encoder.read();
  Serial.print(F("Center position: "));
  Serial.println(actualCenter);
  Serial.println();

  // Relay oscillation test
  Serial.println(F("Starting relay oscillation test...\n"));

  int HYSTERESIS = fullRange / 6;
  const float TEST_VOLTAGE = 5.0;
  const int MAX_PEAKS = 20;
  const int REQUIRED_PEAKS = 16;
  const unsigned long TIMEOUT = 60000;

  Serial.print(F("Hysteresis: ±"));
  Serial.print(HYSTERESIS);
  Serial.println(F(" counts"));
  Serial.print(F("Test voltage: "));
  Serial.print(TEST_VOLTAGE, 1);
  Serial.println(F("V\n"));

  // Initial displacement
  Serial.println(F("Creating initial displacement..."));
  setMotor(-TEST_VOLTAGE);
  delay(1000);
  stopMotor();
  delay(500);

  Serial.print(F("Starting position: "));
  Serial.println(encoder.read());
  Serial.println();

  long peakPositions[MAX_PEAKS];
  unsigned long peakTimes[MAX_PEAKS];
  int peakCount = 0;

  long lastPos = encoder.read();
  bool crossedCenter = false;
  unsigned long startTime = millis();

  while (peakCount < REQUIRED_PEAKS && (millis() - startTime) < TIMEOUT) {
    long currentPos = encoder.read();
    long distanceFromCenter = currentPos - actualCenter;

    // Relay control
    if (distanceFromCenter < -HYSTERESIS) {
      setMotor(TEST_VOLTAGE);   // Move toward 0
      if (crossedCenter) {
        crossedCenter = false;
      }
    } else if (distanceFromCenter > HYSTERESIS) {
      setMotor(-TEST_VOLTAGE);  // Move away from 0
      if (crossedCenter) {
        crossedCenter = false;
      }
    }

    // Peak detection at center crossing
    if (!crossedCenter && abs(distanceFromCenter) < 50) {
      crossedCenter = true;

      unsigned long now = millis();
      peakPositions[peakCount] = lastPos;
      peakTimes[peakCount] = now;

      Serial.print(F("Peak "));
      Serial.print(peakCount + 1);
      Serial.print(F(": Position = "));
      Serial.print(lastPos);
      Serial.print(F(", Time = "));
      Serial.print((now - startTime) / 1000.0, 2);
      Serial.println(F("s"));

      peakCount++;
    }

    lastPos = currentPos;
    delay(5);

    if (Serial.available()) {
      Serial.println(F("\n✗ Aborted by user"));
      stopMotor();
      return false;
    }
  }

  stopMotor();
  delay(500);

  if (peakCount < REQUIRED_PEAKS) {
    Serial.println(F("\n✗ FAILED - Insufficient peaks"));
    Serial.print(F("Got "));
    Serial.print(peakCount);
    Serial.print(F(" / "));
    Serial.println(REQUIRED_PEAKS);
    return false;
  }

  Serial.println(F("\n✓ Oscillation test complete!\n"));

  // Analysis (skip first few peaks for settling)
  const int SKIP = 4;

  long minPos = peakPositions[SKIP];
  long maxPos = peakPositions[SKIP];

  for (int i = SKIP; i < peakCount; i++) {
    if (peakPositions[i] < minPos) minPos = peakPositions[i];
    if (peakPositions[i] > maxPos) maxPos = peakPositions[i];
  }

  float amplitude = abs(maxPos - minPos) / 2.0;

  // Calculate average period
  float totalPeriod = 0;
  int periodCount = 0;

  for (int i = SKIP + 2; i < peakCount; i += 2) {
    float period = (peakTimes[i] - peakTimes[i-2]) / 1000.0;
    totalPeriod += period;
    periodCount++;
  }

  float avgPeriod = totalPeriod / periodCount;

  // Ziegler-Nichols calculations
  float Ku = (4.0 * TEST_VOLTAGE) / (3.14159 * amplitude);

  lastTuneResults.Ku = Ku;
  lastTuneResults.Tu = avgPeriod;
  lastTuneResults.amplitude = amplitude;
  lastTuneResults.peakCount = peakCount;
  lastTuneResults.valid = true;

  Serial.println(F("╔════════════════════════════════════════════╗"));
  Serial.println(F("║   OSCILLATION ANALYSIS RESULTS             ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.print(F("  Oscillation Amplitude: "));
  Serial.print(amplitude, 1);
  Serial.println(F(" counts"));
  Serial.print(F("  Average Period (Tu):   "));
  Serial.print(avgPeriod, 3);
  Serial.println(F(" seconds"));
  Serial.print(F("  Ultimate Gain (Ku):    "));
  Serial.println(Ku, 4);
  Serial.println();

  // Calculate PID gains using different methods
  Serial.println(F("╔════════════════════════════════════════════╗"));
  Serial.println(F("║   RECOMMENDED PID GAINS                    ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  // 1. Classic Ziegler-Nichols
  float ZN_Kp = 0.6 * Ku;
  float ZN_Ki = 1.2 * Ku / avgPeriod;
  float ZN_Kd = 0.075 * Ku * avgPeriod;

  Serial.println(F("1. Classic Ziegler-Nichols:"));
  Serial.print(F("   Kp = "));
  Serial.println(ZN_Kp, 4);
  Serial.print(F("   Ki = "));
  Serial.println(ZN_Ki, 4);
  Serial.print(F("   Kd = "));
  Serial.println(ZN_Kd, 4);
  Serial.println();

  // 2. Conservative (30% safety margin)
  float CONS_Kp = 0.6 * Ku * 0.3;
  float CONS_Ki = 1.2 * Ku / avgPeriod * 0.3;
  float CONS_Kd = 0.075 * Ku * avgPeriod * 0.3;

  Serial.println(F("2. Conservative (30% of ZN - Recommended):"));
  Serial.print(F("   Kp = "));
  Serial.println(CONS_Kp, 4);
  Serial.print(F("   Ki = "));
  Serial.println(CONS_Ki, 4);
  Serial.print(F("   Kd = "));
  Serial.println(CONS_Kd, 4);
  Serial.println();

  // 3. Aggressive (80% of ZN)
  float AGG_Kp = 0.6 * Ku * 0.8;
  float AGG_Ki = 1.2 * Ku / avgPeriod * 0.8;
  float AGG_Kd = 0.075 * Ku * avgPeriod * 0.8;

  Serial.println(F("3. Aggressive (80% of ZN):"));
  Serial.print(F("   Kp = "));
  Serial.println(AGG_Kp, 4);
  Serial.print(F("   Ki = "));
  Serial.println(AGG_Ki, 4);
  Serial.print(F("   Kd = "));
  Serial.println(AGG_Kd, 4);
  Serial.println();

  // 4. PD-only (no integral - good starting point)
  float PD_Kp = 0.8 * Ku * 0.5;
  float PD_Ki = 0;
  float PD_Kd = Ku * avgPeriod / 8.0 * 0.5;

  Serial.println(F("4. PD-Only (No Integral Windup):"));
  Serial.print(F("   Kp = "));
  Serial.println(PD_Kp, 4);
  Serial.print(F("   Ki = "));
  Serial.println(PD_Ki, 4);
  Serial.print(F("   Kd = "));
  Serial.println(PD_Kd, 4);
  Serial.println();

  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
  Serial.println(F("Select tuning method:"));
  Serial.println(F("  1 - Classic Ziegler-Nichols"));
  Serial.println(F("  2 - Conservative (Recommended)"));
  Serial.println(F("  3 - Aggressive"));
  Serial.println(F("  4 - PD-Only"));
  Serial.println(F("  N - Do not apply\n"));

  while (!Serial.available()) {}
  response = Serial.read();
  while (Serial.available()) Serial.read();

  float newKp, newKi, newKd;
  bool applyGains = false;

  switch (response) {
    case '1':
      newKp = ZN_Kp;
      newKi = ZN_Ki;
      newKd = ZN_Kd;
      applyGains = true;
      Serial.println(F("\n✓ Applying Classic Ziegler-Nichols gains"));
      break;

    case '2':
      newKp = CONS_Kp;
      newKi = CONS_Ki;
      newKd = CONS_Kd;
      applyGains = true;
      Serial.println(F("\n✓ Applying Conservative gains (Recommended)"));
      break;

    case '3':
      newKp = AGG_Kp;
      newKi = AGG_Ki;
      newKd = AGG_Kd;
      applyGains = true;
      Serial.println(F("\n✓ Applying Aggressive gains"));
      break;

    case '4':
      newKp = PD_Kp;
      newKi = PD_Ki;
      newKd = PD_Kd;
      applyGains = true;
      Serial.println(F("\n✓ Applying PD-Only gains"));
      break;

    default:
      Serial.println(F("\n✗ Gains not applied"));
      break;
  }

  if (applyGains) {
    KP = newKp;
    KI = newKi;
    KD = newKd;

    Serial.println();
    Serial.print(F("New gains: Kp = "));
    Serial.print(KP, 4);
    Serial.print(F(", Ki = "));
    Serial.print(KI, 4);
    Serial.print(F(", Kd = "));
    Serial.println(KD, 4);
    Serial.println();

    saveCalibration();

    Serial.println(F("Ready to test! Try moving to a position."));
    homeToLeftLimit();
  }

  return true;
}

// ============================================
// STEP RESPONSE ANALYSIS
// ============================================
bool stepResponseTest() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   STEP RESPONSE ANALYSIS                   ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  if (!limitsCalibrated) {
    Serial.println(F("⚠ Please calibrate range first (command 'R')"));
    return false;
  }

  Serial.println(F("This test applies a step voltage and analyzes response."));
  Serial.println(F("Useful for identifying system dynamics.\n"));

  systemEnabled = false;
  stopMotor();

  if (!homeToLeftLimit()) {
    return false;
  }

  delay(1000);

  // Apply step input
  const float STEP_VOLTAGE = 6.0;
  const unsigned long TEST_DURATION = 5000;

  Serial.print(F("Applying "));
  Serial.print(STEP_VOLTAGE, 1);
  Serial.print(F("V step for "));
  Serial.print(TEST_DURATION / 1000);
  Serial.println(F(" seconds...\n"));

  long startPos = encoder.read();
  unsigned long startTime = millis();

  // Data collection arrays
  const int MAX_SAMPLES = 500;
  long positions[MAX_SAMPLES];
  unsigned long times[MAX_SAMPLES];
  int sampleCount = 0;

  setMotor(-STEP_VOLTAGE);  // Negative = move away from 0

  while ((millis() - startTime) < TEST_DURATION && sampleCount < MAX_SAMPLES) {
    positions[sampleCount] = encoder.read();
    times[sampleCount] = millis() - startTime;
    sampleCount++;
    delay(10);
  }

  stopMotor();

  long endPos = encoder.read();
  long totalTravel = abs(endPos - startPos);

  Serial.println(F("\n=== STEP RESPONSE RESULTS ==="));
  Serial.print(F("Total travel: "));
  Serial.print(totalTravel);
  Serial.println(F(" counts"));
  Serial.print(F("Test duration: "));
  Serial.print((millis() - startTime) / 1000.0, 2);
  Serial.println(F(" seconds"));

  // Analyze response
  // Find when we reached 63% of final value (time constant estimate)
  long target63 = startPos + (endPos - startPos) * 0.63;
  unsigned long timeConstant = 0;

  for (int i = 0; i < sampleCount; i++) {
    if (abs(positions[i]) >= abs(target63)) {
      timeConstant = times[i];
      break;
    }
  }

  Serial.print(F("Estimated time constant: "));
  Serial.print(timeConstant / 1000.0, 3);
  Serial.println(F(" seconds"));

  // Estimate system gain (position per volt)
  float systemGain = totalTravel / STEP_VOLTAGE;
  Serial.print(F("System gain: "));
  Serial.print(systemGain, 2);
  Serial.println(F(" counts/volt\n"));

  Serial.println(F("Position vs Time data (first 20 samples):"));
  Serial.println(F("Time(ms)\tPosition"));
  for (int i = 0; i < min(20, sampleCount); i++) {
    Serial.print(times[i]);
    Serial.print(F("\t\t"));
    Serial.println(positions[i]);
  }

  homeToLeftLimit();
  return true;
}

// ============================================
// EEPROM STORAGE
// ============================================
void saveCalibration() {
  EEPROM.put(EEPROM_KP_ADDR, KP);
  EEPROM.put(EEPROM_KI_ADDR, KI);
  EEPROM.put(EEPROM_KD_ADDR, KD);
  EEPROM.put(EEPROM_FRICTION_LEFT_ADDR, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT_ADDR, FRICTION_RIGHT);
  EEPROM.put(EEPROM_LEFT_LIMIT_ADDR, LEFT_LIMIT);
  EEPROM.put(EEPROM_RIGHT_LIMIT_ADDR, RIGHT_LIMIT);
  EEPROM.write(EEPROM_CALIBRATED_FLAG, 1);

  Serial.println(F("✓ Calibration saved to EEPROM"));
}

void loadCalibration() {
  byte calibrated = EEPROM.read(EEPROM_CALIBRATED_FLAG);

  if (calibrated == 1) {
    EEPROM.get(EEPROM_KP_ADDR, KP);
    EEPROM.get(EEPROM_KI_ADDR, KI);
    EEPROM.get(EEPROM_KD_ADDR, KD);
    EEPROM.get(EEPROM_FRICTION_LEFT_ADDR, FRICTION_LEFT);
    EEPROM.get(EEPROM_FRICTION_RIGHT_ADDR, FRICTION_RIGHT);
    EEPROM.get(EEPROM_LEFT_LIMIT_ADDR, LEFT_LIMIT);
    EEPROM.get(EEPROM_RIGHT_LIMIT_ADDR, RIGHT_LIMIT);
    limitsCalibrated = true;

    Serial.println(F("\n✓ Loaded calibration from EEPROM:"));
    Serial.print(F("  PID: Kp="));
    Serial.print(KP, 4);
    Serial.print(F(", Ki="));
    Serial.print(KI, 4);
    Serial.print(F(", Kd="));
    Serial.println(KD, 4);
    Serial.print(F("  Friction: Left="));
    Serial.print(FRICTION_LEFT, 2);
    Serial.print(F("V, Right="));
    Serial.print(FRICTION_RIGHT, 2);
    Serial.println(F("V"));
    Serial.print(F("  Range: "));
    Serial.print(LEFT_LIMIT);
    Serial.print(F(" to "));
    Serial.println(RIGHT_LIMIT);
    Serial.println();
  } else {
    Serial.println(F("\n⚠ No calibration found in EEPROM"));
    Serial.println(F("  Using default values\n"));
  }
}

void clearCalibration() {
  EEPROM.write(EEPROM_CALIBRATED_FLAG, 0);
  Serial.println(F("✓ Calibration cleared from EEPROM"));
}

// ============================================
// MANUAL POSITION TEST
// ============================================
void manualPositionTest() {
  Serial.println(F("\n=== MANUAL POSITION TEST ==="));
  Serial.println(F("This will test the current PID gains."));
  Serial.print(F("Enter target position ("));
  Serial.print(RIGHT_LIMIT);
  Serial.print(F(" to "));
  Serial.print(LEFT_LIMIT);
  Serial.println(F("):"));

  while (!Serial.available()) {}
  long target = Serial.parseInt();
  while (Serial.available()) Serial.read();

  if (target < RIGHT_LIMIT || target > LEFT_LIMIT) {
    Serial.println(F("✗ Target out of range"));
    return;
  }

  Serial.print(F("\nMoving to "));
  Serial.print(target);
  Serial.println(F("..."));

  targetPosition = target;
  errorIntegral = 0;
  lastError = 0;
  systemEnabled = true;

  long startPos = encoder.read();
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  Serial.println(F("\nTime(s)\tPosition\tError\tIntegral"));

  while (abs(encoder.read() - targetPosition) > DEADBAND || (millis() - startTime) < 3000) {
    runPIDControl();

    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      long pos = encoder.read();
      long err = targetPosition - pos;

      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(F("\t"));
      Serial.print(pos);
      Serial.print(F("\t\t"));
      Serial.print(err);
      Serial.print(F("\t"));
      Serial.println(errorIntegral, 2);
    }

    delay(10);

    if (millis() - startTime > 15000) {
      Serial.println(F("\n⚠ Timeout"));
      break;
    }
  }

  stopMotor();
  systemEnabled = false;

  long finalPos = encoder.read();
  long finalError = targetPosition - finalPos;
  unsigned long settleTime = millis() - startTime;

  Serial.println(F("\n=== TEST RESULTS ==="));
  Serial.print(F("Start position:   "));
  Serial.println(startPos);
  Serial.print(F("Target position:  "));
  Serial.println(targetPosition);
  Serial.print(F("Final position:   "));
  Serial.println(finalPos);
  Serial.print(F("Final error:      "));
  Serial.println(finalError);
  Serial.print(F("Settling time:    "));
  Serial.print(settleTime / 1000.0, 2);
  Serial.println(F(" seconds"));

  if (abs(finalError) <= DEADBAND) {
    Serial.println(F("✓ Target reached successfully\n"));
  } else {
    Serial.println(F("✗ Target not reached\n"));
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
    case 'R':
      calibrateRange();
      break;

    case 'F':
      characterizeFriction();
      break;

    case 'Z':
      autoTuneZN();
      break;

    case 'S':
      stepResponseTest();
      break;

    case 'T':
      manualPositionTest();
      break;

    case 'H':
      homeToLeftLimit();
      break;

    case 'P':
      printStatus();
      break;

    case 'C':
      clearCalibration();
      break;

    case '?':
      printHelp();
      break;

    default:
      break;
  }
}

// ============================================
// DISPLAY FUNCTIONS
// ============================================
void printWelcome() {
  Serial.println(F("\n\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   ME 350 - ENHANCED PID AUTO-TUNING        ║"));
  Serial.println(F("║   Comprehensive System Identification      ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("COMMANDS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("  R - Calibrate Range (find left/right limits)"));
  Serial.println(F("  F - Characterize Friction (measure breakaway)"));
  Serial.println(F("  Z - Ziegler-Nichols Auto-Tune (oscillation)"));
  Serial.println(F("  S - Step Response Test (system dynamics)"));
  Serial.println(F("  T - Manual Position Test (test current gains)"));
  Serial.println(F("  H - Home to left limit"));
  Serial.println(F("  P - Print current status"));
  Serial.println(F("  C - Clear EEPROM calibration"));
  Serial.println(F("  ? - Show this help"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));

  Serial.println(F("RECOMMENDED WORKFLOW:"));
  Serial.println(F("  1. R - Calibrate range"));
  Serial.println(F("  2. F - Characterize friction"));
  Serial.println(F("  3. Z - Auto-tune PID (Ziegler-Nichols)"));
  Serial.println(F("  4. T - Test with manual moves"));
  Serial.println(F("  5. S - (Optional) Analyze step response\n"));
}

void printStatus() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   SYSTEM STATUS                            ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  Serial.println(F("━━ CURRENT CONFIGURATION ━━"));
  Serial.print(F("PID Gains:  Kp = "));
  Serial.print(KP, 4);
  Serial.print(F(", Ki = "));
  Serial.print(KI, 4);
  Serial.print(F(", Kd = "));
  Serial.println(KD, 4);

  Serial.print(F("Friction:   Left = "));
  Serial.print(FRICTION_LEFT, 2);
  Serial.print(F("V, Right = "));
  Serial.print(FRICTION_RIGHT, 2);
  Serial.println(F("V"));

  Serial.println(F("\n━━ SYSTEM LIMITS ━━"));
  Serial.print(F("Left Limit:   "));
  Serial.println(LEFT_LIMIT);
  Serial.print(F("Right Limit:  "));
  Serial.println(RIGHT_LIMIT);
  Serial.print(F("Total Range:  "));
  Serial.print(abs(RIGHT_LIMIT - LEFT_LIMIT));
  Serial.println(F(" counts"));
  Serial.print(F("Calibrated:   "));
  Serial.println(limitsCalibrated ? F("YES") : F("NO"));

  Serial.println(F("\n━━ CURRENT STATE ━━"));
  Serial.print(F("Position:     "));
  Serial.println(encoder.read());
  Serial.print(F("Target:       "));
  Serial.println(targetPosition);
  Serial.print(F("System:       "));
  Serial.println(systemEnabled ? F("ENABLED") : F("DISABLED"));

  Serial.print(F("Limit Left:   "));
  Serial.println(leftPressed() ? F("PRESSED") : F("Open"));
  Serial.print(F("Limit Right:  "));
  Serial.println(rightPressed() ? F("PRESSED") : F("Open"));

  if (lastTuneResults.valid) {
    Serial.println(F("\n━━ LAST AUTO-TUNE RESULTS ━━"));
    Serial.print(F("Ultimate Gain (Ku):  "));
    Serial.println(lastTuneResults.Ku, 4);
    Serial.print(F("Ultimate Period (Tu): "));
    Serial.print(lastTuneResults.Tu, 3);
    Serial.println(F(" s"));
    Serial.print(F("Amplitude:           "));
    Serial.print(lastTuneResults.amplitude, 1);
    Serial.println(F(" counts"));
    Serial.print(F("Peaks Collected:     "));
    Serial.println(lastTuneResults.peakCount);
  }

  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}
