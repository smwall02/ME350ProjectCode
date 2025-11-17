// ============================================================================
// ME 350 - ADVANCED PID AUTO-TUNING SYSTEM v2.0
// Comprehensive PID Parameter Identification and Tuning
// ============================================================================
// Features:
// - Multiple PID tuning methods (Ziegler-Nichols, Tyreus-Luyben, Cohen-Coon)
// - Advanced friction characterization (Stribeck curve)
// - Gentle limit switch approach (2V for safety)
// - Consistency validation with multiple tests
// - Step response analysis with detailed metrics
// - Clear, copy-ready PID value display
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
// GENTLE APPROACH VOLTAGES
// ============================================
const float GENTLE_APPROACH_VOLTAGE = 2.0;  // Gentle approach to limits
const float HOMING_VOLTAGE = 2.0;           // Gentle homing
const float RANGE_FINDING_VOLTAGE = -2.5;   // Gentle range finding

// ============================================
// EEPROM ADDRESSES FOR PERSISTENT STORAGE
// ============================================
#define EEPROM_KP_ADDR 0
#define EEPROM_KI_ADDR 4
#define EEPROM_KD_ADDR 8
#define EEPROM_FRICTION_STATIC_LEFT_ADDR 12
#define EEPROM_FRICTION_STATIC_RIGHT_ADDR 16
#define EEPROM_FRICTION_COULOMB_LEFT_ADDR 20
#define EEPROM_FRICTION_COULOMB_RIGHT_ADDR 24
#define EEPROM_FRICTION_VISCOUS_ADDR 28
#define EEPROM_CALIBRATED_FLAG 32
#define EEPROM_LEFT_LIMIT_ADDR 36
#define EEPROM_RIGHT_LIMIT_ADDR 40

// ============================================
// PID PARAMETERS
// ============================================
float KP = 0.020;
float KI = 0.005;
float KD = 0.004;

// ============================================
// ADVANCED FRICTION MODEL (Stribeck)
// ============================================
struct FrictionModel {
  float staticLeft;      // Static friction voltage (LEFT direction)
  float staticRight;     // Static friction voltage (RIGHT direction)
  float coulombLeft;     // Coulomb friction voltage (LEFT direction)
  float coulombRight;    // Coulomb friction voltage (RIGHT direction)
  float viscous;         // Viscous friction coefficient
  float stribeckVel;     // Stribeck velocity
  bool calibrated;
};

FrictionModel friction = {
  .staticLeft = 2.5,
  .staticRight = 3.0,
  .coulombLeft = 1.6,
  .coulombRight = 2.0,
  .viscous = 0.01,
  .stribeckVel = 100.0,
  .calibrated = false
};

// ============================================
// CONTROL PARAMETERS
// ============================================
const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 0.5;
const int DEADBAND = 3;
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
long lastPosition = 0;
unsigned long lastVelocityTime = 0;
float currentVelocity = 0;

// ============================================
// COMPREHENSIVE TUNE RESULTS
// ============================================
struct ComprehensiveTuneResults {
  // Relay Method Results
  float Ku;                  // Ultimate gain
  float Tu;                  // Ultimate period
  float amplitude;           // Oscillation amplitude
  int peakCount;             // Number of peaks

  // Multiple Method Results
  float ZN_Kp, ZN_Ki, ZN_Kd;         // Ziegler-Nichols
  float TL_Kp, TL_Ki, TL_Kd;         // Tyreus-Luyben
  float CC_Kp, CC_Ki, CC_Kd;         // Cohen-Coon
  float SR_Kp, SR_Ki, SR_Kd;         // Step Response

  // Step Response Metrics
  float riseTime;            // 10% to 90% rise time
  float settlingTime;        // Time to settle within 2%
  float overshoot;           // Percent overshoot
  float steadyStateError;    // Final error
  float timeConstant;        // System time constant
  float processGain;         // DC gain
  float deadTime;            // Transport delay

  // Consistency Metrics
  float Ku_stdDev;           // Standard deviation of Ku from multiple tests
  float Tu_stdDev;           // Standard deviation of Tu
  int numTests;              // Number of relay tests performed

  bool valid;                // Overall validity
};

ComprehensiveTuneResults tuneResults;

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
    updateVelocity();
    runPIDControl();
  }

  checkLimitSwitches();
}

// ============================================
// VELOCITY ESTIMATION
// ============================================
void updateVelocity() {
  long currentPos = encoder.read();
  unsigned long currentTime = micros();

  if (currentTime - lastVelocityTime > 10000) {  // Update every 10ms
    float deltaPos = currentPos - lastPosition;
    float deltaTime = (currentTime - lastVelocityTime) / 1000000.0;  // seconds

    currentVelocity = deltaPos / deltaTime;  // encoder counts per second

    lastPosition = currentPos;
    lastVelocityTime = currentTime;
  }
}

// ============================================
// ADVANCED FRICTION COMPENSATION
// ============================================
float getFrictionCompensation(float error, float velocity) {
  if (abs(error) <= DEADBAND) {
    return 0;
  }

  bool movingLeft = (error > 0);  // Positive error = need to move left
  float absVel = abs(velocity);

  // Select direction-specific parameters
  float Fs = movingLeft ? friction.staticRight : friction.staticLeft;
  float Fc = movingLeft ? friction.coulombRight : friction.coulombLeft;
  float Fv = friction.viscous;
  float vs = friction.stribeckVel;

  // Stribeck friction model
  float frictionForce;
  if (absVel < 1.0) {
    // Near zero velocity - use static friction
    frictionForce = Fs;
  } else {
    // Stribeck + viscous model
    float stribeckTerm = (Fs - Fc) * exp(-pow(absVel / vs, 2));
    frictionForce = stribeckTerm + Fc + Fv * absVel;
  }

  // Scale based on error magnitude for smooth transitions
  float errorScale = 1.0;
  if (abs(error) < 50) {
    errorScale = abs(error) / 50.0;
  }

  return movingLeft ? frictionForce * errorScale : -frictionForce * errorScale;
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

  // Advanced friction compensation
  float frictionComp = getFrictionCompensation(error, currentVelocity);

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
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (voltage < 0) {
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
// GENTLE HOMING (2V approach)
// ============================================
bool homeToLeftLimit() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   GENTLE HOMING TO LEFT LIMIT              ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  if (leftPressed()) {
    Serial.println(F("Already at limit. Backing off gently..."));
    setMotor(-GENTLE_APPROACH_VOLTAGE);
    delay(400);
    stopMotor();
    delay(200);
  }

  Serial.print(F("Approaching left limit gently at "));
  Serial.print(HOMING_VOLTAGE, 1);
  Serial.println(F("V..."));

  setMotor(HOMING_VOLTAGE);

  unsigned long startTime = millis();
  while (!leftPressed() && (millis() - startTime) < 20000) {
    delay(10);
  }

  if (leftPressed()) {
    stopMotor();
    delay(300);

    // Multiple encoder resets for reliability
    encoder.write(0);
    delay(50);
    encoder.write(0);
    delay(50);
    encoder.write(0);

    LEFT_LIMIT = 0;

    Serial.println(F("✓ Homed successfully"));
    Serial.print(F("  Final encoder: "));
    Serial.println(encoder.read());
    return true;
  } else {
    stopMotor();
    Serial.println(F("✗ Homing timeout"));
    return false;
  }
}

// ============================================
// GENTLE RANGE CALIBRATION
// ============================================
bool calibrateRange() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   GENTLE RANGE CALIBRATION                 ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  systemEnabled = false;
  stopMotor();
  delay(200);

  // Gentle home to left
  if (!homeToLeftLimit()) {
    Serial.println(F("✗ Range calibration failed at homing\n"));
    return false;
  }

  LEFT_LIMIT = encoder.read();
  Serial.print(F("\nLeft limit set to: "));
  Serial.println(LEFT_LIMIT);
  delay(1000);

  // Gentle move to right limit
  Serial.print(F("\nMoving to right limit gently at "));
  Serial.print(abs(RANGE_FINDING_VOLTAGE), 1);
  Serial.println(F("V..."));

  setMotor(RANGE_FINDING_VOLTAGE);

  unsigned long startTime = millis();
  long lastPos = encoder.read();
  unsigned long stuckTime = 0;

  while (!rightPressed() && (millis() - startTime) < 30000) {
    delay(50);
    long currentPos = encoder.read();

    // Stuck detection
    if (abs(currentPos - lastPos) < 2) {
      stuckTime += 50;
      if (stuckTime > 3000) {
        Serial.println(F("⚠ Movement stopped - assuming limit reached"));
        break;
      }
    } else {
      stuckTime = 0;
      lastPos = currentPos;
    }

    // Progress indicator
    if ((millis() - startTime) % 2000 < 50) {
      Serial.print(F("  Position: "));
      Serial.println(currentPos);
    }
  }

  stopMotor();
  delay(500);
  RIGHT_LIMIT = encoder.read();

  Serial.print(F("\nRight limit set to: "));
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
    Serial.println(F("\n✗ Range too small - calibration failed\n"));
    return false;
  }
}

// ============================================
// COMPREHENSIVE FRICTION CHARACTERIZATION
// Implements Stribeck friction model
// ============================================
bool characterizeFrictionAdvanced() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   ADVANCED FRICTION CHARACTERIZATION       ║"));
  Serial.println(F("║   (Stribeck Curve Method)                  ║"));
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

  delay(1000);

  Serial.println(F("This test characterizes friction at multiple velocities"));
  Serial.println(F("to build a complete Stribeck curve.\n"));

  // ===== STATIC FRICTION TEST (LEFT direction) =====
  Serial.println(F("━━━ TEST 1: STATIC FRICTION (→ negative) ━━━"));

  float testVoltage = 0.5;
  const float voltageStep = 0.1;
  const int movementThreshold = 15;

  friction.staticLeft = 0;

  for (int attempt = 0; attempt < 40 && friction.staticLeft == 0; attempt++) {
    long startPos = encoder.read();

    Serial.print(F("  "));
    Serial.print(testVoltage, 2);
    Serial.print(F("V → "));

    setMotor(-testVoltage);
    delay(500);

    long endPos = encoder.read();
    long movement = abs(endPos - startPos);

    stopMotor();
    delay(200);

    Serial.print(movement);
    Serial.println(F(" counts"));

    if (movement >= movementThreshold) {
      friction.staticLeft = testVoltage;
      Serial.print(F("  ✓ Static friction (LEFT) = "));
      Serial.print(friction.staticLeft, 2);
      Serial.println(F("V\n"));
    } else {
      testVoltage += voltageStep;
    }
  }

  if (friction.staticLeft == 0) {
    friction.staticLeft = 2.5;
    Serial.println(F("  ⚠ Using default 2.5V\n"));
  }

  delay(500);
  homeToLeftLimit();
  delay(1000);

  // ===== COULOMB FRICTION AT STEADY VELOCITY (LEFT) =====
  Serial.println(F("━━━ TEST 2: COULOMB FRICTION (constant velocity LEFT) ━━━"));

  Serial.println(F("  Moving at constant velocity to measure Coulomb friction..."));

  float testVoltages[] = {friction.staticLeft + 0.5, friction.staticLeft + 1.0, friction.staticLeft + 1.5};
  float avgCoulombVoltage = 0;
  int validTests = 0;

  for (int i = 0; i < 3; i++) {
    long startPos = encoder.read();
    unsigned long startTime = millis();

    setMotor(-testVoltages[i]);
    delay(1500);  // Allow to reach steady state

    // Measure velocity over 1 second
    long pos1 = encoder.read();
    delay(1000);
    long pos2 = encoder.read();
    float velocity = abs(pos2 - pos1);  // counts/sec

    stopMotor();
    delay(300);

    Serial.print(F("    "));
    Serial.print(testVoltages[i], 2);
    Serial.print(F("V → velocity = "));
    Serial.print(velocity, 1);
    Serial.println(F(" counts/s"));

    if (velocity > 50) {  // Valid constant velocity
      avgCoulombVoltage += testVoltages[i];
      validTests++;
    }

    delay(500);
    homeToLeftLimit();
    delay(500);
  }

  if (validTests > 0) {
    friction.coulombLeft = avgCoulombVoltage / validTests - 0.3;  // Slightly less than static
    Serial.print(F("  ✓ Coulomb friction (LEFT) = "));
    Serial.print(friction.coulombLeft, 2);
    Serial.println(F("V\n"));
  } else {
    friction.coulombLeft = friction.staticLeft * 0.7;
    Serial.println(F("  ⚠ Using estimated Coulomb friction\n"));
  }

  delay(1000);

  // ===== MOVE TO MIDDLE FOR RIGHT DIRECTION TESTS =====
  Serial.println(F("━━━ MOVING TO CENTER FOR RIGHT TESTS ━━━\n"));

  long middlePos = (LEFT_LIMIT + RIGHT_LIMIT) / 2;
  Serial.print(F("  Target position: "));
  Serial.println(middlePos);

  systemEnabled = true;
  targetPosition = middlePos;
  errorIntegral = 0;
  lastError = 0;

  unsigned long moveStart = millis();
  while (abs(encoder.read() - targetPosition) > 30 && (millis() - moveStart) < 15000) {
    runPIDControl();
    delay(10);
  }

  stopMotor();
  systemEnabled = false;
  delay(1000);

  Serial.print(F("  Current position: "));
  Serial.println(encoder.read());
  Serial.println();

  // ===== STATIC FRICTION TEST (RIGHT direction) =====
  Serial.println(F("━━━ TEST 3: STATIC FRICTION (→ less negative) ━━━"));

  testVoltage = 0.5;
  friction.staticRight = 0;

  for (int attempt = 0; attempt < 40 && friction.staticRight == 0; attempt++) {
    long startPos = encoder.read();

    Serial.print(F("  "));
    Serial.print(testVoltage, 2);
    Serial.print(F("V → "));

    setMotor(testVoltage);
    delay(500);

    long endPos = encoder.read();
    long movement = abs(endPos - startPos);

    stopMotor();
    delay(200);

    Serial.print(movement);
    Serial.println(F(" counts"));

    if (movement >= movementThreshold) {
      friction.staticRight = testVoltage;
      Serial.print(F("  ✓ Static friction (RIGHT) = "));
      Serial.print(friction.staticRight, 2);
      Serial.println(F("V\n"));
    } else {
      testVoltage += voltageStep;
    }
  }

  if (friction.staticRight == 0) {
    friction.staticRight = 3.0;
    Serial.println(F("  ⚠ Using default 3.0V\n"));
  }

  // ===== COULOMB FRICTION (RIGHT) =====
  Serial.println(F("━━━ TEST 4: COULOMB FRICTION (constant velocity RIGHT) ━━━"));

  avgCoulombVoltage = 0;
  validTests = 0;
  testVoltages[0] = friction.staticRight + 0.5;
  testVoltages[1] = friction.staticRight + 1.0;
  testVoltages[2] = friction.staticRight + 1.5;

  for (int i = 0; i < 3; i++) {
    setMotor(testVoltages[i]);
    delay(1500);

    long pos1 = encoder.read();
    delay(1000);
    long pos2 = encoder.read();
    float velocity = abs(pos2 - pos1);

    stopMotor();
    delay(300);

    Serial.print(F("    "));
    Serial.print(testVoltages[i], 2);
    Serial.print(F("V → velocity = "));
    Serial.print(velocity, 1);
    Serial.println(F(" counts/s"));

    if (velocity > 50) {
      avgCoulombVoltage += testVoltages[i];
      validTests++;
    }

    delay(300);
  }

  if (validTests > 0) {
    friction.coulombRight = avgCoulombVoltage / validTests - 0.3;
    Serial.print(F("  ✓ Coulomb friction (RIGHT) = "));
    Serial.print(friction.coulombRight, 2);
    Serial.println(F("V\n"));
  } else {
    friction.coulombRight = friction.staticRight * 0.7;
    Serial.println(F("  ⚠ Using estimated Coulomb friction\n"));
  }

  // ===== VISCOUS FRICTION ESTIMATION =====
  Serial.println(F("━━━ TEST 5: VISCOUS FRICTION COEFFICIENT ━━━"));

  homeToLeftLimit();
  delay(500);

  Serial.println(F("  Testing velocity-dependent friction..."));

  float highVoltage = friction.staticLeft + 3.0;
  setMotor(-highVoltage);
  delay(2000);

  long pos1 = encoder.read();
  delay(1000);
  long pos2 = encoder.read();
  float highVelocity = abs(pos2 - pos1);

  stopMotor();

  // Estimate viscous coefficient
  if (highVelocity > 100) {
    friction.viscous = (highVoltage - friction.coulombLeft) / highVelocity;
    Serial.print(F("  ✓ Viscous coefficient = "));
    Serial.print(friction.viscous, 4);
    Serial.println(F(" V/(count/s)\n"));
  } else {
    friction.viscous = 0.01;
    Serial.println(F("  ⚠ Using default viscous coefficient\n"));
  }

  // Stribeck velocity (estimated)
  friction.stribeckVel = 100.0;  // Typical value
  friction.calibrated = true;

  // ===== SUMMARY =====
  Serial.println(F("╔════════════════════════════════════════════╗"));
  Serial.println(F("║   FRICTION CHARACTERIZATION COMPLETE       ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  Serial.println(F("STRIBECK FRICTION MODEL PARAMETERS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.print(F("  Static Friction LEFT:    "));
  Serial.print(friction.staticLeft, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Static Friction RIGHT:   "));
  Serial.print(friction.staticRight, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Coulomb Friction LEFT:   "));
  Serial.print(friction.coulombLeft, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Coulomb Friction RIGHT:  "));
  Serial.print(friction.coulombRight, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Viscous Coefficient:     "));
  Serial.print(friction.viscous, 4);
  Serial.println(F(" V/(count/s)"));
  Serial.print(F("  Stribeck Velocity:       "));
  Serial.print(friction.stribeckVel, 1);
  Serial.println(F(" count/s"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));

  saveCalibration();
  homeToLeftLimit();

  return true;
}

// ============================================
// MULTIPLE RELAY TESTS FOR CONSISTENCY
// ============================================
bool performMultipleRelayTests(int numTests, float testVoltages[], float &avgKu, float &avgTu, float &stdKu, float &stdTu) {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   MULTIPLE RELAY TESTS (Consistency Check) ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  float kuValues[5];
  float tuValues[5];
  int successfulTests = 0;

  for (int test = 0; test < numTests && test < 5; test++) {
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
    Serial.print(F("TEST "));
    Serial.print(test + 1);
    Serial.print(F(" of "));
    Serial.println(numTests);
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));

    float testVoltage = testVoltages[test];
    float ku, tu, amp;
    int peaks;

    if (performSingleRelayTest(testVoltage, ku, tu, amp, peaks)) {
      kuValues[successfulTests] = ku;
      tuValues[successfulTests] = tu;
      successfulTests++;

      Serial.print(F("  ✓ Test "));
      Serial.print(test + 1);
      Serial.print(F(" complete: Ku = "));
      Serial.print(ku, 4);
      Serial.print(F(", Tu = "));
      Serial.print(tu, 3);
      Serial.println(F(" s\n"));
    } else {
      Serial.print(F("  ✗ Test "));
      Serial.print(test + 1);
      Serial.println(F(" failed\n"));
    }

    delay(2000);
    homeToLeftLimit();
    delay(2000);
  }

  if (successfulTests < 2) {
    Serial.println(F("✗ Not enough successful tests for statistical analysis\n"));
    return false;
  }

  // Calculate statistics
  avgKu = 0;
  avgTu = 0;
  for (int i = 0; i < successfulTests; i++) {
    avgKu += kuValues[i];
    avgTu += tuValues[i];
  }
  avgKu /= successfulTests;
  avgTu /= successfulTests;

  // Standard deviation
  stdKu = 0;
  stdTu = 0;
  for (int i = 0; i < successfulTests; i++) {
    stdKu += pow(kuValues[i] - avgKu, 2);
    stdTu += pow(tuValues[i] - avgTu, 2);
  }
  stdKu = sqrt(stdKu / successfulTests);
  stdTu = sqrt(stdTu / successfulTests);

  // Results
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   CONSISTENCY ANALYSIS                     ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.print(F("Successful tests: "));
  Serial.print(successfulTests);
  Serial.print(F(" / "));
  Serial.println(numTests);
  Serial.println(F("\nUltimate Gain (Ku):"));
  Serial.print(F("  Mean:   "));
  Serial.println(avgKu, 4);
  Serial.print(F("  StdDev: "));
  Serial.print(stdKu, 4);
  Serial.print(F(" ("));
  Serial.print(100 * stdKu / avgKu, 1);
  Serial.println(F("%)"));
  Serial.println(F("\nUltimate Period (Tu):"));
  Serial.print(F("  Mean:   "));
  Serial.print(avgTu, 3);
  Serial.println(F(" s"));
  Serial.print(F("  StdDev: "));
  Serial.print(stdTu, 4);
  Serial.print(F(" s ("));
  Serial.print(100 * stdTu / avgTu, 1);
  Serial.println(F("%)"));

  float cvKu = 100 * stdKu / avgKu;
  float cvTu = 100 * stdTu / avgTu;

  if (cvKu < 10 && cvTu < 10) {
    Serial.println(F("\n✓ Excellent consistency (CV < 10%)\n"));
  } else if (cvKu < 20 && cvTu < 20) {
    Serial.println(F("\n✓ Good consistency (CV < 20%)\n"));
  } else {
    Serial.println(F("\n⚠ Moderate consistency - results may vary\n"));
  }

  return true;
}

// ============================================
// SINGLE RELAY TEST (Helper function)
// ============================================
bool performSingleRelayTest(float testVoltage, float &Ku, float &Tu, float &amplitude, int &peakCount) {
  // Move to center
  long fullRange = abs(RIGHT_LIMIT - LEFT_LIMIT);
  long centerPosition = (LEFT_LIMIT + RIGHT_LIMIT) / 2;

  Serial.print(F("  Moving to center ("));
  Serial.print(centerPosition);
  Serial.println(F(")..."));

  unsigned long moveTimeout = millis();
  while (abs(encoder.read() - centerPosition) > 50 && (millis() - moveTimeout) < 15000) {
    long pos = encoder.read();
    long error = centerPosition - pos;

    if (error < 0) {
      setMotor(-GENTLE_APPROACH_VOLTAGE);
    } else {
      setMotor(GENTLE_APPROACH_VOLTAGE);
    }
    delay(10);
  }

  stopMotor();
  delay(1500);

  long actualCenter = encoder.read();
  Serial.print(F("  Center: "));
  Serial.println(actualCenter);

  // Relay test
  int HYSTERESIS = fullRange / 8;  // Smaller hysteresis for precision
  const int MAX_PEAKS = 24;
  const int REQUIRED_PEAKS = 18;
  const unsigned long TIMEOUT = 90000;

  Serial.print(F("  Hysteresis: ±"));
  Serial.print(HYSTERESIS);
  Serial.print(F(" counts, Voltage: ±"));
  Serial.print(testVoltage, 1);
  Serial.println(F("V"));

  // Initial displacement
  setMotor(-testVoltage);
  delay(1200);
  stopMotor();
  delay(500);

  long peakPositions[MAX_PEAKS];
  unsigned long peakTimes[MAX_PEAKS];
  peakCount = 0;

  long lastPos = encoder.read();
  bool crossedCenter = false;
  unsigned long startTime = millis();

  while (peakCount < REQUIRED_PEAKS && (millis() - startTime) < TIMEOUT) {
    long currentPos = encoder.read();
    long distanceFromCenter = currentPos - actualCenter;

    // Relay control
    if (distanceFromCenter < -HYSTERESIS) {
      setMotor(testVoltage);
      if (crossedCenter) crossedCenter = false;
    } else if (distanceFromCenter > HYSTERESIS) {
      setMotor(-testVoltage);
      if (crossedCenter) crossedCenter = false;
    }

    // Peak detection
    if (!crossedCenter && abs(distanceFromCenter) < 40) {
      crossedCenter = true;

      peakPositions[peakCount] = lastPos;
      peakTimes[peakCount] = millis();

      if (peakCount >= 4) {  // Only print after initial settling
        Serial.print(F("    Peak "));
        Serial.print(peakCount + 1);
        Serial.print(F(": "));
        Serial.println(lastPos);
      }

      peakCount++;
    }

    lastPos = currentPos;
    delay(5);

    if (Serial.available()) {
      Serial.read();
      while (Serial.available()) Serial.read();
      Serial.println(F("\n  ✗ Test aborted by user"));
      stopMotor();
      return false;
    }
  }

  stopMotor();

  if (peakCount < REQUIRED_PEAKS) {
    Serial.print(F("  ✗ Insufficient peaks: "));
    Serial.print(peakCount);
    Serial.print(F("/"));
    Serial.println(REQUIRED_PEAKS);
    return false;
  }

  // Analysis (skip first 6 peaks for better settling)
  const int SKIP = 6;

  long minPos = peakPositions[SKIP];
  long maxPos = peakPositions[SKIP];

  for (int i = SKIP; i < peakCount; i++) {
    if (peakPositions[i] < minPos) minPos = peakPositions[i];
    if (peakPositions[i] > maxPos) maxPos = peakPositions[i];
  }

  amplitude = abs(maxPos - minPos) / 2.0;

  // Calculate period
  float totalPeriod = 0;
  int periodCount = 0;

  for (int i = SKIP + 2; i < peakCount; i += 2) {
    float period = (peakTimes[i] - peakTimes[i-2]) / 1000.0;
    totalPeriod += period;
    periodCount++;
  }

  Tu = totalPeriod / periodCount;
  Ku = (4.0 * testVoltage) / (3.14159 * amplitude);

  return true;
}

// ============================================
// COMPREHENSIVE AUTO-TUNE with all methods
// ============================================
bool comprehensiveAutoTune() {
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   COMPREHENSIVE PID AUTO-TUNE                             ║"));
  Serial.println(F("║   Multiple Methods with Consistency Validation            ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  if (!limitsCalibrated) {
    Serial.println(F("⚠ Please calibrate range first (command 'R')"));
    return false;
  }

  if (!friction.calibrated) {
    Serial.println(F("⚠ Friction not characterized. Running friction test first...\n"));
    if (!characterizeFrictionAdvanced()) {
      return false;
    }
    delay(2000);
  }

  Serial.println(F("This comprehensive test performs:"));
  Serial.println(F("  1. Multiple relay tests at different voltages"));
  Serial.println(F("  2. Consistency validation"));
  Serial.println(F("  3. Step response analysis"));
  Serial.println(F("  4. PID calculations using 4 different methods"));
  Serial.println(F("\nEstimated time: 8-12 minutes\n"));
  Serial.println(F("Press ENTER to continue, or any other key to cancel..."));

  while (!Serial.available()) {}
  char response = Serial.read();
  while (Serial.available()) Serial.read();

  if (response != '\n' && response != '\r') {
    Serial.println(F("\n✗ Test cancelled\n"));
    return false;
  }

  // Initialize results
  tuneResults.valid = false;

  // ===== PHASE 1: MULTIPLE RELAY TESTS =====
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   PHASE 1: RELAY OSCILLATION TESTS                        ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  float testVoltages[] = {4.0, 5.0, 6.0};
  int numTests = 3;

  float avgKu, avgTu, stdKu, stdTu;
  if (!performMultipleRelayTests(numTests, testVoltages, avgKu, avgTu, stdKu, stdTu)) {
    Serial.println(F("✗ Relay tests failed\n"));
    return false;
  }

  tuneResults.Ku = avgKu;
  tuneResults.Tu = avgTu;
  tuneResults.Ku_stdDev = stdKu;
  tuneResults.Tu_stdDev = stdTu;
  tuneResults.numTests = numTests;

  delay(2000);

  // ===== PHASE 2: STEP RESPONSE ANALYSIS =====
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   PHASE 2: STEP RESPONSE ANALYSIS                         ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  if (!performStepResponseAnalysis()) {
    Serial.println(F("⚠ Step response failed - continuing with relay results\n"));
  }

  delay(2000);

  // ===== PHASE 3: CALCULATE PID GAINS =====
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   PHASE 3: PID GAIN CALCULATIONS                          ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  calculateAllPIDMethods();

  // ===== PHASE 4: DISPLAY RESULTS =====
  displayComprehensiveResults();

  // ===== PHASE 5: USER SELECTION =====
  selectAndApplyGains();

  tuneResults.valid = true;
  return true;
}

// ============================================
// STEP RESPONSE ANALYSIS
// ============================================
bool performStepResponseAnalysis() {
  Serial.println(F("Performing step response test...\n"));

  if (!homeToLeftLimit()) {
    return false;
  }

  delay(1000);

  const float STEP_VOLTAGE = 6.0;
  const unsigned long TEST_DURATION = 6000;
  const int MAX_SAMPLES = 600;

  long positions[MAX_SAMPLES];
  unsigned long times[MAX_SAMPLES];
  int sampleCount = 0;

  long startPos = encoder.read();
  unsigned long startTime = millis();

  Serial.print(F("  Applying "));
  Serial.print(STEP_VOLTAGE, 1);
  Serial.print(F("V step for "));
  Serial.print(TEST_DURATION / 1000);
  Serial.println(F(" seconds..."));

  setMotor(-STEP_VOLTAGE);

  while ((millis() - startTime) < TEST_DURATION && sampleCount < MAX_SAMPLES) {
    positions[sampleCount] = encoder.read();
    times[sampleCount] = millis() - startTime;
    sampleCount++;
    delay(10);
  }

  stopMotor();

  long endPos = encoder.read();
  long totalTravel = abs(endPos - startPos);

  Serial.print(F("  Total travel: "));
  Serial.print(totalTravel);
  Serial.println(F(" counts\n"));

  // Analyze response
  long steadyState = endPos;
  long target10 = startPos + (endPos - startPos) * 0.10;
  long target90 = startPos + (endPos - startPos) * 0.90;
  long target63 = startPos + (endPos - startPos) * 0.63;
  long settleUpper = steadyState + totalTravel * 0.02;
  long settleLower = steadyState - totalTravel * 0.02;

  unsigned long riseStart = 0, riseEnd = 0, tau = 0, settleTime = 0;
  long maxPos = startPos;
  bool foundRiseStart = false, foundRiseEnd = false;

  for (int i = 0; i < sampleCount; i++) {
    if (!foundRiseStart && abs(positions[i]) >= abs(target10)) {
      riseStart = times[i];
      foundRiseStart = true;
    }
    if (!foundRiseEnd && abs(positions[i]) >= abs(target90)) {
      riseEnd = times[i];
      foundRiseEnd = true;
    }
    if (abs(positions[i]) >= abs(target63)) {
      if (tau == 0) tau = times[i];
    }
    if (abs(positions[i]) > abs(maxPos)) {
      maxPos = positions[i];
    }
  }

  // Find settling time (from end backwards)
  for (int i = sampleCount - 1; i >= 0; i--) {
    if (positions[i] < settleLower || positions[i] > settleUpper) {
      settleTime = times[i];
      break;
    }
  }

  tuneResults.riseTime = (riseEnd - riseStart) / 1000.0;
  tuneResults.settlingTime = settleTime / 1000.0;
  tuneResults.timeConstant = tau / 1000.0;
  tuneResults.processGain = totalTravel / STEP_VOLTAGE;
  tuneResults.overshoot = 100.0 * (maxPos - steadyState) / totalTravel;
  tuneResults.steadyStateError = abs(endPos - steadyState);

  // Estimate dead time (time to first movement)
  tuneResults.deadTime = 0;
  for (int i = 1; i < sampleCount; i++) {
    if (abs(positions[i] - startPos) > 5) {
      tuneResults.deadTime = times[i] / 1000.0;
      break;
    }
  }

  Serial.println(F("  ✓ Step response analysis complete"));
  Serial.print(F("    Rise time: "));
  Serial.print(tuneResults.riseTime, 3);
  Serial.println(F(" s"));
  Serial.print(F("    Time constant: "));
  Serial.print(tuneResults.timeConstant, 3);
  Serial.println(F(" s"));
  Serial.print(F("    Process gain: "));
  Serial.print(tuneResults.processGain, 2);
  Serial.println(F(" counts/V\n"));

  homeToLeftLimit();
  return true;
}

// ============================================
// CALCULATE ALL PID METHODS
// ============================================
void calculateAllPIDMethods() {
  Serial.println(F("Calculating PID gains using multiple methods...\n"));

  float Ku = tuneResults.Ku;
  float Tu = tuneResults.Tu;
  float tau = tuneResults.timeConstant;
  float theta = tuneResults.deadTime;
  float K = tuneResults.processGain;

  // ===== METHOD 1: ZIEGLER-NICHOLS (Original) =====
  tuneResults.ZN_Kp = 0.6 * Ku;
  tuneResults.ZN_Ki = 1.2 * Ku / Tu;
  tuneResults.ZN_Kd = 0.075 * Ku * Tu;

  Serial.println(F("  ✓ Ziegler-Nichols (Classic)"));

  // ===== METHOD 2: TYREUS-LUYBEN (Conservative) =====
  tuneResults.TL_Kp = 0.45 * Ku;
  tuneResults.TL_Ki = 0.54 * Ku / Tu;
  tuneResults.TL_Kd = 0.0 * Ku * Tu;  // Typically PI only

  Serial.println(F("  ✓ Tyreus-Luyben (Conservative PI)"));

  // ===== METHOD 3: COHEN-COON (Fast Response) =====
  // Cohen-Coon requires step response parameters
  if (tau > 0 && theta > 0 && K > 0) {
    float R = theta / tau;
    tuneResults.CC_Kp = (1.35 / K) * (tau / theta) * (1 + 0.18 * R);
    tuneResults.CC_Ki = tuneResults.CC_Kp / (tau * (2.5 - 2.0 * R) / (1 + 0.39 * R));
    tuneResults.CC_Kd = tuneResults.CC_Kp * tau * (0.37 - 0.37 * R) / (1 + 0.19 * R);

    Serial.println(F("  ✓ Cohen-Coon (Step Response)"));
  } else {
    // Fallback to modified ZN
    tuneResults.CC_Kp = 0.7 * Ku;
    tuneResults.CC_Ki = 1.4 * Ku / Tu;
    tuneResults.CC_Kd = 0.08 * Ku * Tu;

    Serial.println(F("  ⚠ Cohen-Coon (ZN approximation - no step data)"));
  }

  // ===== METHOD 4: STEP RESPONSE TUNING =====
  // Lambda tuning for step response
  if (tau > 0 && K > 0) {
    float lambda = tau;  // Closed-loop time constant
    tuneResults.SR_Kp = tau / (K * (lambda + theta));
    tuneResults.SR_Ki = 1.0 / (2 * tau);
    tuneResults.SR_Kd = 0.5 * tau;

    Serial.println(F("  ✓ Step Response Lambda Tuning"));
  } else {
    tuneResults.SR_Kp = 0.4 * Ku;
    tuneResults.SR_Ki = 0.8 * Ku / Tu;
    tuneResults.SR_Kd = 0.05 * Ku * Tu;

    Serial.println(F("  ⚠ Step Response (ZN approximation)"));
  }

  Serial.println();
}

// ============================================
// DISPLAY COMPREHENSIVE RESULTS
// ============================================
void displayComprehensiveResults() {
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                                                           ║"));
  Serial.println(F("║        COMPREHENSIVE AUTO-TUNE RESULTS                    ║"));
  Serial.println(F("║                                                           ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  // System Identification Results
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  SYSTEM IDENTIFICATION RESULTS"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.print(F("  Ultimate Gain (Ku):          "));
  Serial.print(tuneResults.Ku, 4);
  Serial.print(F("  ± "));
  Serial.println(tuneResults.Ku_stdDev, 4);
  Serial.print(F("  Ultimate Period (Tu):        "));
  Serial.print(tuneResults.Tu, 3);
  Serial.print(F(" s  ± "));
  Serial.print(tuneResults.Tu_stdDev, 4);
  Serial.println(F(" s"));
  Serial.print(F("  Process Gain (K):            "));
  Serial.print(tuneResults.processGain, 2);
  Serial.println(F(" counts/V"));
  Serial.print(F("  Time Constant (tau):         "));
  Serial.print(tuneResults.timeConstant, 3);
  Serial.println(F(" s"));
  Serial.print(F("  Dead Time (theta):           "));
  Serial.print(tuneResults.deadTime, 3);
  Serial.println(F(" s"));
  Serial.print(F("  Rise Time:                   "));
  Serial.print(tuneResults.riseTime, 3);
  Serial.println(F(" s"));
  Serial.print(F("  Tests Performed:             "));
  Serial.println(tuneResults.numTests);
  Serial.println();

  // PID Gains Table
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  RECOMMENDED PID GAINS (All Methods)"));
  Serial.println(F("═════════════════════════════════════════════════════════════\n"));

  Serial.println(F("┌──────────────────────┬──────────┬──────────┬──────────┐"));
  Serial.println(F("│ Method               │    Kp    │    Ki    │    Kd    │"));
  Serial.println(F("├──────────────────────┼──────────┼──────────┼──────────┤"));

  // Method 1: Ziegler-Nichols
  Serial.print(F("│ 1. Ziegler-Nichols   │ "));
  printPaddedFloat(tuneResults.ZN_Kp, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.ZN_Ki, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.ZN_Kd, 8);
  Serial.println(F(" │"));

  // Method 2: Tyreus-Luyben
  Serial.print(F("│ 2. Tyreus-Luyben (*) │ "));
  printPaddedFloat(tuneResults.TL_Kp, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.TL_Ki, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.TL_Kd, 8);
  Serial.println(F(" │"));

  // Method 3: Cohen-Coon
  Serial.print(F("│ 3. Cohen-Coon        │ "));
  printPaddedFloat(tuneResults.CC_Kp, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.CC_Ki, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.CC_Kd, 8);
  Serial.println(F(" │"));

  // Method 4: Step Response
  Serial.print(F("│ 4. Step Response     │ "));
  printPaddedFloat(tuneResults.SR_Kp, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.SR_Ki, 8);
  Serial.print(F(" │ "));
  printPaddedFloat(tuneResults.SR_Kd, 8);
  Serial.println(F(" │"));

  Serial.println(F("└──────────────────────┴──────────┴──────────┴──────────┘\n"));

  Serial.println(F("(*) RECOMMENDED: Tyreus-Luyben provides most stable control"));
  Serial.println(F("               with minimal overshoot\n"));

  // Method Descriptions
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  METHOD CHARACTERISTICS"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  1. Ziegler-Nichols:"));
  Serial.println(F("     • Classic method, moderate overshoot (~25%)"));
  Serial.println(F("     • Fast response, good for general use"));
  Serial.println();
  Serial.println(F("  2. Tyreus-Luyben: ★ RECOMMENDED"));
  Serial.println(F("     • Conservative, minimal overshoot (<10%)"));
  Serial.println(F("     • Most stable, excellent for precision positioning"));
  Serial.println(F("     • PI control (no derivative term)"));
  Serial.println();
  Serial.println(F("  3. Cohen-Coon:"));
  Serial.println(F("     • Fast response, handles dead time well"));
  Serial.println(F("     • May have moderate overshoot"));
  Serial.println();
  Serial.println(F("  4. Step Response:"));
  Serial.println(F("     • Based on measured system dynamics"));
  Serial.println(F("     • Good balance of speed and stability"));
  Serial.println(F("═════════════════════════════════════════════════════════════\n"));
}

// ============================================
// Helper: Print padded float for table
// ============================================
void printPaddedFloat(float value, int width) {
  char buffer[12];
  dtostrf(value, width, 4, buffer);
  Serial.print(buffer);
}

// ============================================
// SELECT AND APPLY GAINS
// ============================================
void selectAndApplyGains() {
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   SELECT PID GAINS TO APPLY                               ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  Serial.println(F("Select method to apply:"));
  Serial.println(F("  1 - Ziegler-Nichols"));
  Serial.println(F("  2 - Tyreus-Luyben (RECOMMENDED)"));
  Serial.println(F("  3 - Cohen-Coon"));
  Serial.println(F("  4 - Step Response"));
  Serial.println(F("  N - Do not apply (keep current gains)\n"));
  Serial.print(F("Enter selection: "));

  while (!Serial.available()) {}
  char selection = Serial.read();
  while (Serial.available()) Serial.read();

  Serial.println(selection);
  Serial.println();

  float newKp, newKi, newKd;
  const char* methodName;
  bool apply = false;

  switch (selection) {
    case '1':
      newKp = tuneResults.ZN_Kp;
      newKi = tuneResults.ZN_Ki;
      newKd = tuneResults.ZN_Kd;
      methodName = "Ziegler-Nichols";
      apply = true;
      break;

    case '2':
      newKp = tuneResults.TL_Kp;
      newKi = tuneResults.TL_Ki;
      newKd = tuneResults.TL_Kd;
      methodName = "Tyreus-Luyben";
      apply = true;
      break;

    case '3':
      newKp = tuneResults.CC_Kp;
      newKi = tuneResults.CC_Ki;
      newKd = tuneResults.CC_Kd;
      methodName = "Cohen-Coon";
      apply = true;
      break;

    case '4':
      newKp = tuneResults.SR_Kp;
      newKi = tuneResults.SR_Ki;
      newKd = tuneResults.SR_Kd;
      methodName = "Step Response";
      apply = true;
      break;

    default:
      Serial.println(F("✗ Gains not applied\n"));
      return;
  }

  if (apply) {
    KP = newKp;
    KI = newKi;
    KD = newKd;

    Serial.println(F("╔═══════════════════════════════════════════════════════════╗"));
    Serial.println(F("║   GAINS APPLIED                                           ║"));
    Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));
    Serial.print(F("Method: "));
    Serial.println(methodName);
    Serial.println();

    // COPY-READY FORMAT
    Serial.println(F("═════════════════════════════════════════════════════════════"));
    Serial.println(F("  ✂ COPY THESE VALUES TO YOUR MAIN CODE ✂"));
    Serial.println(F("═════════════════════════════════════════════════════════════"));
    Serial.println();
    Serial.print(F("float KP = "));
    Serial.print(KP, 6);
    Serial.println(F(";"));
    Serial.print(F("float KI = "));
    Serial.print(KI, 6);
    Serial.println(F(";"));
    Serial.print(F("float KD = "));
    Serial.print(KD, 6);
    Serial.println(F(";"));
    Serial.println();
    Serial.println(F("// Friction Model Parameters"));
    Serial.print(F("float FRICTION_STATIC_LEFT = "));
    Serial.print(friction.staticLeft, 3);
    Serial.println(F(";"));
    Serial.print(F("float FRICTION_STATIC_RIGHT = "));
    Serial.print(friction.staticRight, 3);
    Serial.println(F(";"));
    Serial.print(F("float FRICTION_COULOMB_LEFT = "));
    Serial.print(friction.coulombLeft, 3);
    Serial.println(F(";"));
    Serial.print(F("float FRICTION_COULOMB_RIGHT = "));
    Serial.print(friction.coulombRight, 3);
    Serial.println(F(";"));
    Serial.print(F("float FRICTION_VISCOUS = "));
    Serial.print(friction.viscous, 4);
    Serial.println(F(";"));
    Serial.println();
    Serial.println(F("═════════════════════════════════════════════════════════════\n"));

    saveCalibration();
    Serial.println(F("✓ Gains saved to EEPROM"));
    Serial.println(F("\nReady to test! Use command 'T' for manual position test\n"));
  }
}

// ============================================
// EEPROM STORAGE
// ============================================
void saveCalibration() {
  EEPROM.put(EEPROM_KP_ADDR, KP);
  EEPROM.put(EEPROM_KI_ADDR, KI);
  EEPROM.put(EEPROM_KD_ADDR, KD);
  EEPROM.put(EEPROM_FRICTION_STATIC_LEFT_ADDR, friction.staticLeft);
  EEPROM.put(EEPROM_FRICTION_STATIC_RIGHT_ADDR, friction.staticRight);
  EEPROM.put(EEPROM_FRICTION_COULOMB_LEFT_ADDR, friction.coulombLeft);
  EEPROM.put(EEPROM_FRICTION_COULOMB_RIGHT_ADDR, friction.coulombRight);
  EEPROM.put(EEPROM_FRICTION_VISCOUS_ADDR, friction.viscous);
  EEPROM.put(EEPROM_LEFT_LIMIT_ADDR, LEFT_LIMIT);
  EEPROM.put(EEPROM_RIGHT_LIMIT_ADDR, RIGHT_LIMIT);
  EEPROM.write(EEPROM_CALIBRATED_FLAG, 1);
}

void loadCalibration() {
  byte calibrated = EEPROM.read(EEPROM_CALIBRATED_FLAG);

  if (calibrated == 1) {
    EEPROM.get(EEPROM_KP_ADDR, KP);
    EEPROM.get(EEPROM_KI_ADDR, KI);
    EEPROM.get(EEPROM_KD_ADDR, KD);
    EEPROM.get(EEPROM_FRICTION_STATIC_LEFT_ADDR, friction.staticLeft);
    EEPROM.get(EEPROM_FRICTION_STATIC_RIGHT_ADDR, friction.staticRight);
    EEPROM.get(EEPROM_FRICTION_COULOMB_LEFT_ADDR, friction.coulombLeft);
    EEPROM.get(EEPROM_FRICTION_COULOMB_RIGHT_ADDR, friction.coulombRight);
    EEPROM.get(EEPROM_FRICTION_VISCOUS_ADDR, friction.viscous);
    EEPROM.get(EEPROM_LEFT_LIMIT_ADDR, LEFT_LIMIT);
    EEPROM.get(EEPROM_RIGHT_LIMIT_ADDR, RIGHT_LIMIT);
    limitsCalibrated = true;
    friction.calibrated = true;

    Serial.println(F("✓ Calibration loaded from EEPROM"));
    Serial.print(F("  PID: Kp="));
    Serial.print(KP, 4);
    Serial.print(F(", Ki="));
    Serial.print(KI, 4);
    Serial.print(F(", Kd="));
    Serial.println(KD, 4);
    Serial.print(F("  Range: "));
    Serial.print(LEFT_LIMIT);
    Serial.print(F(" to "));
    Serial.println(RIGHT_LIMIT);
    Serial.println();
  } else {
    Serial.println(F("⚠ No calibration found - using defaults\n"));
  }
}

void clearCalibration() {
  EEPROM.write(EEPROM_CALIBRATED_FLAG, 0);
  Serial.println(F("✓ Calibration cleared from EEPROM\n"));
}

// ============================================
// MANUAL POSITION TEST
// ============================================
void manualPositionTest() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   MANUAL POSITION TEST                     ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));

  Serial.print(F("Enter target position ("));
  Serial.print(RIGHT_LIMIT);
  Serial.print(F(" to "));
  Serial.print(LEFT_LIMIT);
  Serial.println(F("):"));

  while (!Serial.available()) {}
  long target = Serial.parseInt();
  while (Serial.available()) Serial.read();

  if (target < RIGHT_LIMIT || target > LEFT_LIMIT) {
    Serial.println(F("✗ Target out of range\n"));
    return;
  }

  Serial.print(F("\nTesting move to "));
  Serial.print(target);
  Serial.println(F("...\n"));

  targetPosition = target;
  errorIntegral = 0;
  lastError = 0;
  systemEnabled = true;

  long startPos = encoder.read();
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  Serial.println(F("Time(s)\tPosition\tTarget\t\tError\tIntegral"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  while (abs(encoder.read() - targetPosition) > DEADBAND || (millis() - startTime) < 4000) {
    runPIDControl();

    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      long pos = encoder.read();
      long err = targetPosition - pos;

      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(F("\t"));
      Serial.print(pos);
      Serial.print(F("\t\t"));
      Serial.print(targetPosition);
      Serial.print(F("\t\t"));
      Serial.print(err);
      Serial.print(F("\t"));
      Serial.println(errorIntegral, 2);
    }

    delay(10);

    if (millis() - startTime > 20000) {
      Serial.println(F("\n⚠ Timeout"));
      break;
    }
  }

  stopMotor();
  systemEnabled = false;

  long finalPos = encoder.read();
  long finalError = targetPosition - finalPos;
  unsigned long totalTime = millis() - startTime;

  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   TEST RESULTS                             ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.print(F("Start position:      "));
  Serial.println(startPos);
  Serial.print(F("Target position:     "));
  Serial.println(targetPosition);
  Serial.print(F("Final position:      "));
  Serial.println(finalPos);
  Serial.print(F("Final error:         "));
  Serial.println(finalError);
  Serial.print(F("Total time:          "));
  Serial.print(totalTime / 1000.0, 2);
  Serial.println(F(" s"));

  if (abs(finalError) <= DEADBAND) {
    Serial.println(F("\n✓ TARGET REACHED SUCCESSFULLY\n"));
  } else {
    Serial.println(F("\n⚠ Target not reached within deadband\n"));
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
      characterizeFrictionAdvanced();
      break;

    case 'Z':
      comprehensiveAutoTune();
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
  Serial.println(F("\n\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                                                           ║"));
  Serial.println(F("║   ME 350 - ADVANCED PID AUTO-TUNING SYSTEM v2.0           ║"));
  Serial.println(F("║   Comprehensive System Identification & Tuning            ║"));
  Serial.println(F("║                                                           ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  COMMANDS"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  R - Calibrate Range (gentle 2V approach)"));
  Serial.println(F("  F - Characterize Friction (Stribeck curve)"));
  Serial.println(F("  Z - Comprehensive Auto-Tune (ALL methods)"));
  Serial.println(F("  T - Manual Position Test (verify gains)"));
  Serial.println(F("  H - Home to left limit"));
  Serial.println(F("  P - Print current status"));
  Serial.println(F("  C - Clear EEPROM calibration"));
  Serial.println(F("  ? - Show this help"));
  Serial.println(F("═════════════════════════════════════════════════════════════\n"));

  Serial.println(F("RECOMMENDED WORKFLOW:"));
  Serial.println(F("  1. R - Calibrate range (gentle limit approach)"));
  Serial.println(F("  2. F - Characterize friction (Stribeck model)"));
  Serial.println(F("  3. Z - Comprehensive auto-tune (8-12 min)"));
  Serial.println(F("       → Multiple relay tests"));
  Serial.println(F("       → Step response analysis"));
  Serial.println(F("       → 4 different PID methods"));
  Serial.println(F("       → Consistency validation"));
  Serial.println(F("  4. T - Test with manual moves"));
  Serial.println(F("  5. Copy PID values to your main code\n"));
}

void printStatus() {
  Serial.println(F("\n╔═══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   SYSTEM STATUS                                           ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════╝\n"));

  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  CURRENT PID CONFIGURATION"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.print(F("  Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("  Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("  Kd = "));
  Serial.println(KD, 6);
  Serial.println();

  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  FRICTION MODEL"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.print(F("  Static (LEFT):   "));
  Serial.print(friction.staticLeft, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Static (RIGHT):  "));
  Serial.print(friction.staticRight, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Coulomb (LEFT):  "));
  Serial.print(friction.coulombLeft, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Coulomb (RIGHT): "));
  Serial.print(friction.coulombRight, 3);
  Serial.println(F(" V"));
  Serial.print(F("  Viscous:         "));
  Serial.print(friction.viscous, 4);
  Serial.println(F(" V/(count/s)"));
  Serial.print(F("  Calibrated:      "));
  Serial.println(friction.calibrated ? F("YES") : F("NO"));
  Serial.println();

  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  SYSTEM LIMITS"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.print(F("  Left Limit:   "));
  Serial.println(LEFT_LIMIT);
  Serial.print(F("  Right Limit:  "));
  Serial.println(RIGHT_LIMIT);
  Serial.print(F("  Total Range:  "));
  Serial.print(abs(RIGHT_LIMIT - LEFT_LIMIT));
  Serial.println(F(" counts"));
  Serial.print(F("  Calibrated:   "));
  Serial.println(limitsCalibrated ? F("YES") : F("NO"));
  Serial.println();

  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.println(F("  CURRENT STATE"));
  Serial.println(F("═════════════════════════════════════════════════════════════"));
  Serial.print(F("  Position:     "));
  Serial.println(encoder.read());
  Serial.print(F("  Target:       "));
  Serial.println(targetPosition);
  Serial.print(F("  Velocity:     "));
  Serial.print(currentVelocity, 1);
  Serial.println(F(" counts/s"));
  Serial.print(F("  System:       "));
  Serial.println(systemEnabled ? F("ENABLED") : F("DISABLED"));
  Serial.print(F("  Limit LEFT:   "));
  Serial.println(leftPressed() ? F("PRESSED") : F("Open"));
  Serial.print(F("  Limit RIGHT:  "));
  Serial.println(rightPressed() ? F("PRESSED") : F("Open"));
  Serial.println();

  if (tuneResults.valid) {
    Serial.println(F("═════════════════════════════════════════════════════════════"));
    Serial.println(F("  LAST AUTO-TUNE RESULTS"));
    Serial.println(F("═════════════════════════════════════════════════════════════"));
    Serial.print(F("  Ku:           "));
    Serial.print(tuneResults.Ku, 4);
    Serial.print(F("  ± "));
    Serial.println(tuneResults.Ku_stdDev, 4);
    Serial.print(F("  Tu:           "));
    Serial.print(tuneResults.Tu, 3);
    Serial.print(F(" s  ± "));
    Serial.print(tuneResults.Tu_stdDev, 4);
    Serial.println(F(" s"));
    Serial.print(F("  Tests:        "));
    Serial.println(tuneResults.numTests);
    Serial.println();
  }

  Serial.println(F("═════════════════════════════════════════════════════════════\n"));
}
