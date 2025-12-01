// ME350 PID Auto-Tune

#include <Encoder.h>
#include <EEPROM.h>
#include <math.h>

// PIN CONFIGURATION

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

// GLOBAL OBJECTS

Encoder motorEncoder(ENCODER_A, ENCODER_B);

// PID PARAMETERS

float KP = 0.020;  // Proportional gain
float KI = 0.005;  // Integral gain
float KD = 0.004;  // Derivative gain

// PID state variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float lastDerivative = 0;  // For derivative filtering

// Filtering constants
const float DERIVATIVE_FILTER_ALPHA = 0.7;  // Low-pass filter for derivative (0-1, higher = less filtering)
const float POSITION_FILTER_ALPHA = 0.85;   // Low-pass filter for position readings

// Control parameters
const long DEADBAND = 5;  // Encoder counts
const unsigned long CONTROL_PERIOD = 10;  // ms (100 Hz)
const float TEST_MAX_VOLTAGE = 9.0;  // Base max drive during manual tests (9V nominal as per UpdatedGameCodeNov17.ino)
const unsigned long LOG_INTERVAL_MS = 30; // Logging cadence for manual tests/moves

// Anti-windup parameters
const float INTEGRAL_MAX = 1000.0;  // Maximum integral accumulation
const float INTEGRAL_DECAY_FAR = 0.95;  // Decay rate when far from target
const float INTEGRAL_DECAY_CROSS = 0.5;  // Decay on zero crossing

// FRICTION

float FRICTION_LEFT = 2.2;   // Voltage to overcome friction moving LEFT
float FRICTION_RIGHT = 2.9;  // Voltage to overcome friction moving RIGHT
bool frictionCharacterized = false;

const float HOMING_EXTRA_VOLTAGE = 0.6;     // Added on top of friction during homing
const unsigned long HOMING_HOLD_TIME = 500; // ms to hold on switch before zeroing (increased)
const int HOMING_STABLE_TICKS = 5;          // Require this many consecutive stable readings (increased)

// CALIBRATION

long LEFT_LIMIT_POSITION = 0;
long RIGHT_LIMIT_POSITION = -1800;
long TOTAL_RANGE = 0;

// Lane positions (can be set via serial)
long LANE_POSITIONS[4] = {0, -350, -700, -1050};

bool isCalibrated = false;

// EEPROM STORAGE

const int EEPROM_CALIBRATION_FLAG = 0;
const int EEPROM_KP = 1;
const int EEPROM_KI = 5;
const int EEPROM_KD = 9;
const int EEPROM_FRICTION_LEFT = 13;
const int EEPROM_FRICTION_RIGHT = 17;
const int EEPROM_LEFT_LIMIT = 21;
const int EEPROM_RIGHT_LIMIT = 25;
const int EEPROM_LANES_BASE = 29;  // 4 lanes * 4 bytes each

// AUTO-TUNE RESULTS

struct TuneResults {
  float Ku;           // Ultimate gain
  float Tu;           // Ultimate period
  float amplitude;    // Oscillation amplitude
  int peakCount;      // Number of peaks collected
  float stdDev;       // Standard deviation of periods (quality metric)
  bool valid;         // Results validity flag
};

TuneResults lastTuneResults = {0, 0, 0, 0, 0, false};

// ============================================================================
// FILTERING AND SIGNAL PROCESSING
// ============================================================================

// Simple exponential filter for position (saves memory vs moving average)
float filteredPosition = 0;
bool positionFilterInitialized = false;

// Exponential moving average for derivative
float filteredDerivative(float rawDerivative) {
  lastDerivative = DERIVATIVE_FILTER_ALPHA * rawDerivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * lastDerivative;
  return lastDerivative;
}

// SETUP

void setup() {
  Serial.begin(115200);

  // Configure motor pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);

  // Configure limit switches (active HIGH per hardware wiring)
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

// MAIN LOOP

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    processCommand(command);
  }
}

// COMMANDS

void processCommand(char cmd) {
  // Ignore whitespace/newlines to reduce "Unknown command" spam
  if (cmd == '\r' || cmd == '\n' || cmd == ' ' || cmd == '\t') return;

  switch (toupper(cmd)) {
    case 'R':
      calibrateRange();
      break;

    case 'F':
      characterizeFriction();
      break;

    case 'Z':
      if (!ensureRangeAndFrictionReady()) return;
      autoTuneZieglerNichols();
      break;

    case 'S':
      if (!ensureRangeAndFrictionReady()) return;
      stepResponse();
      break;

    case 'T':
      if (!ensureRangeAndFrictionReady()) return;
      manualPositionTest();
      break;

    case 'M':
      if (!ensureRangeAndFrictionReady()) return;
      manualLaneMove();
      break;

    case 'X':
      if (!ensureRangeAndFrictionReady()) return;
      testLaneTransitions();
      break;

    case 'H':
      if (!ensureRangeAndFrictionReady()) return;
      homeToLeft();
      break;

    case 'P':
      printStatus();
      break;

    case 'C':
      clearCalibration();
      break;

    case '1': case '2': case '3': case '4': {
      int lane = cmd - '1';
      LANE_POSITIONS[lane] = motorEncoder.read();
      Serial.print(F("L"));
      Serial.print(lane + 1);
      Serial.print(F(":"));
      Serial.println(LANE_POSITIONS[lane]);
      saveCalibration();
      break;
    }

    case 'L':
      loadCalibration();
      Serial.println(F("Loaded"));
      printStatus();
      break;

    case 'W':
      saveCalibration();
      Serial.println(F("Saved"));
      break;

    case 'I':
      setPIDValues();
      break;

    case '?':
    case '/':
      printHelp();
      break;

    default:
      Serial.println(F("? for help"));
      break;
  }
}

// MOTOR CONTROL

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
  if (digitalRead(LIMIT_LEFT) == HIGH && voltage > 0) {
    voltage = 0;
    pwmValue = 0;
  }
  if (digitalRead(LIMIT_RIGHT) == HIGH && voltage < 0) {
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

// PID CONTROLLER

bool ensureRangeAndFrictionReady() {
  if (!isCalibrated) {
    calibrateRange();
  }

  if (!frictionCharacterized) {
    characterizeFriction();
  }

  return isCalibrated && frictionCharacterized;
}

float updatePID(long targetPosition) {
  // Read and filter position
  long rawPosition = motorEncoder.read();
  if (!positionFilterInitialized) {
    filteredPosition = (float)rawPosition;
    positionFilterInitialized = true;
  } else {
    filteredPosition = POSITION_FILTER_ALPHA * (float)rawPosition + (1.0 - POSITION_FILTER_ALPHA) * filteredPosition;
  }
  long currentPosition = (long)filteredPosition;

  // Calculate error
  error = targetPosition - currentPosition;

  // Apply deadband with hysteresis
  if (abs(error) < DEADBAND) {
    error = 0;
    // Conditional integral reset - only reset if we've been stable
    if (abs(integral) < 10.0) {
      integral = 0;
    } else {
      // Gradual decay instead of hard reset
      integral *= 0.9;
    }
  }
  else {
    // Calculate integral with conditional integration (anti-windup)
    float dt = CONTROL_PERIOD / 1000.0;
    float integralTerm = error * dt;
    
    // Conditional integration - don't accumulate if output is saturated
    float testIntegral = integral + integralTerm;
    if (abs(testIntegral) < INTEGRAL_MAX) {
      integral = testIntegral;
    }
    // Otherwise, don't accumulate (clamping anti-windup)
  }

  // Calculate derivative with filtering
  float rawDerivative = (error - lastError) / (CONTROL_PERIOD / 1000.0);
  derivative = filteredDerivative(rawDerivative);

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

// RANGE CALIBRATION

void calibrateRange() {
  Serial.println(F("\nRANGE CAL"));

  // Move to left limit
  float leftDrive = max(FRICTION_RIGHT + HOMING_EXTRA_VOLTAGE + 0.1, 3.0);
  setMotorVoltage(leftDrive);

  unsigned long startTime = millis();
  long lastPosition = motorEncoder.read();
  unsigned long lastMoveTime = millis();

  while (digitalRead(LIMIT_LEFT) == LOW) {  // LOW = not pressed
    if (millis() - startTime > 12000) {  // Increased timeout
      Serial.println(F("ERR: timeout"));
      setMotorVoltage(0);
      return;
    }
    
    // Check for stuck condition
    long currentPos = motorEncoder.read();
    if (abs(currentPos - lastPosition) > 2) {
      lastMoveTime = millis();
      lastPosition = currentPos;
    } else if (millis() - lastMoveTime > 3000) {
      leftDrive = min(leftDrive + 0.5, 8.0);
      setMotorVoltage(leftDrive);
      lastMoveTime = millis();
    }
    
    delay(10);
  }

  // Wait for motor to stop
  delay(300);
  
  // Hold against left limit to seat before zeroing; require stability
  unsigned long holdStart = millis();
  float holdVoltage = max(FRICTION_RIGHT + 0.1, 2.5);
  long lastPos = motorEncoder.read();
  int stableTicks = 0;
  while (millis() - holdStart < HOMING_HOLD_TIME || stableTicks < HOMING_STABLE_TICKS) {
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
  setMotorVoltage(0);
  delay(200);

  // Zero the encoder
  motorEncoder.write(0);
  LEFT_LIMIT_POSITION = 0;
  positionFilterInitialized = false;  // Reset filter

  // Move to right limit
  delay(500);
  float rightDrive = -max(FRICTION_LEFT + HOMING_EXTRA_VOLTAGE + 0.1, 3.0);
  setMotorVoltage(rightDrive);

  startTime = millis();
  lastPosition = 0;
  lastMoveTime = millis();

  while (digitalRead(LIMIT_RIGHT) == LOW) {
    if (millis() - startTime > 12000) {
      Serial.println(F("ERR: timeout"));
      setMotorVoltage(0);
      return;
    }
    
    // Check for stuck condition
    long currentPos = motorEncoder.read();
    if (abs(currentPos - lastPosition) > 2) {
      lastMoveTime = millis();
      lastPosition = currentPos;
    } else if (millis() - lastMoveTime > 3000) {
      rightDrive = max(rightDrive - 0.5, -8.0);
      setMotorVoltage(rightDrive);
      lastMoveTime = millis();
    }
    
    delay(10);
  }

  // Wait for motor to stop
  delay(300);
  
  // Hold on right limit briefly with stability check
  holdStart = millis();
  float holdVoltageRight = -max(FRICTION_LEFT + 0.1, 2.5);
  lastPos = motorEncoder.read();
  stableTicks = 0;
  while (millis() - holdStart < HOMING_HOLD_TIME || stableTicks < HOMING_STABLE_TICKS) {
    setMotorVoltage(holdVoltageRight);
    delay(10);
    long pos = motorEncoder.read();
    if (abs(pos - lastPos) <= 1) {
      stableTicks++;
    } else {
      stableTicks = 0;
      lastPos = pos;
    }
  }
  setMotorVoltage(0);
  delay(200);

  // Record right limit position
  RIGHT_LIMIT_POSITION = motorEncoder.read();
  TOTAL_RANGE = abs(RIGHT_LIMIT_POSITION - LEFT_LIMIT_POSITION);

  Serial.print(F("R:"));
  Serial.println(RIGHT_LIMIT_POSITION);

  Serial.print(F("Range:"));
  Serial.println(TOTAL_RANGE);

  if (TOTAL_RANGE < 100) {
    Serial.println(F("ERR: range too small"));
    return;
  }

  isCalibrated = true;
  saveCalibration();

  Serial.println(F("Range calibration complete!"));

  // Return to center
  homeToLeft();
}

// FRICTION CHARACTERIZATION

void characterizeFriction() {
  Serial.println(F("\nFRICTION CAL"));

  if (!isCalibrated) {
    Serial.println(F("ERR: Run R first"));
    return;
  }

  // Home to left limit first
  homeToLeft();
  delay(1000);

  // Characterize RIGHT friction (moving from left to right) - multiple measurements

  float measurements[3];
  int measurementCount = 0;

  for (int attempt = 0; attempt < 3; attempt++) {
    float testVoltage = 0.5;
    bool motionDetected = false;

    while (!motionDetected && testVoltage < 8.0) {
      long startPos = motorEncoder.read();
      setMotorVoltage(-testVoltage);  // Negative = RIGHT
      delay(400);  // Longer test duration
      long endPos = motorEncoder.read();
      setMotorVoltage(0);
      delay(300);

      long movement = abs(endPos - startPos);

      if (movement > 15) {  // Increased threshold for more reliable detection
        motionDetected = true;
        measurements[measurementCount] = testVoltage - 0.15;  // Safety margin
        measurementCount++;
        
        // Return to start position
        homeToLeft();
        delay(1000);
        break;
      }
      else {
        testVoltage += 0.2;  // Smaller increments for better resolution
      }
    }
  }

  if (measurementCount < 2) {
    Serial.println(F("ERR"));
    return;
  }

  // Calculate average, rejecting outliers
  float sum = 0;
  for (int i = 0; i < measurementCount; i++) {
    sum += measurements[i];
  }
  float avg = sum / measurementCount;
  
  // Calculate standard deviation
  float variance = 0;
  for (int i = 0; i < measurementCount; i++) {
    variance += (measurements[i] - avg) * (measurements[i] - avg);
  }
  float stdDev = sqrt(variance / measurementCount);
  
  // Use median if high variance (outlier rejection)
  if (stdDev > 0.3) {
    // Simple bubble sort for median
    for (int i = 0; i < measurementCount - 1; i++) {
      for (int j = 0; j < measurementCount - i - 1; j++) {
        if (measurements[j] > measurements[j + 1]) {
          float temp = measurements[j];
          measurements[j] = measurements[j + 1];
          measurements[j + 1] = temp;
        }
      }
    }
    FRICTION_LEFT = measurements[measurementCount / 2];  // Median
  } else {
    FRICTION_LEFT = avg;
  }

  Serial.print(F("R:"));
  Serial.println(FRICTION_LEFT, 1);

  // Move to right limit
  float driveRight = -max(FRICTION_LEFT + HOMING_EXTRA_VOLTAGE + 0.1, 3.0);
  setMotorVoltage(driveRight);
  unsigned long driveStart = millis();
  while (digitalRead(LIMIT_RIGHT) == LOW) {
    if (millis() - driveStart > 12000) {
      Serial.println(F("ERR: timeout"));
      setMotorVoltage(0);
      return;
    }
    delay(10);
  }
  // Hold briefly on right limit
  unsigned long holdStart = millis();
  float holdRight = -max(FRICTION_LEFT, 2.5);
  while (millis() - holdStart < HOMING_HOLD_TIME) {
    setMotorVoltage(holdRight);
    delay(10);
  }
  setMotorVoltage(0);
  delay(500);

  // Characterize LEFT friction (moving from right to left) - multiple measurements

  measurementCount = 0;
  for (int attempt = 0; attempt < 3; attempt++) {
    float testVoltage = 0.5;
    bool motionDetected = false;

    while (!motionDetected && testVoltage < 8.0) {
      long startPos = motorEncoder.read();
      setMotorVoltage(testVoltage);  // Positive = LEFT
      delay(400);
      long endPos = motorEncoder.read();
      setMotorVoltage(0);
      delay(300);

      long movement = abs(endPos - startPos);

      if (movement > 15) {
        motionDetected = true;
        measurements[measurementCount] = testVoltage - 0.15;
        measurementCount++;
        
        // Return to right limit
        driveRight = -max(FRICTION_LEFT + HOMING_EXTRA_VOLTAGE + 0.1, 3.0);
        setMotorVoltage(driveRight);
        driveStart = millis();
        while (digitalRead(LIMIT_RIGHT) == LOW && millis() - driveStart < 12000) {
          delay(10);
        }
        setMotorVoltage(0);
        delay(1000);
        break;
      }
      else {
        testVoltage += 0.2;
      }
    }
  }

  if (measurementCount < 2) {
    Serial.println(F("ERR"));
    return;
  }

  // Calculate average with outlier rejection
  sum = 0;
  for (int i = 0; i < measurementCount; i++) {
    sum += measurements[i];
  }
  avg = sum / measurementCount;
  
  variance = 0;
  for (int i = 0; i < measurementCount; i++) {
    variance += (measurements[i] - avg) * (measurements[i] - avg);
  }
  stdDev = sqrt(variance / measurementCount);
  
  if (stdDev > 0.3) {
    // Median
    for (int i = 0; i < measurementCount - 1; i++) {
      for (int j = 0; j < measurementCount - i - 1; j++) {
        if (measurements[j] > measurements[j + 1]) {
          float temp = measurements[j];
          measurements[j] = measurements[j + 1];
          measurements[j + 1] = temp;
        }
      }
    }
    FRICTION_RIGHT = measurements[measurementCount / 2];
  } else {
    FRICTION_RIGHT = avg;
  }

  frictionCharacterized = true;
  saveCalibration();

  Serial.print(F("F L:"));
  Serial.print(FRICTION_LEFT, 1);
  Serial.print(F(" R:"));
  Serial.println(FRICTION_RIGHT, 1);

  // Return home
  homeToLeft();
}

// ZIEGLER-NICHOLS AUTO-TUNE

void autoTuneZieglerNichols() {
  Serial.println(F("\nZN TUNE"));

  if (!isCalibrated) {
    Serial.println(F("ERR: Run R first"));
    return;
  }

  Serial.println(F("ZN: Y to start"));

  // Flush any stray input
  while (Serial.available()) { Serial.read(); }
  char response = 0;
  while (response == 0) {
    while (!Serial.available()) { }
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    response = c;
  }

  if (toupper(response) != 'Y') return;
  long centerPosition = (LEFT_LIMIT_POSITION + RIGHT_LIMIT_POSITION) / 2;

  positionFilterInitialized = false;
  
  // Enhanced staged approach - using 9V nominal voltage
  unsigned long moveStart = millis();
  // Stage 1: coarse approach
  while (abs(motorEncoder.read() - centerPosition) > 30 && millis() - moveStart < 10000) {
    long currentPos = motorEncoder.read();
    long err = centerPosition - currentPos;
    float voltage = constrain(err * 0.012, -9.0, 9.0);  // Updated to 9V max
    setMotorVoltage(voltage);
    delay(10);
  }
  
  // Stage 2: fine approach
  moveStart = millis();
  while (abs(motorEncoder.read() - centerPosition) > 10 && millis() - moveStart < 5000) {
    long currentPos = motorEncoder.read();
    long err = centerPosition - currentPos;
    float voltage = constrain(err * 0.008, -8.0, 8.0);  // Updated to 8V max
    setMotorVoltage(voltage);
    delay(10);
  }
  
  // Stage 3: micro adjust
  moveStart = millis();
  while (abs(motorEncoder.read() - centerPosition) > 5 && millis() - moveStart < 4000) {
    long currentPos = motorEncoder.read();
    long err = centerPosition - currentPos;
    float voltage = constrain(err * 0.005, -7.0, 7.0);  // Updated to 7V max
    setMotorVoltage(voltage);
    delay(10);
  }
  
  // Final PID-based centering
  error = lastError = integral = derivative = lastDerivative = 0;
  unsigned long pidStart = millis();
  unsigned long stableStart = millis();
  const int CENTER_TOL = 3;
  int stableCount = 0;
  
  while (millis() - pidStart < 4000) {
    float pidVoltage = updatePID(centerPosition);
    float applied = constrain(pidVoltage, -7.0, 7.0);  // Updated to 7V max
    setMotorVoltage(applied);

    if (abs(error) <= CENTER_TOL) {
      stableCount++;
      if (stableCount > 30) break;  // 300ms stable
    } else {
      stableCount = 0;
      stableStart = millis();
    }
    delay(10);
  }
  setMotorVoltage(0);
  delay(800);  // Longer settle time

  long finalCenterPos = motorEncoder.read();

  // Enhanced relay parameters - using 9V nominal voltage
  const float TEST_VOLTAGE = 9.0;  // Relay amplitude (9V as per UpdatedGameCodeNov17.ino)
  const long HYSTERESIS = TOTAL_RANGE / 8;  // Slightly smaller hysteresis for better oscillation
  const int TARGET_PEAKS = 30;       // Increased for better accuracy
  const int MIN_PEAKS = 20;          // Minimum acceptable (increased)
  const int SETTLE_PEAKS = 6;        // Peaks to skip for settling (increased)
  const unsigned long TIMEOUT = 180000;  // 180 seconds (increased)

  Serial.print(TEST_VOLTAGE);
  Serial.print(F("V "));
  Serial.println(HYSTERESIS);

  // Use zero-crossing detection for period measurement (standard ZN method)
  // Track both peaks for amplitude and zero crossings for period
  struct PeakData {
    long position;
    unsigned long time;
    bool isMax;  // true for maximum, false for minimum
  };
  
  struct ZeroCrossing {
    unsigned long time;
    bool rising;  // true if crossing upward, false if downward
  };
  
  PeakData peaks[TARGET_PEAKS];
  ZeroCrossing crossings[TARGET_PEAKS * 2];  // More crossings than peaks
  int peakCount = 0;
  int crossingCount = 0;
  
  // State for peak and crossing detection
  long lastPosition = motorEncoder.read();
  unsigned long lastTime = millis();
  const long CROSSING_THRESHOLD = 2;  // Minimum distance from center to count as crossing
  int lastSide = 0;  // -1 = below, 0 = at center, 1 = above
  if (lastPosition < centerPosition - CROSSING_THRESHOLD) lastSide = -1;
  else if (lastPosition > centerPosition + CROSSING_THRESHOLD) lastSide = 1;
  bool rising = false;
  long localMax = centerPosition;
  long localMin = centerPosition;
  unsigned long maxTime = millis();
  unsigned long minTime = millis();
  
  // Start oscillation
  unsigned long testStart = millis();
  bool relayState = true;  // Start moving up
  long currentPos = motorEncoder.read();
  long deviation = currentPos - centerPosition;
  
  // Initialize direction and side state
  if (deviation > 0) {
    rising = false;
    relayState = false;
    lastSide = 1;  // Above center
  } else {
    rising = true;
    relayState = true;
    lastSide = -1;  // Below center
  }

  while (peakCount < TARGET_PEAKS && millis() - testStart < TIMEOUT) {
    currentPos = motorEncoder.read();
    deviation = currentPos - centerPosition;
    unsigned long currentTime = millis();

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

    // Detect zero crossings (center crossings) - only count actual crossings
    int currentSide = 0;  // -1 = below, 0 = at center, 1 = above
    if (currentPos < centerPosition - CROSSING_THRESHOLD) currentSide = -1;
    else if (currentPos > centerPosition + CROSSING_THRESHOLD) currentSide = 1;
    
    // Detect crossing: was on one side, now on the other
    if (crossingCount < TARGET_PEAKS * 2 && lastSide != 0 && currentSide != 0) {
      if (lastSide == -1 && currentSide == 1) {
        // Crossing upward (below to above)
        crossings[crossingCount].time = currentTime;
        crossings[crossingCount].rising = true;
        crossingCount++;
      } else if (lastSide == 1 && currentSide == -1) {
        // Crossing downward (above to below)
        crossings[crossingCount].time = currentTime;
        crossings[crossingCount].rising = false;
        crossingCount++;
      }
    }
    
    // Update side state
    if (currentSide != 0) {
      lastSide = currentSide;  // Only update if we're clearly on one side
    }

    // Enhanced peak detection - detect local maxima and minima
    long positionChange = currentPos - lastPosition;
    
    if (positionChange > 0) {
      // Rising
      if (!rising) {
        // Was falling, now rising - found a minimum
        if (peakCount < TARGET_PEAKS && peakCount >= SETTLE_PEAKS) {
          peaks[peakCount - SETTLE_PEAKS].position = localMin;
          peaks[peakCount - SETTLE_PEAKS].time = minTime;
          peaks[peakCount - SETTLE_PEAKS].isMax = false;
        }
        peakCount++;
      }
      rising = true;
      if (currentPos > localMax) {
        localMax = currentPos;
        maxTime = currentTime;
      }
    } else if (positionChange < 0) {
      // Falling
      if (rising) {
        // Was rising, now falling - found a maximum
        if (peakCount < TARGET_PEAKS && peakCount >= SETTLE_PEAKS) {
          peaks[peakCount - SETTLE_PEAKS].position = localMax;
          peaks[peakCount - SETTLE_PEAKS].time = maxTime;
          peaks[peakCount - SETTLE_PEAKS].isMax = true;
        }
        peakCount++;
      }
      rising = false;
      if (currentPos < localMin) {
        localMin = currentPos;
        minTime = currentTime;
      }
    }
    
    lastPosition = currentPos;
    lastTime = currentTime;

    if (peakCount > 0 && peakCount % 6 == 0) {
      Serial.print(F("P:"));
      Serial.print(peakCount);
      Serial.print(F(", Xings: "));
      Serial.println(crossingCount);
    }

    delay(5);  // Short delay for control loop
  }

  setMotorVoltage(0);

  int peaksUsed = peakCount - SETTLE_PEAKS;
  if (peaksUsed < MIN_PEAKS) {
    Serial.println(F("ERROR: Insufficient peaks collected for reliable tuning."));
    Serial.print(F("Collected "));
    Serial.print(peaksUsed);
    Serial.print(F(" usable peaks, needed "));
    Serial.println(MIN_PEAKS);
    lastTuneResults.valid = false;
    return;
  }

  // Calculate amplitudes and periods with outlier rejection
  float amplitudes[TARGET_PEAKS];
  float periods[TARGET_PEAKS];
  int ampCount = 0;
  int periodCount = 0;
  
  // Calculate oscillation amplitudes (center-to-peak, which is what ZN formula uses)
  // The ZN formula Ku = 4V/(πA) uses A as the oscillation amplitude (center to peak)
  for (int i = 0; i < peaksUsed; i++) {
    long amp = abs(peaks[i].position - centerPosition);
    if (amp > 5) {  // Sanity check: minimum 5 counts
      amplitudes[ampCount++] = (float)amp;
    }
  }
  
  // Also calculate peak-to-peak for verification/cross-check
  long maxPeak = centerPosition;
  long minPeak = centerPosition;
  for (int i = 0; i < peaksUsed; i++) {
    if (peaks[i].isMax && peaks[i].position > maxPeak) {
      maxPeak = peaks[i].position;
    }
    if (!peaks[i].isMax && peaks[i].position < minPeak) {
      minPeak = peaks[i].position;
    }
  }
  float peakToPeak = (float)(maxPeak - minPeak);
  float centerToPeakFromP2P = peakToPeak / 2.0;
  
  // Calculate periods using zero crossings (standard ZN method)
  // Measure time between consecutive crossings in the same direction
  // This gives half-periods, which we'll double to get full period
  unsigned long lastRisingTime = 0;
  unsigned long lastFallingTime = 0;
  
  // Skip first few crossings (settling period)
  int skipCrossings = SETTLE_PEAKS * 2;
  
  for (int i = skipCrossings; i < crossingCount; i++) {
    if (crossings[i].rising) {
      // Rising crossing
      if (lastRisingTime > 0) {
        // Time between two rising crossings = full period
        float period = (crossings[i].time - lastRisingTime) / 1000.0;  // Convert to seconds
        if (period > 0.01 && period < 10.0) {  // Sanity check: 10ms to 10s
          periods[periodCount++] = period;
        }
      }
      lastRisingTime = crossings[i].time;
    } else {
      // Falling crossing
      if (lastFallingTime > 0) {
        // Time between two falling crossings = full period
        float period = (crossings[i].time - lastFallingTime) / 1000.0;
        if (period > 0.01 && period < 10.0) {
          periods[periodCount++] = period;
        }
      }
      lastFallingTime = crossings[i].time;
    }
  }
  
  // If we don't have enough periods from same-direction crossings, use alternating crossings
  if (periodCount < 8) {
    periodCount = 0;
    unsigned long lastCrossingTime = 0;
    for (int i = skipCrossings; i < crossingCount; i++) {
      if (lastCrossingTime > 0) {
        // Time between any two crossings = half period
        float halfPeriod = (crossings[i].time - lastCrossingTime) / 1000.0;
        if (halfPeriod > 0.01 && halfPeriod < 10.0) {
          periods[periodCount++] = halfPeriod * 2.0;  // Double to get full period
        }
      }
      lastCrossingTime = crossings[i].time;
    }
  }
  
  // Check if we have enough period measurements
  if (periodCount < 5) {
    Serial.print(F("ERR: "));
    Serial.print(periodCount);
    Serial.println(F(" periods"));
    lastTuneResults.valid = false;
    return;
  }

  // Calculate statistics with outlier rejection (using less aggressive trimming for more data)
  float avgAmplitude = 0;
  float avgPeriod = 0;
  
  // Sort amplitudes for median/trimmed mean
  for (int i = 0; i < ampCount - 1; i++) {
    for (int j = 0; j < ampCount - i - 1; j++) {
      if (amplitudes[j] > amplitudes[j + 1]) {
        float temp = amplitudes[j];
        amplitudes[j] = amplitudes[j + 1];
        amplitudes[j + 1] = temp;
      }
    }
  }
  
  // Use trimmed mean (remove top and bottom 10% for better accuracy)
  int trimCount = (int)(ampCount * 0.1);
  if (trimCount < 1) trimCount = 0;  // Ensure we have at least some data
  float sumAmp = 0;
  int validAmpCount = ampCount - 2 * trimCount;
  if (validAmpCount < 1) validAmpCount = ampCount;  // Fallback to all data
  
  for (int i = trimCount; i < ampCount - trimCount; i++) {
    sumAmp += amplitudes[i];
  }
  avgAmplitude = sumAmp / validAmpCount;
  
  // Refine amplitude using both methods for better accuracy
  // Combine center-to-peak average with half of peak-to-peak
  if (centerToPeakFromP2P > 5) {
    // Weighted average: 70% from individual peaks, 30% from peak-to-peak/2
    // This accounts for asymmetry and gives more accurate result
    avgAmplitude = 0.7 * avgAmplitude + 0.3 * centerToPeakFromP2P;
  }
  
  // Sort periods
  for (int i = 0; i < periodCount - 1; i++) {
    for (int j = 0; j < periodCount - i - 1; j++) {
      if (periods[j] > periods[j + 1]) {
        float temp = periods[j];
        periods[j] = periods[j + 1];
        periods[j + 1] = temp;
      }
    }
  }
  
  // Trimmed mean for periods (10% trimming for better accuracy)
  trimCount = (int)(periodCount * 0.1);
  if (trimCount < 1) trimCount = 0;
  float sumPeriod = 0;
  int validPeriodCount = periodCount - 2 * trimCount;
  if (validPeriodCount < 1) validPeriodCount = periodCount;  // Fallback to all data
  
  for (int i = trimCount; i < periodCount - trimCount; i++) {
    sumPeriod += periods[i];
  }
  avgPeriod = sumPeriod / validPeriodCount;
  
  // Calculate standard deviation of periods (quality metric)
  float periodVariance = 0;
  for (int i = trimCount; i < periodCount - trimCount; i++) {
    periodVariance += (periods[i] - avgPeriod) * (periods[i] - avgPeriod);
  }
  float periodStdDev = 0;
  if (validPeriodCount > 1) {
    periodStdDev = sqrt(periodVariance / validPeriodCount);
  }

  // Validate results
  if (avgAmplitude < 10.0 || avgPeriod < 0.01 || avgPeriod > 10.0) {
    Serial.println(F("ERR"));
    Serial.print(F("A:"));
    Serial.print(avgAmplitude, 0);
    Serial.print(F(" T:"));
    Serial.println(avgPeriod, 2);
    lastTuneResults.valid = false;
    return;
  }

  // Calculate Ku (ultimate gain)
  // Standard ZN: Ku = 4 * V / (π * A) where A is oscillation amplitude (center to peak)
  // The amplitude has already been refined using weighted average method
  float Ku = (4.0 * TEST_VOLTAGE) / (PI * avgAmplitude);

  // Store results - avgPeriod is already the full period
  lastTuneResults.Ku = Ku;
  lastTuneResults.Tu = avgPeriod;  // Full period from zero crossings
  lastTuneResults.amplitude = avgAmplitude;
  lastTuneResults.peakCount = peaksUsed;
  lastTuneResults.stdDev = periodStdDev;
  lastTuneResults.valid = true;

  Serial.print(peaksUsed);
  Serial.print(F(","));
  Serial.print(periodCount);
  Serial.print(F(","));
  Serial.print(avgAmplitude, 0);
  Serial.print(F(","));
  Serial.print(avgPeriod, 2);
  Serial.print(F(","));
  Serial.print(lastTuneResults.Tu, 2);
  Serial.print(F(","));
  Serial.println(Ku, 2);
  
  if (periodStdDev > avgPeriod * 0.15 && avgPeriod > 0.01) {
  }

  // Validate Tu before calculating gains
  if (lastTuneResults.Tu < 0.01 || lastTuneResults.Tu > 10.0) {
    Serial.print(F("ERR Tu:"));
    Serial.println(lastTuneResults.Tu, 2);
    lastTuneResults.valid = false;
    return;
  }

  // Offer tuning options
  Serial.println(F("\nSelect 1-6:"));
  
  // Pre-compute suggested gains with safety checks
  float Tu = lastTuneResults.Tu;
  float kp1 = 0.3 * 0.6 * Ku;
  float ki1 = (Tu > 0.001) ? (0.3 * 1.2 * Ku / Tu) : 0.0;
  float kd1 = 0.3 * 0.075 * Ku * Tu;
  float kp2 = 0.6 * Ku;
  float ki2 = (Tu > 0.001) ? (1.2 * Ku / Tu) : 0.0;
  float kd2 = 0.075 * Ku * Tu;
  float kp3 = 0.8 * 0.6 * Ku;
  float ki3 = (Tu > 0.001) ? (0.8 * 1.2 * Ku / Tu) : 0.0;
  float kd3 = 0.8 * 0.075 * Ku * Tu;
  float kp4 = 0.45 * Ku;
  float ki4 = (Tu > 0.001) ? (kp4 * 2.2 / Tu) : 0.0;
  float kd4 = kp4 * (Tu / 6.3);
  float kp5 = 0.6 * Ku;
  float ki5 = 0.0;
  float kd5 = 0.125 * Ku * Tu;

  Serial.print(F("1:"));
  Serial.print(kp1, 3);
  Serial.print(F(","));
  Serial.print(ki1, 3);
  Serial.print(F(","));
  Serial.println(kd1, 3);
  Serial.print(F("2:"));
  Serial.print(kp2, 3);
  Serial.print(F(","));
  Serial.print(ki2, 3);
  Serial.print(F(","));
  Serial.println(kd2, 3);
  Serial.print(F("3:"));
  Serial.print(kp3, 3);
  Serial.print(F(","));
  Serial.print(ki3, 3);
  Serial.print(F(","));
  Serial.println(kd3, 3);
  Serial.print(F("4:"));
  Serial.print(kp4, 3);
  Serial.print(F(","));
  Serial.print(ki4, 3);
  Serial.print(F(","));
  Serial.println(kd4, 3);
  Serial.print(F("5:"));
  Serial.print(kp5, 3);
  Serial.print(F(","));
  Serial.print(ki5, 3);
  Serial.print(F(","));
  Serial.println(kd5, 3);
  Serial.println(F("6: Cancel"));

  // Flush any leftover input
  while (Serial.available()) { Serial.read(); }
  char selection = 0;
  while (selection == 0) {
    while (!Serial.available()) { }
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    selection = c;
  }

  float newKp, newKi, newKd;

  switch (selection) {
    case '1':  // Conservative
      newKp = 0.3 * 0.6 * Ku;
      newKi = (Tu > 0.001) ? (0.3 * 1.2 * Ku / Tu) : 0.0;
      newKd = 0.3 * 0.075 * Ku * Tu;
      break;

    case '2':  // Classic ZN
      newKp = 0.6 * Ku;
      newKi = (Tu > 0.001) ? (1.2 * Ku / Tu) : 0.0;
      newKd = 0.075 * Ku * Tu;
      break;

    case '3':  // Aggressive
      newKp = 0.8 * 0.6 * Ku;
      newKi = (Tu > 0.001) ? (0.8 * 1.2 * Ku / Tu) : 0.0;
      newKd = 0.8 * 0.075 * Ku * Tu;
      break;

    case '4': {  // Tyreus-Luyben
      newKp = 0.45 * Ku;
      newKi = (Tu > 0.001) ? (newKp * 2.2 / Tu) : 0.0;
      newKd = newKp * (Tu / 6.3);
      break;
    }

    case '5':  // PD-Only
      newKp = 0.6 * Ku;
      newKi = 0.0;
      newKd = 0.125 * Ku * Tu;
      break;

    default:
      Serial.println(F("Cancel"));
      return;
  }

  KP = newKp;
  KI = newKi;
  KD = newKd;

  Serial.print(F("P:"));
  Serial.print(KP, 4);
  Serial.print(F(" I:"));
  Serial.print(KI, 4);
  Serial.print(F(" D:"));
  Serial.println(KD, 4);
  saveCalibration();
}

// STEP RESPONSE

void stepResponse() {
  Serial.println(F("\nSTEP"));
  if (!isCalibrated) {
    Serial.println(F("ERR: Run R first"));
    return;
  }
  Serial.println(F("Y to start"));

  // Flush and wait for response
  while (Serial.available()) { Serial.read(); }
  char response = 0;
  while (response == 0) {
    while (!Serial.available()) { }
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    response = c;
  }

  if (toupper(response) != 'Y') {
    Serial.println(F("Cancel"));
    return;
  }

  // Move to a known starting position
  homeToLeft();
  delay(1000);

  Serial.println(F("T(ms),Pos,Vel"));

  unsigned long startTime = millis();
  long startPosition = motorEncoder.read();
  long lastPosition = startPosition;
  unsigned long lastTime = startTime;
  
  // Data for analysis
  const int MAX_SAMPLES = 60;  // Reduced for memory
  unsigned long times[MAX_SAMPLES];
  long positions[MAX_SAMPLES];
  int sampleCount = 0;

  setMotorVoltage(9.0);  // Updated to 9V as per UpdatedGameCodeNov17.ino

  // Record for 3 seconds or until right limit
  while (millis() - startTime < 3000 && digitalRead(LIMIT_RIGHT) == LOW && sampleCount < MAX_SAMPLES) {
    unsigned long elapsed = millis() - startTime;
    long position = motorEncoder.read();
    unsigned long currentTime = millis();
    
    // Calculate velocity
    float velocity = 0;
    if (currentTime > lastTime) {
      velocity = (float)(position - lastPosition) / ((currentTime - lastTime) / 1000.0);
    }
    
    if (sampleCount < MAX_SAMPLES) {
      times[sampleCount] = elapsed;
      positions[sampleCount] = position;
      sampleCount++;
    }

    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(position);
    Serial.print(",");
    Serial.println(velocity, 1);

    lastPosition = position;
    lastTime = currentTime;
    delay(30);  // Sample every 30ms
  }

  setMotorVoltage(0);

  long finalPosition = motorEncoder.read();
  long totalMovement = abs(finalPosition - startPosition);

  Serial.print(F("Move:"));
  Serial.println(totalMovement);
  long target10 = startPosition + (long)(totalMovement * 0.1);
  long target50 = startPosition + (long)(totalMovement * 0.5);
  long target90 = startPosition + (long)(totalMovement * 0.9);
  unsigned long time10 = 0, time50 = 0, time90 = 0;
  for (int i = 0; i < sampleCount; i++) {
    if (time10 == 0 && positions[i] >= target10) time10 = times[i];
    if (time50 == 0 && positions[i] >= target50) time50 = times[i];
    if (time90 == 0 && positions[i] >= target90) time90 = times[i];
  }
  if (time10 > 0 && time90 > 0) {
    Serial.print(F("Rise:"));
    Serial.println(time90 - time10);
  }
  if (time50 > 0) {
    Serial.print(F("T50:"));
    Serial.println(time50);
  }

  // Return home
  delay(1000);
  homeToLeft();
}

// ============================================================================
// MANUAL POSITION TEST - ENHANCED
// ============================================================================

void manualPositionTest() {
  Serial.println(F("\n=== MANUAL POSITION TEST ==="));

  if (!isCalibrated) {
    Serial.println(F("ERR: Run R first"));
    return;
  }

  Serial.println(F("Current PID gains:"));
  Serial.print(F("Kp = "));
  Serial.println(KP, 6);
  Serial.print(F("Ki = "));
  Serial.println(KI, 6);
  Serial.print(F("Kd = "));
  Serial.println(KD, 6);

  // Always home first to avoid drift
  homeToLeft();
  delay(300);
  positionFilterInitialized = false;

  // Always target Lane 3 for this test
  long targetPosition = LANE_POSITIONS[2];

  Serial.println(F("\nT(s),Pos,Err,I,V"));

  // Reset PID state
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
  lastDerivative = 0;

  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  unsigned long settledTime = 0;
  bool hasSettled = false;
  long maxOvershoot = 0;
  long initialError = abs(targetPosition - motorEncoder.read());

  // Clear any pending serial input
  while (Serial.available()) { Serial.read(); }

  while (millis() - startTime < 5000) {  // 5 second test
    // Update PID
    float voltage = updatePID(targetPosition);
    
    // Enhanced anti-windup
    if ((error != 0) && (error * lastError < 0)) {
      integral *= INTEGRAL_DECAY_CROSS;  // Soften windup on zero crossing
    }
    if (abs(error) > 400) {
      integral *= INTEGRAL_DECAY_FAR;  // Bleed integral when far
    }
    if (abs(error) > 600) {
      integral *= INTEGRAL_DECAY_FAR;  // Additional bleed when very far
    }
    
    float applied = cappedVoltageForError(voltage, error);
    setMotorVoltage(applied);

    // Track overshoot
    long currentError = abs(error);
    if (currentError > maxOvershoot && initialError > 0) {
      maxOvershoot = currentError;
    }

    // Print status
    if (millis() - lastPrint >= LOG_INTERVAL_MS) {
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
      Serial.println(applied, 3);

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

  Serial.print(F("Final:"));
  Serial.print(motorEncoder.read());
  Serial.print(F(" Err:"));
  Serial.println(error);
  if (hasSettled) {
    Serial.print(F("Settle:"));
    Serial.println((settledTime - startTime) / 1000.0, 1);
  }
  if (maxOvershoot > 0) {
    Serial.print(F("Overshoot:"));
    Serial.println(maxOvershoot);
  }
}

// MANUAL LANE MOVE

void manualLaneMove() {
  if (!isCalibrated) {
    Serial.println(F("ERR: Run R first"));
    return;
  }
  Serial.println(F("Lane 1-4:"));
  while (Serial.available()) Serial.read();
  int lane = -1;
  while (lane == -1) {
    while (!Serial.available()) { }
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ' || c == '\t') continue;
    if (c == '0') { lane = 0; }
    else if (c >= '1' && c <= '4') { lane = c - '0'; }
    else { lane = 0; }
    while (Serial.available()) Serial.read();
  }

  if (lane < 1 || lane > 4) return;
  long target = LANE_POSITIONS[lane - 1];

  homeToLeft();
  delay(300);
  positionFilterInitialized = false;

  manualMoveTo(target);
}

// MANUAL MOVE HELPER

void manualMoveTo(long targetPosition) {
  Serial.println(F("\nT(s),Pos,Err,I,V"));

  // Clear any pending serial input
  while (Serial.available()) { Serial.read(); }

  // Reset PID state
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
  lastDerivative = 0;

  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  unsigned long settledTime = 0;
  bool hasSettled = false;

  while (millis() - startTime < 5000) {  // 5 second window
    float voltage = updatePID(targetPosition);
    
    // Enhanced anti-windup
    if ((error != 0) && (error * lastError < 0)) {
      integral *= INTEGRAL_DECAY_CROSS;
    }
    if (abs(error) > 600) {
      integral *= INTEGRAL_DECAY_FAR;
    }
    
    float applied = cappedVoltageForError(voltage, error);
    setMotorVoltage(applied);

    if (millis() - lastPrint >= LOG_INTERVAL_MS) {
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
      Serial.println(applied, 3);

      lastPrint = millis();
    }

    if (abs(error) < DEADBAND && !hasSettled) {
      settledTime = millis();
      hasSettled = true;
    }

    if (Serial.available()) {
      Serial.read();
      break;
    }

    delay(CONTROL_PERIOD);
  }

  setMotorVoltage(0);

  Serial.print(motorEncoder.read());
  Serial.print(F(" "));
  Serial.println(error);
  if (hasSettled) {
    Serial.print(F("Settle:"));
    Serial.println((settledTime - startTime) / 1000.0, 1);
  }
}

// LANE TRANSITION TEST

void testLaneTransitions() {
  Serial.println(F("\nLANE TRANS TEST"));

  if (!isCalibrated) {
    Serial.println(F("ERR: Run R first"));
    return;
  }

  // Verify all lane positions are set
  bool allLanesSet = true;
  for (int i = 0; i < 4; i++) {
    if (LANE_POSITIONS[i] == 0 && i != 0) {
      allLanesSet = false;
    }
  }
  if (!allLanesSet) {
    Serial.println(F("ERROR: Set all lane positions first (commands 1-4)"));
    return;
  }

  Serial.println(F("Current PID gains:"));
  Serial.print(F("Kp="));
  Serial.print(KP, 4);
  Serial.print(F(" Ki="));
  Serial.print(KI, 4);
  Serial.print(F(" Kd="));
  Serial.println(KD, 4);
  Serial.println();

  // Home first
  homeToLeft();
  delay(500);
  positionFilterInitialized = false;

  // Test all transitions: 1->2, 1->3, 1->4, 2->1, 2->3, 2->4, 3->1, 3->2, 3->4, 4->1, 4->2, 4->3
  int transitions[12][2] = {
    {1, 2}, {1, 3}, {1, 4},
    {2, 1}, {2, 3}, {2, 4},
    {3, 1}, {3, 2}, {3, 4},
    {4, 1}, {4, 2}, {4, 3}
  };

  Serial.println(F("From,To,T(s),Ovr,Err,Set"));

  for (int t = 0; t < 12; t++) {
    int fromLane = transitions[t][0];
    int toLane = transitions[t][1];
    
    long startPos = LANE_POSITIONS[fromLane - 1];
    long targetPos = LANE_POSITIONS[toLane - 1];
    
    // Move to starting lane
    manualMoveToPosition(startPos, 3000);
    delay(500);
    
    // Reset PID state
    error = 0;
    lastError = 0;
    integral = 0;
    derivative = 0;
    lastDerivative = 0;
    positionFilterInitialized = false;
    
    // Perform transition
    unsigned long transStart = millis();
    unsigned long settledTime = 0;
    bool hasSettled = false;
    long maxOvershoot = 0;
    long initialError = abs(targetPos - motorEncoder.read());
    long maxError = initialError;
    
    unsigned long timeout = 8000;  // 8 second max per transition
    bool earlyStop = false;
    
    while (millis() - transStart < timeout) {
      float voltage = updatePID(targetPos);
      
      // Anti-windup
      if ((error != 0) && (error * lastError < 0)) {
        integral *= INTEGRAL_DECAY_CROSS;
      }
      if (abs(error) > 600) {
        integral *= INTEGRAL_DECAY_FAR;
      }
      
      float applied = cappedVoltageForError(voltage, error);
      setMotorVoltage(applied);
      
      // Track overshoot
      long currentError = abs(error);
      if (currentError > maxError) {
        maxError = currentError;
      }
      if (currentError > maxOvershoot && initialError > 0) {
        maxOvershoot = currentError;
      }
      
      // Check settled
      if (abs(error) < DEADBAND && !hasSettled) {
        settledTime = millis();
        hasSettled = true;
      }
      
      // Check for early stop
      if (Serial.available()) {
        Serial.read();
        earlyStop = true;
        break;
      }
      
      delay(CONTROL_PERIOD);
    }
    
    setMotorVoltage(0);
    
    float transTime = (millis() - transStart) / 1000.0;
    long finalError = abs(targetPos - motorEncoder.read());
    float settleTime = hasSettled ? (settledTime - transStart) / 1000.0 : -1.0;
    long overshoot = maxOvershoot > initialError ? (maxOvershoot - initialError) : 0;
    
    Serial.print(fromLane);
    Serial.print(F(","));
    Serial.print(toLane);
    Serial.print(F(","));
    Serial.print(transTime, 2);
    Serial.print(F(","));
    Serial.print(overshoot);
    Serial.print(F(","));
    Serial.print(finalError);
    Serial.print(F(","));
    if (hasSettled) {
      Serial.print(settleTime, 2);
    } else {
      Serial.print(F("NO"));
    }
    Serial.println();
    
    delay(500);
    
    if (earlyStop) {
      Serial.println(F("Stop"));
      break;
    }
  }
  
  setMotorVoltage(0);
}

void manualMoveToPosition(long targetPosition, unsigned long maxTime) {
  unsigned long startTime = millis();
  
  while (millis() - startTime < maxTime) {
    float voltage = updatePID(targetPosition);
    
    if ((error != 0) && (error * lastError < 0)) {
      integral *= INTEGRAL_DECAY_CROSS;
    }
    if (abs(error) > 600) {
      integral *= INTEGRAL_DECAY_FAR;
    }
    
    float applied = cappedVoltageForError(voltage, error);
    setMotorVoltage(applied);
    
    if (abs(error) < DEADBAND) {
      break;
    }
    
    if (Serial.available()) {
      Serial.read();
      break;
    }
    
    delay(CONTROL_PERIOD);
  }
  
  setMotorVoltage(0);
}

// UTILITY

void homeToLeft() {

  float driveVoltage = max(FRICTION_RIGHT + HOMING_EXTRA_VOLTAGE + 0.1, 3.0);
  setMotorVoltage(driveVoltage);

  unsigned long startTime = millis();
  long lastPosition = motorEncoder.read();
  unsigned long lastMoveTime = millis();
  
  while (digitalRead(LIMIT_LEFT) == LOW) {  // LOW = not pressed
    if (millis() - startTime > 12000) {
      Serial.println(F("ERR: timeout"));
      setMotorVoltage(0);
      return;
    }
    
    // Check for stuck condition
    long currentPos = motorEncoder.read();
    if (abs(currentPos - lastPosition) > 2) {
      lastMoveTime = millis();
      lastPosition = currentPos;
    } else if (millis() - lastMoveTime > 3000) {
      driveVoltage = min(driveVoltage + 0.5, 8.0);
      setMotorVoltage(driveVoltage);
      lastMoveTime = millis();
    }
    
    delay(10);
  }

  // Hold on the switch before zeroing; ensure it settles
  unsigned long holdStart = millis();
  float holdVoltage = max(FRICTION_RIGHT + 0.1, 2.5);
  long lastPos = motorEncoder.read();
  int stableTicks = 0;
  while (millis() - holdStart < HOMING_HOLD_TIME || stableTicks < HOMING_STABLE_TICKS) {
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
  setMotorVoltage(0);
  delay(200);

  motorEncoder.write(0);
  positionFilterInitialized = false;
}

void printStatus() {
  Serial.print(isCalibrated ? F("Y") : F("N"));
  Serial.print(F(" "));
  Serial.print(motorEncoder.read());
  Serial.print(F(" "));
  Serial.print(LEFT_LIMIT_POSITION);
  Serial.print(F("-"));
  Serial.print(RIGHT_LIMIT_POSITION);
  Serial.print(F(" P:"));
  Serial.print(KP, 2);
  Serial.print(F(" I:"));
  Serial.print(KI, 2);
  Serial.print(F(" D:"));
  Serial.print(KD, 2);
  Serial.print(F(" F:"));
  Serial.print(FRICTION_LEFT, 1);
  Serial.print(F("/"));
  Serial.print(FRICTION_RIGHT, 1);
  Serial.print(F(" L:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(LANE_POSITIONS[i]);
    if (i < 3) Serial.print(F(","));
  }
  if (lastTuneResults.valid) {
    Serial.print(F(" Ku:"));
    Serial.print(lastTuneResults.Ku, 1);
    Serial.print(F(" Tu:"));
    Serial.print(lastTuneResults.Tu, 1);
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("\nR F Z S T M X H P C 1-4 L W I ?"));
  Serial.println(F("I = Input PID values manually"));
}

void printWelcome() {
  Serial.println(F("\nPID TUNE"));
  if (isCalibrated) {
    Serial.println(F("Cal OK"));
  } else {
    Serial.println(F("Run R"));
  }
}

// MANUAL PID INPUT

void setPIDValues() {
  Serial.println(F("\n=== SET PID VALUES ==="));
  Serial.print(F("Current values: Kp="));
  Serial.print(KP, 6);
  Serial.print(F(" Ki="));
  Serial.print(KI, 6);
  Serial.print(F(" Kd="));
  Serial.println(KD, 6);
  Serial.println(F("\nEnter PID values as: Kp,Ki,Kd"));
  Serial.println(F("Example: 0.020,0.005,0.004"));
  Serial.print(F("> "));

  // Flush any leftover input
  while (Serial.available()) { Serial.read(); }

  // Wait for input with timeout
  unsigned long startTime = millis();
  String inputString = "";
  
  while (millis() - startTime < 30000) {  // 30 second timeout
    while (Serial.available()) {
      char c = Serial.read();
      
      if (c == '\n' || c == '\r') {
        // Process the input
        inputString.trim();
        if (inputString.length() > 0) {
          // Parse comma-separated values
          int comma1 = inputString.indexOf(',');
          int comma2 = inputString.indexOf(',', comma1 + 1);
          
          if (comma1 > 0 && comma2 > comma1) {
            String kpStr = inputString.substring(0, comma1);
            String kiStr = inputString.substring(comma1 + 1, comma2);
            String kdStr = inputString.substring(comma2 + 1);
            
            kpStr.trim();
            kiStr.trim();
            kdStr.trim();
            
            float newKp = kpStr.toFloat();
            float newKi = kiStr.toFloat();
            float newKd = kdStr.toFloat();
            
            // Validate values
            if (newKp >= 0 && newKp <= 10.0 &&
                newKi >= 0 && newKi <= 10.0 &&
                newKd >= 0 && newKd <= 10.0 &&
                (kpStr.length() > 0 && kiStr.length() > 0 && kdStr.length() > 0)) {
              
              KP = newKp;
              KI = newKi;
              KD = newKd;
              
              Serial.print(F("\nNew values: Kp="));
              Serial.print(KP, 6);
              Serial.print(F(" Ki="));
              Serial.print(KI, 6);
              Serial.print(F(" Kd="));
              Serial.println(KD, 6);
              
              // Save to EEPROM
              saveCalibration();
              Serial.println(F("Saved to EEPROM"));
              return;
            } else {
              Serial.println(F("ERROR: Invalid values. Must be 0-10.0"));
              Serial.print(F("> "));
              inputString = "";
              continue;
            }
          } else {
            Serial.println(F("ERROR: Invalid format. Use: Kp,Ki,Kd"));
            Serial.print(F("> "));
            inputString = "";
            continue;
          }
        }
      } else if (c == 8 || c == 127) {  // Backspace/Delete
        if (inputString.length() > 0) {
          inputString.remove(inputString.length() - 1);
          Serial.print(c);  // Echo backspace
        }
      } else if (c >= 32 && c <= 126) {  // Printable ASCII
        inputString += c;
        Serial.print(c);  // Echo character
      }
    }
    delay(10);
  }
  
  Serial.println(F("\nTimeout - no input received"));
}

// EEPROM

void saveCalibration() {
  EEPROM.write(EEPROM_CALIBRATION_FLAG, 0xAA);  // Magic number
  EEPROM.put(EEPROM_KP, KP);
  EEPROM.put(EEPROM_KI, KI);
  EEPROM.put(EEPROM_KD, KD);
  EEPROM.put(EEPROM_FRICTION_LEFT, FRICTION_LEFT);
  EEPROM.put(EEPROM_FRICTION_RIGHT, FRICTION_RIGHT);
  EEPROM.put(EEPROM_LEFT_LIMIT, LEFT_LIMIT_POSITION);
  EEPROM.put(EEPROM_RIGHT_LIMIT, RIGHT_LIMIT_POSITION);
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_LANES_BASE + i * sizeof(long), LANE_POSITIONS[i]);
  }

  Serial.println(F("Saved"));
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
    for (int i = 0; i < 4; i++) {
      long v;
      EEPROM.get(EEPROM_LANES_BASE + i * sizeof(long), v);
      LANE_POSITIONS[i] = v;
    }

    TOTAL_RANGE = abs(RIGHT_LIMIT_POSITION - LEFT_LIMIT_POSITION);
    isCalibrated = (TOTAL_RANGE > 100);
    frictionCharacterized = (FRICTION_LEFT > 0.1 && FRICTION_RIGHT > 0.1);
  } else {
    frictionCharacterized = false;
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
  frictionCharacterized = false;

  Serial.println(F("Cleared"));
}

float cappedVoltageForError(float voltage, long error) {
  long absErr = abs(error);
  float cap;
  // Updated voltage caps to match 9V nominal (as per UpdatedGameCodeNov17.ino)
  if (absErr > 1000) cap = 9.0f;  // Very large moves - use full 9V
  else if (absErr > 800) cap = 8.5f;  // Large moves - high speed
  else if (absErr > 500) cap = 8.0f;  // Medium-large moves
  else if (absErr > 300) cap = 7.5f;  // Medium moves
  else if (absErr > 100) cap = 7.0f;  // Small-medium moves
  else if (absErr > 50) cap = 6.0f;  // Small moves
  else cap = 5.0f;  // Fine positioning
  cap = min(cap, TEST_MAX_VOLTAGE);
  return constrain(voltage, -cap, cap);
}
