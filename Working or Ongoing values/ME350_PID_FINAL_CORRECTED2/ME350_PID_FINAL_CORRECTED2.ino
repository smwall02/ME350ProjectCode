// ME 350 Lab 10 - PID Control FINAL
// Team 25 - Corrected based on working test code

#include <Encoder.h>

// ============================================
// PIN DEFINITIONS
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
// CALIBRATION PARAMETERS
// ============================================
long TARGET_1_POSITION = -64;
long TARGET_2_POSITION = -330;
long TARGET_3_POSITION = -593;
long TARGET_4_POSITION = -1257;

float FRICTION_COMP_VOLTAGE = 4.0;

// ============================================
// PID GAINS
// ============================================
float KP = 0.020;
float KI = 0.005;
float KD = 0.004;

// ============================================
// CONTROL PARAMETERS
// ============================================
const float MAX_VOLTAGE = 10.0;
const float MIN_CONTROL_VOLTAGE = 3.0;
const int DEADBAND = 15;
const float MAX_INTEGRAL = 500.0;
const unsigned long CONTROL_PERIOD = 10;

// ============================================
// GLOBAL VARIABLES
// ============================================
long targetPosition = 0;
float errorIntegral = 0;
float lastError = 0;
unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;

bool systemEnabled = false;

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
    
    if (currentTime - lastPrintTime >= 500) {
      lastPrintTime = currentTime;
      long currentPos = encoder.read();
      long error = targetPosition - currentPos;
      
      if (abs(error) > DEADBAND) {
        Serial.print(F("Pos: "));
        Serial.print(currentPos);
        Serial.print(F(" → "));
        Serial.print(targetPosition);
        Serial.print(F(" (Δ="));
        Serial.print(error);
        Serial.println(F(")"));
      }
    }
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
  
  float frictionComp = 0;
  if (abs(error) > DEADBAND) {
    if (error < 0) {
      // Need to move RIGHT (to more negative position)
      frictionComp = -FRICTION_COMP_VOLTAGE;
    } else {
      // Need to move LEFT (to less negative position)
      frictionComp = FRICTION_COMP_VOLTAGE;
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
// MOTOR CONTROL (FROM WORKING CODE)
// ============================================
void setMotor(float voltage) {
  voltage = constrain(voltage, -10.0, 10.0);
  int pwm = abs(voltage) * 25.5;
  
  if (voltage > 0) {
    // Positive = LEFT
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else if (voltage < 0) {
    // Negative = RIGHT
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
// LIMIT SWITCHES
// ============================================
void checkLimitSwitches() {
  if (digitalRead(LIMIT_LEFT) == LOW) {
    stopMotor();
    encoder.write(0);
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
  Serial.println(F("\n🏠 HOMING..."));
  
  if (leftPressed()) {
    encoder.write(0);
    Serial.println(F("✓ Already home\n"));
    return true;
  }
  
  unsigned long startTime = millis();
  setMotor(5.0);  // Positive voltage = move LEFT
  
  while (!leftPressed() && (millis() - startTime) < 15000) {
    delay(10);
  }
  
  stopMotor();
  
  if (leftPressed()) {
    encoder.write(0);
    Serial.println(F("✓ Homed\n"));
    delay(500);
    return true;
  } else {
    Serial.println(F("✗ Timeout\n"));
    return false;
  }
}

// ============================================
// AUTO-TUNING
// ============================================
void autoTunePID() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   AUTO-TUNE                                ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  Serial.println(F("▶ Press ENTER..."));
  
  while (!Serial.available()) {}
  char response = Serial.read();
  while (Serial.available()) Serial.read();
  
  if (response != '\n' && response != '\r') {
    Serial.println(F("\n✗ Cancelled\n"));
    return;
  }
  
  bool wasEnabled = systemEnabled;
  systemEnabled = false;
  stopMotor();
  delay(500);
  
  // Home
  if (!homeToLeftLimit()) {
    systemEnabled = wasEnabled;
    return;
  }
  
  // Find range
  Serial.println(F("Finding range..."));
  
  long leftPos = encoder.read();
  Serial.print(F("Left: "));
  Serial.println(leftPos);
  
  setMotor(-5.0);  // Negative voltage = move RIGHT
  
  unsigned long moveTimeout = millis();
  while (!rightPressed() && (millis() - moveTimeout) < 15000) {
    delay(10);
  }
  
  stopMotor();
  delay(500);
  
  long rightPos = encoder.read();
  long fullRange = abs(rightPos - leftPos);
  
  Serial.print(F("Right: "));
  Serial.println(rightPos);
  Serial.print(F("Range: "));
  Serial.print(fullRange);
  Serial.println(F(" counts\n"));
  
  if (fullRange < 100) {
    Serial.println(F("✗ Range too small!"));
    systemEnabled = wasEnabled;
    return;
  }
  
  // Move to center
  Serial.println(F("Moving to center..."));
  
  long centerPosition = (leftPos + rightPos) / 2;
  Serial.print(F("Target: "));
  Serial.println(centerPosition);
  
  moveTimeout = millis();
  while (abs(encoder.read() - centerPosition) > 50 && (millis() - moveTimeout) < 15000) {
    long pos = encoder.read();
    long error = centerPosition - pos;
    
    // Error is negative = need to move RIGHT (more negative)
    // Error is positive = need to move LEFT (less negative)
    if (error < 0) {
      setMotor(-5.0);  // Move RIGHT
    } else {
      setMotor(5.0);   // Move LEFT
    }
    
    if ((millis() % 500) < 20) {
      Serial.print(F("  Pos: "));
      Serial.print(pos);
      Serial.print(F(" Error: "));
      Serial.println(error);
      delay(20);
    }
    
    delay(10);
  }
  
  stopMotor();
  delay(1500);
  
  long actualCenter = encoder.read();
  Serial.print(F("✓ At: "));
  Serial.println(actualCenter);
  Serial.println();
  
  // Oscillations
  Serial.println(F("Starting oscillations...\n"));
  
  int HYSTERESIS = fullRange / 6;
  const float TEST_VOLTAGE = 5.0;
  const int MAX_PEAKS = 20;
  const int REQUIRED_PEAKS = 16;
  const unsigned long TIMEOUT = 50000;
  
  Serial.print(F("Hysteresis: ±"));
  Serial.print(HYSTERESIS);
  Serial.println(F(" counts"));
  Serial.println();
  
  // First, move away from center to start oscillation
  Serial.println(F("Moving RIGHT to start oscillation..."));
  setMotor(-TEST_VOLTAGE);  // Move RIGHT
  delay(1000);  // Move for 1 second
  stopMotor();
  delay(500);
  
  Serial.print(F("Starting pos: "));
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
    
    // If too far RIGHT (more negative), move LEFT (positive voltage)
    // If too far LEFT (less negative), move RIGHT (negative voltage)
    if (distanceFromCenter < -HYSTERESIS) {
      setMotor(TEST_VOLTAGE);   // Move LEFT
      if (crossedCenter) {
        crossedCenter = false;  // Reset for next crossing
      }
    } else if (distanceFromCenter > HYSTERESIS) {
      setMotor(-TEST_VOLTAGE);  // Move RIGHT
      if (crossedCenter) {
        crossedCenter = false;  // Reset for next crossing
      }
    }
    
    // Detect peak when crossing center
    if (!crossedCenter && abs(distanceFromCenter) < 50) {
      crossedCenter = true;
      
      unsigned long now = millis();
      peakPositions[peakCount] = lastPos;
      peakTimes[peakCount] = now;
      
      Serial.print(F("Peak "));
      Serial.print(peakCount + 1);
      Serial.print(F(": "));
      Serial.print(lastPos);
      Serial.print(F(" (t="));
      Serial.print((now - startTime) / 1000.0, 2);
      Serial.println(F("s)"));
      
      peakCount++;
    }
    
    lastPos = currentPos;
    delay(5);
    
    if (Serial.available()) {
      Serial.println(F("\n✗ Aborted"));
      stopMotor();
      systemEnabled = wasEnabled;
      return;
    }
  }
  
  stopMotor();
  delay(500);
  
  if (peakCount < REQUIRED_PEAKS) {
    Serial.println(F("\n✗ FAILED"));
    Serial.print(F("Got "));
    Serial.print(peakCount);
    Serial.print(F("/"));
    Serial.println(REQUIRED_PEAKS);
    systemEnabled = wasEnabled;
    return;
  }
  
  Serial.println(F("\n✓ Complete!\n"));
  
  // Analysis
  const int SKIP = 4;
  
  long minPos = peakPositions[SKIP];
  long maxPos = peakPositions[SKIP];
  
  for (int i = SKIP; i < peakCount; i++) {
    if (peakPositions[i] < minPos) minPos = peakPositions[i];
    if (peakPositions[i] > maxPos) maxPos = peakPositions[i];
  }
  
  float amplitude = abs(maxPos - minPos) / 2.0;
  
  float totalPeriod = 0;
  int periodCount = 0;
  
  for (int i = SKIP + 2; i < peakCount; i += 2) {
    float period = (peakTimes[i] - peakTimes[i-2]) / 1000.0;
    totalPeriod += period;
    periodCount++;
  }
  
  float avgPeriod = totalPeriod / periodCount;
  float Ku = (4.0 * TEST_VOLTAGE) / (3.14159 * amplitude);
  
  Serial.println(F("RESULTS:"));
  Serial.print(F("  Amplitude: "));
  Serial.print(amplitude, 1);
  Serial.println(F(" counts"));
  Serial.print(F("  Period:    "));
  Serial.print(avgPeriod, 3);
  Serial.println(F(" s"));
  Serial.print(F("  Ku:        "));
  Serial.println(Ku, 4);
  Serial.println();
  
  // Calculate
  const float SAFETY = 0.35;
  
  float newKp = SAFETY * 0.6 * Ku;
  float newKi = SAFETY * 1.2 * Ku / avgPeriod;
  float newKd = SAFETY * 0.075 * Ku * avgPeriod;
  
  Serial.println(F("GAINS:"));
  Serial.print(F("  Kp = "));
  Serial.println(newKp, 4);
  Serial.print(F("  Ki = "));
  Serial.println(newKi, 4);
  Serial.print(F("  Kd = "));
  Serial.println(newKd, 4);
  Serial.println();
  
  Serial.println(F("Apply? (Y/N)"));
  while (!Serial.available()) {}
  response = Serial.read();
  while (Serial.available()) Serial.read();
  
  if (response == 'Y' || response == 'y') {
    KP = newKp;
    KI = newKi;
    KD = newKd;
    
    Serial.println(F("\n✓ Applied!\n"));
    
    homeToLeftLimit();
    
    Serial.println(F("Ready! Try 1-4\n"));
    
    systemEnabled = wasEnabled;
  } else {
    Serial.println(F("\n✗ Not applied\n"));
    systemEnabled = wasEnabled;
  }
}

// ============================================
// COMMANDS
// ============================================
void processCommand() {
  char cmd = Serial.read();
  while (Serial.available()) Serial.read();
  
  cmd = toupper(cmd);
  
  switch (cmd) {
    case '1':
      setTargetLane(1);
      systemEnabled = true;
      lastPrintTime = 0;
      break;
      
    case '2':
      setTargetLane(2);
      systemEnabled = true;
      lastPrintTime = 0;
      break;
      
    case '3':
      setTargetLane(3);
      systemEnabled = true;
      lastPrintTime = 0;
      break;
      
    case '4':
      setTargetLane(4);
      systemEnabled = true;
      lastPrintTime = 0;
      break;
      
    case 'S':
      Serial.println(F("\n⏹ STOP\n"));
      systemEnabled = false;
      stopMotor();
      errorIntegral = 0;
      break;
      
    case 'P':
      printStatus();
      break;
      
    case 'H':
      printHelp();
      break;
      
    case 'A':
      autoTunePID();
      break;
      
    case 'Z':
      homeToLeftLimit();
      break;
      
    default:
      break;
  }
}

void setTargetLane(int lane) {
  switch (lane) {
    case 1: targetPosition = TARGET_1_POSITION; break;
    case 2: targetPosition = TARGET_2_POSITION; break;
    case 3: targetPosition = TARGET_3_POSITION; break;
    case 4: targetPosition = TARGET_4_POSITION; break;
    default: return;
  }
  
  errorIntegral = 0;
  lastError = 0;
  
  long currentPos = encoder.read();
  long error = targetPosition - currentPos;
  
  Serial.print(F("\n→ Lane "));
  Serial.print(lane);
  Serial.print(F(" | "));
  Serial.print(currentPos);
  Serial.print(F(" → "));
  Serial.print(targetPosition);
  Serial.print(F(" (Δ="));
  Serial.print(error);
  Serial.println(F(")"));
}

void printWelcome() {
  Serial.println(F("\n\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║   ME 350 Lab 10 - PID FINAL                ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("COMMANDS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("  1-4    Move to lane"));
  Serial.println(F("  S      Stop"));
  Serial.println(F("  P      Status"));
  Serial.println(F("  A      Auto-tune"));
  Serial.println(F("  Z      Home"));
  Serial.println(F("  H      Help"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}

void printStatus() {
  long currentPos = encoder.read();
  float error = targetPosition - currentPos;
  
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("STATUS"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.print(F("Position:  "));
  Serial.println(currentPos);
  Serial.print(F("Target:    "));
  Serial.println(targetPosition);
  Serial.print(F("Error:     "));
  Serial.println((int)error);
  Serial.print(F("Enabled:   "));
  Serial.println(systemEnabled ? F("YES") : F("NO"));
  Serial.println(F("\n━━ PID ━━"));
  Serial.print(F("Kp = "));
  Serial.println(KP, 4);
  Serial.print(F("Ki = "));
  Serial.println(KI, 4);
  Serial.print(F("Kd = "));
  Serial.println(KD, 4);
  Serial.println(F("\n━━ LANES ━━"));
  Serial.print(F("1: "));
  Serial.println(TARGET_1_POSITION);
  Serial.print(F("2: "));
  Serial.println(TARGET_2_POSITION);
  Serial.print(F("3: "));
  Serial.println(TARGET_3_POSITION);
  Serial.print(F("4: "));
  Serial.println(TARGET_4_POSITION);
  Serial.print(F("\nLimit L: "));
  Serial.print(leftPressed() ? "PRESSED" : "open");
  Serial.print(F("  R: "));
  Serial.println(rightPressed() ? "PRESSED" : "open");
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"));
}
