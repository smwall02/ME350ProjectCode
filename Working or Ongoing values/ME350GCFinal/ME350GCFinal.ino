// ME350 Zombie Defense - FIXED VERSION
// Shorter dwells + emergency targeting for high-threat zombies

#include <Encoder.h>
#include <EEPROM.h>

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
const int EEPROM_LANES_BASE = 29;

//============================================
// STATE MACHINE
//============================================
enum State {
  CALIBRATE = 1,
  CHOOSE_TARGET = 2,
  MOVE_TO_TARGET = 3,
  DWELL_AT_TARGET = 4
};

State state = CALIBRATE;

// Direction constants
const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

//============================================
// TARGET POSITIONS
//============================================
long targetPositions[4] = {-73, -341, -594, -1200};
const long WAIT_POSITION = -594;
const int TARGET_BAND = 20;  // Increased for more forgiving positioning

//============================================
// SENSOR CALIBRATION RANGES
//============================================
int ProxRange[4][2] = {
  {615, 88},
  {634, 124},
  {622, 147},
  {590, 80}
};

//============================================
// PROXIMITY SENSORS
//============================================
struct ProxSensor {
  float currVal;
  float prevVal;
  unsigned long prevChangeTime;
  int pin;
  int direction;
  int forwardCount;
  int backwardCount;
};

ProxSensor ProxSensors[4];

const float alpha = 0.925;
const int stopTimeout = 150;
int noiseLimit = 10;
const int lowerNoiseLimit = 8;
const int upperNoiseLimit = 12;
const int noiseThreshold = 225;

//============================================
// GAME STATE
//============================================
bool gameOver = false;

//============================================
// THREAT LEVELS
//============================================
const float CRITICAL_THREAT = 0.10;  // <10% distance = CRITICAL (90%+ danger)
const float MAX_ENGAGE = 0.12;       // <12% distance = too close (88%+ danger)
const float MIN_ENGAGE = 0.20;       // >20% distance = too far (<80% danger)

//============================================
// TARGET TRACKING
//============================================
int activeTargetIndex = -1;
long desiredPosition = WAIT_POSITION;
float zombieDistances[4];
bool WAIT_POS = true;

// Dwell timing
unsigned long arrivalTime = 0;
float peakZombieDistance = 1.0;
const unsigned long MIN_DWELL_TIME = 500;    // Minimum 500ms - wait for light detection
const unsigned long MAX_DWELL_TIME = 1200;   // Maximum 1200ms
const unsigned long BACKWARD_CONFIRM = 200;  // 200ms backward - ensure it's actually moving back

//============================================
// PID CONTROLLER
//============================================
float KP = 0.015;
float KI = 0.003;
float KD = 0.020;
float FRICTION_LEFT = 0.25;
float FRICTION_RIGHT = 0.25;

float SUPPLY_VOLTAGE = 12.0;
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
  Serial.println(F("ME350 Zombie Defense - EMERGENCY FIX"));
  Serial.println(F("===================================="));
  
  loadPIDFromEEPROM();
  loadLanePositionsFromEEPROM();
  
  ProxSensors[0].pin = PROX_SENSOR_1;
  ProxSensors[1].pin = PROX_SENSOR_2;
  ProxSensors[2].pin = PROX_SENSOR_3;
  ProxSensors[3].pin = PROX_SENSOR_4;
  
  for (int i = 0; i < 4; i++) {
    ProxSensors[i].currVal = analogRead(ProxSensors[i].pin);
    ProxSensors[i].prevVal = ProxSensors[i].currVal;
    ProxSensors[i].prevChangeTime = millis();
    ProxSensors[i].direction = STOPPED;
    ProxSensors[i].forwardCount = 0;
    ProxSensors[i].backwardCount = 0;
  }
  
  stopMotor();
  
  Serial.println(F("Commands:"));
  Serial.println(F("  G - Start auto"));
  Serial.println(F("  S - Stop"));
  Serial.println(F("  H - Home"));
  Serial.println(F("  1/2/3/4 - Move to lane"));
  Serial.println(F("  C1/C2/C3/C4 - Capture lane position"));
  Serial.println(F("  P - Print positions"));
  Serial.println(F("  W - Write to EEPROM"));
  Serial.println(F("  V<value> - Set voltage (e.g. V12, V9)"));
  Serial.println();
  Serial.print(F("PID: KP="));
  Serial.print(KP, 4);
  Serial.print(F(" KI="));
  Serial.print(KI, 4);
  Serial.print(F(" KD="));
  Serial.println(KD, 4);
  Serial.print(F("Lanes: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(targetPositions[i]);
    Serial.print(F(" "));
  }
  Serial.println();
}

//============================================
// MAIN LOOP
//============================================
void loop() {
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();
  
  // Process serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'G' || cmd == 'g') {
      Serial.println(F("AUTO MODE START"));
      autoMode = true;
      systemEnabled = true;
      gameOver = false;
      state = CHOOSE_TARGET;
      
    } else if (cmd == 'S' || cmd == 's') {
      Serial.println(F("STOP"));
      autoMode = false;
      systemEnabled = false;
      stopMotor();
      
    } else if (cmd == 'H' || cmd == 'h') {
      Serial.println(F("HOMING..."));
      homeToLeftLimit();
      
    } else if (cmd == '1' || cmd == '2' || cmd == '3' || cmd == '4') {
      int lane = cmd - '1';  // Convert '1'-'4' to 0-3
      Serial.print(F("Moving to L"));
      Serial.print(lane + 1);
      Serial.print(F(" ("));
      Serial.print(targetPositions[lane]);
      Serial.println(F(")"));
      desiredPosition = targetPositions[lane];
      autoMode = false;
      systemEnabled = true;
      
    } else if (cmd == 'C') {
      // Wait for lane number
      while (!Serial.available()) delay(10);
      char laneChar = Serial.read();
      if (laneChar >= '1' && laneChar <= '4') {
        int lane = laneChar - '1';
        long pos = encoder.read();
        targetPositions[lane] = pos;
        Serial.print(F("Captured L"));
        Serial.print(lane + 1);
        Serial.print(F(" = "));
        Serial.println(pos);
      }
      
    } else if (cmd == 'P' || cmd == 'p') {
      Serial.println(F("Lane positions:"));
      for (int i = 0; i < 4; i++) {
        Serial.print(F("  L"));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.println(targetPositions[i]);
      }
      
    } else if (cmd == 'W' || cmd == 'w') {
      Serial.println(F("Saving to EEPROM..."));
      for (int i = 0; i < 4; i++) {
        EEPROM.put(EEPROM_LANES_BASE + i * 4, targetPositions[i]);
      }
      Serial.println(F("Saved!"));
      
    } else if (cmd == 'V' || cmd == 'v') {
      // Read voltage value from serial
      float newVoltage = Serial.parseFloat();
      if (newVoltage > 0) {
        SUPPLY_VOLTAGE = newVoltage;
        Serial.print(F("Motor voltage set to "));
        Serial.print(SUPPLY_VOLTAGE, 1);
        Serial.println(F("V"));
      } else {
        Serial.print(F("Current voltage: "));
        Serial.print(SUPPLY_VOLTAGE, 1);
        Serial.println(F("V"));
      }
    }
  }
  
  computeVelocity();
  updateSensors();
  
  // Calculate zombie distances
  for (int i = 0; i < 4; i++) {
    zombieDistances[i] = (ProxSensors[i].currVal - ProxRange[i][1]) / 
                          (float)(ProxRange[i][0] - ProxRange[i][1]);
    zombieDistances[i] = constrain(zombieDistances[i], 0.0, 1.0);
  }
  
  //============================================
  // STATE MACHINE
  //============================================
  if (autoMode && systemEnabled && !gameOver) {
    
    // EMERGENCY: Check for critical threats (90%+ zombies)
    bool emergencyTriggered = checkEmergencyThreats();
    
    if (!emergencyTriggered) {
      switch (state) {
        case CALIBRATE:
          break;
        
        case CHOOSE_TARGET:
          chooseTarget();
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
      Serial.println(F("Switch OFF"));
    }
  }
  
  //============================================
  // STATUS OUTPUT
  //============================================
  if (autoMode && (millis() - lastPrintTime >= 200)) {
    lastPrintTime = millis();
    printStatus();
  }
}

//============================================
// EMERGENCY THREAT CHECK
//============================================
bool checkEmergencyThreats() {
  // If we're moving to or dwelling at a target, only EXTREME emergencies can interrupt
  bool committed = (state == MOVE_TO_TARGET || state == DWELL_AT_TARGET);
  float emergencyThreshold = committed ? 0.03 : CRITICAL_THREAT;  // 3% if committed, 10% otherwise
  
  int criticalLane = -1;
  float lowestDist = 2.0;  // Lowest distance remaining = highest danger
  
  for (int i = 0; i < 4; i++) {
    if (ProxSensors[i].direction == FORWARD) {
      // Apply lane weighting (L2 and L3 are shorter)
      float effectiveDist = zombieDistances[i];
      if (i == 1 || i == 2) {  // Lanes 2 and 3
        effectiveDist *= 0.6;  // Make them appear 40% closer
      }
      
      if (effectiveDist < emergencyThreshold) {
        if (effectiveDist < lowestDist) {
          lowestDist = effectiveDist;
          criticalLane = i;
        }
      }
    }
  }
  
  if (criticalLane >= 0) {
    if (activeTargetIndex != criticalLane) {
      Serial.print(F("!!! EMERGENCY L"));
      Serial.print(criticalLane + 1);
      Serial.print(F(" @"));
      Serial.print((int)((1.0 - lowestDist) * 100));
      Serial.println(F("% !!!"));
      
      activeTargetIndex = criticalLane;
      desiredPosition = targetPositions[criticalLane];
      WAIT_POS = false;
      state = MOVE_TO_TARGET;
    }
    return true;
  }
  
  return false;
}

//============================================
// CHOOSE TARGET
//============================================
void chooseTarget() {
  int bestLane = -1;
  float bestDist = 2.0;
  
  // Engage zombies between MAX_ENGAGE and MIN_ENGAGE distance remaining
  // Priority: L2&3 > L1&4
  
  // Pass 1: Priority lanes (2&3)
  for (int i = 1; i <= 2; i++) {
    if (ProxSensors[i].direction == FORWARD &&
        zombieDistances[i] >= MAX_ENGAGE &&
        zombieDistances[i] <= MIN_ENGAGE) {
      if (zombieDistances[i] < bestDist) {
        bestDist = zombieDistances[i];
        bestLane = i;
      }
    }
  }
  
  // Pass 2: Regular lanes (1&4)
  if (bestLane < 0) {
    for (int i = 0; i < 4; i++) {
      if ((i == 0 || i == 3) &&
          ProxSensors[i].direction == FORWARD &&
          zombieDistances[i] >= MAX_ENGAGE &&
          zombieDistances[i] <= MIN_ENGAGE) {
        if (zombieDistances[i] < bestDist) {
          bestDist = zombieDistances[i];
          bestLane = i;
        }
      }
    }
  }
  
  // Pass 3: FALLBACK - No targets in engagement window, find most advanced zombie
  if (bestLane < 0) {
    float lowestDist = 2.0;
    int candidateLanes[4];
    int candidateCount = 0;
    
    // Find the lowest distance (most advanced)
    for (int i = 0; i < 4; i++) {
      if (ProxSensors[i].direction == FORWARD) {
        float effectiveDist = zombieDistances[i];
        if (i == 1 || i == 2) {
          effectiveDist *= 0.6;  // Apply L2&3 weighting
        }
        
        if (effectiveDist < lowestDist) {
          lowestDist = effectiveDist;
        }
      }
    }
    
    // Find all lanes tied for most advanced
    for (int i = 0; i < 4; i++) {
      if (ProxSensors[i].direction == FORWARD) {
        float effectiveDist = zombieDistances[i];
        if (i == 1 || i == 2) {
          effectiveDist *= 0.6;
        }
        
        if (abs(effectiveDist - lowestDist) < 0.01) {  // Within 1% = tied
          candidateLanes[candidateCount++] = i;
        }
      }
    }
    
    // Pick randomly among tied candidates
    if (candidateCount > 0) {
      bestLane = candidateLanes[random(candidateCount)];
    }
  }
  
  if (bestLane >= 0) {
    activeTargetIndex = bestLane;
    desiredPosition = targetPositions[bestLane];
    WAIT_POS = false;
    
    Serial.print(F(">> L"));
    Serial.print(bestLane + 1);
    Serial.print(F(" @"));
    Serial.print((int)((1.0 - zombieDistances[bestLane]) * 100));
    Serial.print(F("%"));
    if (bestLane == 1 || bestLane == 2) Serial.print(F(" [PRI]"));
    if (bestDist >= 2.0) Serial.print(F(" [FALLBACK]"));  // Was in fallback mode
    Serial.println();
    
    state = MOVE_TO_TARGET;
  } else {
    desiredPosition = WAIT_POSITION;
    WAIT_POS = true;
    activeTargetIndex = -1;
  }
}

//============================================
// MOVE TO TARGET
//============================================
void moveToTarget() {
  long currentPos = encoder.read();
  
  if (abs(currentPos - desiredPosition) <= TARGET_BAND) {
    arrivalTime = millis();
    peakZombieDistance = zombieDistances[activeTargetIndex];
    
    if (WAIT_POS) {
      state = CHOOSE_TARGET;
    } else {
      state = DWELL_AT_TARGET;
    }
  }
}

//============================================
// DWELL AT TARGET
//============================================
void dwellAtTarget() {
  unsigned long dwellTime = millis() - arrivalTime;
  float currentDist = zombieDistances[activeTargetIndex];
  int currentDir = ProxSensors[activeTargetIndex].direction;
  
  // Lane 4 gets extra time
  unsigned long maxDwell = (activeTargetIndex == 3) ? 1800 : MAX_DWELL_TIME;
  
  // Update peak
  if (currentDist < peakZombieDistance) {
    peakZombieDistance = currentDist;
  }
  
  // Exit 1: Backward detection (200ms backward AFTER min dwell)
  static unsigned long backwardStart = 0;
  if (currentDir == BACKWARD) {
    if (backwardStart == 0) backwardStart = millis();
    if (millis() - backwardStart >= BACKWARD_CONFIRM && 
        dwellTime >= MIN_DWELL_TIME) {
      Serial.print(F("HIT L"));
      Serial.println(activeTargetIndex + 1);
      backwardStart = 0;
      state = CHOOSE_TARGET;
      return;
    }
  } else {
    backwardStart = 0;
  }
  
  // Exit 2: Stopped (600ms stopped)
  if (currentDir == STOPPED && dwellTime >= 600) {
    Serial.print(F("STOPPED L"));
    Serial.println(activeTargetIndex + 1);
    state = CHOOSE_TARGET;
    return;
  }
  
  // Exit 3: Timeout (1200ms or 1800ms for lane 4)
  if (dwellTime >= maxDwell) {
    Serial.print(F("TIMEOUT L"));
    Serial.println(activeTargetIndex + 1);
    state = CHOOSE_TARGET;
    return;
  }
}

//============================================
// UPDATE SENSORS
//============================================
void updateSensors() {
  for (int i = 0; i < 4; i++) {
    int rawVal = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal + (1.0 - alpha) * rawVal;
    
    if (ProxSensors[i].currVal >= noiseThreshold) {
      noiseLimit = upperNoiseLimit;
    } else {
      noiseLimit = lowerNoiseLimit;
    }
    
    if (abs(ProxSensors[i].currVal - ProxSensors[i].prevVal) < noiseLimit) {
      if (millis() - ProxSensors[i].prevChangeTime >= stopTimeout) {
        ProxSensors[i].direction = STOPPED;
      }
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].backwardCount = 0;
    } else if (ProxSensors[i].currVal < ProxSensors[i].prevVal) {
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;
      if (ProxSensors[i].forwardCount > 3) {
        ProxSensors[i].direction = FORWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = millis();
      }
    } else {
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      if (ProxSensors[i].backwardCount > 3) {
        ProxSensors[i].direction = BACKWARD;
        ProxSensors[i].prevVal = ProxSensors[i].currVal;
        ProxSensors[i].prevChangeTime = millis();
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
  
  if (abs(currentPos - previousMotorPosition) > 2 || 
      (currentTime - previousVelCompTime) > 10000) {
    motorVelocity = (double)(currentPos - previousMotorPosition) * 1000000.0 / 
                    (currentTime - previousVelCompTime);
    previousMotorPosition = currentPos;
    previousVelCompTime = currentTime;
  }
}

void runPIDController() {
  long currentPos = encoder.read();
  float positionError = desiredPosition - currentPos;
  
  // Lane 4 gets tighter tolerance for better accuracy
  int effectiveBand = (activeTargetIndex == 3) ? 12 : TARGET_BAND;
  int effectiveVelThreshold = (activeTargetIndex == 3) ? 50 : 80;
  
  if (abs(positionError) <= effectiveBand && abs(motorVelocity) < effectiveVelThreshold) {
    stopMotor();
    errorIntegral = 0;
    return;
  }
  
  errorIntegral += positionError * (float)executionDuration / 1000000.0;
  errorIntegral = constrain(errorIntegral, -1000, 1000);
  
  float velocityError = 0 - motorVelocity;
  float voltage = KP * positionError + KI * errorIntegral + KD * velocityError;
  
  if (positionError < -5) {
    voltage -= FRICTION_LEFT;
  } else if (positionError > 5) {
    voltage += FRICTION_RIGHT;
  }
  
  if (abs(voltage) > SUPPLY_VOLTAGE) {
    errorIntegral -= positionError * (float)executionDuration / 1000000.0;
    voltage = constrain(voltage, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
  }
  
  setMotorVoltage(voltage);
}

//============================================
// MOTOR & UTILITIES
//============================================
void setMotorVoltage(float voltage) {
  int pwm = (int)(abs(voltage) * 255.0 / SUPPLY_VOLTAGE);
  pwm = constrain(pwm, 0, 255);
  
  analogWrite(MOTOR_ENA, pwm);
  
  if (voltage >= 0) {
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
  }
}

void stopMotor() {
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
}

void homeToLeftLimit() {
  Serial.println(F("Homing..."));
  
  // Drive toward limit switch
  while (digitalRead(LIMIT_LEFT) == LOW) {
    setMotorVoltage(4.0);
    delay(10);
  }
  
  // Hold against limit while setting zero
  setMotorVoltage(2.5);
  delay(100);
  
  encoder.write(0);
  errorIntegral = 0;
  
  stopMotor();
  delay(50);
  
  Serial.println(F("Homed! Pos=0"));
}

void loadPIDFromEEPROM() {
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
}

void loadLanePositionsFromEEPROM() {
  for (int i = 0; i < 4; i++) {
    long pos;
    EEPROM.get(EEPROM_LANES_BASE + i * 4, pos);
    if (pos != 0 && pos > -2000 && pos < 100) {
      targetPositions[i] = pos;
    }
  }
}

void printStatus() {
  long pos = encoder.read();
  
  Serial.print(pos);
  Serial.print(F(" D:"));
  Serial.print(desiredPosition);
  Serial.print(F(" T:"));
  Serial.print(activeTargetIndex + 1);
  Serial.print(F(" |"));
  
  for (int i = 0; i < 4; i++) {
    int pct = (int)((1.0 - zombieDistances[i]) * 100);
    Serial.print(pct);
    if (ProxSensors[i].direction == FORWARD) Serial.print(F("F "));
    else if (ProxSensors[i].direction == BACKWARD) Serial.print(F("B "));
    else Serial.print(F("- "));
  }
  
  Serial.println();
}
