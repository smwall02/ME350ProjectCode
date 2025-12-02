#include <Encoder.h>
#include <EEPROM.h>

// Basic pin assignments
const int ENCODER_A = 2;
const int ENCODER_B = 3;
const int MOTOR_PWM = 11;
const int MOTOR_IN1 = 12;
const int MOTOR_IN2 = 13;
const int LIMIT_SWITCH = 8;
const int ON_OFF_SWITCH = 5;
const int LASER_PIN = 7; // photosensor driver

const int PROX_PINS[4] = {A0, A1, A2, A3};

Encoder encoder(ENCODER_A, ENCODER_B);

const int FORWARD = 1;
const int BACKWARD = -1;
const int STOPPED = 0;

// Motor control
float KP = 0.0177f;
float KD = 0.0015f;
float targetAngleDeg = 0.0f;
long targetCounts = 0;
long motorPositionCounts = 0;
float motorVelocityCPS = 0.0f;

// Timing
unsigned long lastLoopMicros = 0;
const unsigned long MIN_VEL_COMP_TIME = 8000; // us
const int MIN_VEL_COMP_COUNT = 2;

// Proximity sensors
struct Sensor {
  float currVal;
  float prevVal;
  unsigned long prevChangeTime;
  float speed; // counts per ms
  int direction; // 1 forward (toward photosensor), -1 backward, 0 stopped
  int pin;
};

Sensor prox[4];

// Calibrated ranges [far,start],[impact] copied from latest calibration snapshot
int ProxRange[4][2] = {
  {638, 78},
  {624, 102},
  {628, 92},
  {645, 132}
};

// Distance cache and TTI
float zombieDistance[4] = {0, 0, 0, 0};
float laneTTI[4] = {99999, 99999, 99999, 99999};

// State machine
enum SystemState {
  ST_IDLE = 0,
  ST_HOME,
  ST_SELECT_LANE,
  ST_AIM,
  ST_FIRE_DWELL,
  ST_PAUSE,
  ST_ERROR
};

SystemState systemState = ST_IDLE;

// Game control
bool gameRunning = false;
unsigned long dwellStartMillis = 0;
const unsigned long SENSOR_DWELL_TIME_MS = 200;

// Target positions (encoder counts)
const long targetPositions[4] = {-109, -376, -629, -1257};
const float encoderCountsPerDeg = 2.0f; // adjust if needed

// Selection rules
const float MAX_ENGAGE_DISTANCE = 0.65f;
const float EMERGENCY_DISTANCE = 0.75f;
const float SWITCH_TTI_MARGIN = 200.0f; // ms improvement required to switch
int lastLane = -1;
float lastLaneTTI = 99999.0f;
unsigned long lastLaneCommit = 0;
const unsigned long MIN_SERVICE_TIME = 300; // ms

//=====================================================
// Helper prototypes
//=====================================================
void updateSensors(unsigned long nowMillis);
void updateMotorVelocity(unsigned long nowMicros);
float computeDistance(int lane, float raw);
float computeTTI(int lane, unsigned long nowMillis);
int pickLane(unsigned long nowMillis);
bool atTargetAngle();
void setTargetLane(int lane);
void setMotorCommand(float voltage);
void stopMotor();
void handleAimState(unsigned long nowMillis);
void handleFireDwellState(unsigned long nowMillis);
void laserOn();
void laserOff();

//=====================================================
// Setup
//=====================================================
void setup() {
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(ON_OFF_SWITCH, INPUT);
  pinMode(LASER_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    prox[i].pin = PROX_PINS[i];
    prox[i].currVal = analogRead(prox[i].pin);
    prox[i].prevVal = prox[i].currVal;
    prox[i].prevChangeTime = millis();
    prox[i].speed = 0;
    prox[i].direction = 0;
  }

  Serial.begin(115200);
  Serial.println("TTI Aim State Machine v1");
  lastLoopMicros = micros();
}

//=====================================================
// Main Loop
//=====================================================
void loop() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // Keep position cache
  motorPositionCounts = encoder.read();
  updateMotorVelocity(nowMicros);
  updateSensors(nowMillis);

  // Simple serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'T' || c == 'Y' || c == 'G') {
      gameRunning = true;
      systemState = ST_SELECT_LANE;
    } else if (c == 'S') {
      gameRunning = false;
      systemState = ST_IDLE;
    } else if (c >= '1' && c <= '4') {
      gameRunning = false;
      setTargetLane((int)(c - '1'));
      systemState = ST_AIM;
    }
  }

  switch (systemState) {
    case ST_IDLE:
      stopMotor();
      laserOff();
      if (gameRunning) systemState = ST_SELECT_LANE;
      break;

    case ST_SELECT_LANE: {
      int lane = pickLane(nowMillis);
      if (lane >= 0) {
        setTargetLane(lane);
        systemState = ST_AIM;
      } else {
        stopMotor();
      }
      break;
    }

    case ST_AIM:
      handleAimState(nowMillis);
      break;

    case ST_FIRE_DWELL:
      handleFireDwellState(nowMillis);
      break;

    case ST_PAUSE:
      stopMotor();
      laserOff();
      break;

    case ST_ERROR:
      stopMotor();
      laserOff();
      break;
  }

  lastLoopMicros = nowMicros;
}

//=====================================================
// Sensor + Threat model
//=====================================================
void updateSensors(unsigned long nowMillis) {
  for (int i = 0; i < 4; i++) {
    float raw = analogRead(prox[i].pin);
    prox[i].currVal = 0.85f * prox[i].currVal + 0.15f * raw;
    unsigned long dt = nowMillis - prox[i].prevChangeTime;
    if (dt > 0) {
      float delta = prox[i].currVal - prox[i].prevVal;
      prox[i].speed = delta / (float)dt; // counts per ms
      if (fabs(delta) > 3) {
        prox[i].direction = (delta < 0) ? FORWARD : BACKWARD;
        prox[i].prevVal = prox[i].currVal;
        prox[i].prevChangeTime = nowMillis;
      }
    }
    zombieDistance[i] = computeDistance(i, prox[i].currVal);
    laneTTI[i] = computeTTI(i, nowMillis);
  }
}

float computeDistance(int lane, float raw) {
  float span = (float)(ProxRange[lane][0] - ProxRange[lane][1]);
  if (span <= 0.1f) return 0.0f;
  float d = (raw - ProxRange[lane][1]) / span; // 0 at impact end
  d = constrain(d, 0.0f, 1.0f);
  return d;
}

float computeTTI(int lane, unsigned long nowMillis) {
  // Only forward motion counts
  if (prox[lane].direction != FORWARD) return 99999.0f;
  float dist = zombieDistance[lane];
  if (dist > EMERGENCY_DISTANCE) return 0.0f; // emergency now

  // Convert prox speed to distance speed
  float span = (float)(ProxRange[lane][0] - ProxRange[lane][1]);
  if (span <= 0.1f) return 99999.0f;
  float distSpeed = prox[lane].speed / span; // per ms
  if (distSpeed >= 0.0f || fabs(distSpeed) < 0.0001f) return 99999.0f;

  float criticalPos = 0.10f;
  float tti = (dist - criticalPos) / -distSpeed; // ms until hitting critical
  if (tti < 0) tti = 0;
  return tti;
}

int pickLane(unsigned long nowMillis) {
  int best = -1;
  float bestTTI = 99999.0f;

  for (int i = 0; i < 4; i++) {
    float dist = zombieDistance[i];
    if (dist > EMERGENCY_DISTANCE) {
      // Immediate emergency override
      best = i;
      bestTTI = -1;
      break;
    }
    if (dist < 0.05f || dist > MAX_ENGAGE_DISTANCE) continue; // ignore very near/far
    float tti = laneTTI[i];
    if (tti < bestTTI) {
      best = i;
      bestTTI = tti;
    }
  }

  // Hysteresis: only switch if better by margin and after service time
  if (best >= 0 && lastLane >= 0 && (nowMillis - lastLaneCommit) < MIN_SERVICE_TIME) {
    return lastLane;
  }
  if (best >= 0 && lastLane >= 0 && lastLane != best) {
    if ((bestTTI + SWITCH_TTI_MARGIN) >= lastLaneTTI) {
      return lastLane;
    }
  }

  if (best >= 0) {
    lastLane = best;
    lastLaneTTI = bestTTI;
    lastLaneCommit = nowMillis;
  }
  return best;
}

//=====================================================
// Motor helpers
//=====================================================
void updateMotorVelocity(unsigned long nowMicros) {
  static long lastPos = 0;
  static unsigned long lastTime = 0;
  if (lastTime == 0) {
    lastTime = nowMicros;
    lastPos = motorPositionCounts;
    return;
  }
  unsigned long dt = nowMicros - lastTime;
  if (dt < MIN_VEL_COMP_TIME) return;
  long pos = motorPositionCounts;
  long delta = pos - lastPos;
  motorVelocityCPS = (float)delta * 1e6f / (float)dt;
  lastPos = pos;
  lastTime = nowMicros;
}

float motorCountsToDegrees(long counts) {
  return (float)counts / encoderCountsPerDeg;
}

long motorDegreesToCounts(float deg) {
  return (long)(deg * encoderCountsPerDeg);
}

bool atTargetAngle() {
  float currentDeg = motorCountsToDegrees(motorPositionCounts);
  float targetDeg = motorCountsToDegrees(targetCounts);
  return fabs(currentDeg - targetDeg) <= 1.0f;
}

void setTargetLane(int lane) {
  targetCounts = targetPositions[lane];
  targetAngleDeg = motorCountsToDegrees(targetCounts);
}

void setMotorCommand(float voltage) {
  float command = constrain(fabs(voltage) * 255.0f / 6.0f, 0.0f, 255.0f);
  int pwm = (int)command;
  analogWrite(MOTOR_PWM, pwm);
  if (voltage >= 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
}

void stopMotor() {
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void laserOn() {
  digitalWrite(LASER_PIN, HIGH);
}

void laserOff() {
  digitalWrite(LASER_PIN, LOW);
}

//=====================================================
// State handlers
//=====================================================
void handleAimState(unsigned long nowMillis) {
  // Simple PD
  float error = (float)(targetCounts - motorPositionCounts);
  float dterm = -motorVelocityCPS;
  float voltage = KP * error + KD * dterm;
  voltage = constrain(voltage, -6.0f, 6.0f);
  setMotorCommand(voltage);

  if (!gameRunning && digitalRead(ON_OFF_SWITCH) == LOW) {
    stopMotor();
    systemState = ST_IDLE;
    return;
  }

  if (atTargetAngle()) {
    laserOn();
    dwellStartMillis = nowMillis;
    systemState = ST_FIRE_DWELL;
  }
}

void handleFireDwellState(unsigned long nowMillis) {
  if ((nowMillis - dwellStartMillis) >= SENSOR_DWELL_TIME_MS) {
    laserOff();
    if (!gameRunning) {
      systemState = ST_IDLE;
    } else {
      systemState = ST_SELECT_LANE;
    }
  }
}
