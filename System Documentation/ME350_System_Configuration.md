# ME350 Project - Complete System Configuration and Analysis

## Table of Contents
1. [Hardware Overview](#hardware-overview)
2. [Pin Configuration](#pin-configuration)
3. [Motor Control System](#motor-control-system)
4. [Encoder Configuration](#encoder-configuration)
5. [Limit Switch System](#limit-switch-system)
6. [Proximity Sensor Configuration](#proximity-sensor-configuration)
7. [Target Positions](#target-positions)
8. [PID Controller Values](#pid-controller-values)
9. [Friction Compensation](#friction-compensation)
10. [State Machine Architecture](#state-machine-architecture)
11. [Competition Requirements](#competition-requirements)
12. [Code File Summary](#code-file-summary)

---

## Hardware Overview

The ME350 project implements a **position-controlled linear actuator system** designed to intercept moving targets (zombies) on a playing field. The system uses:

- **Motor**: DC motor with H-bridge driver (L298N or similar)
- **Transmission**: 30:1 planetary gearbox
- **Encoder**: Quadrature encoder (64 CPR motor × 30:1 gearbox = 1920 CPR effective)
- **Linear Motion**: Rack and pinion mechanism
- **Sensors**: 4 proximity sensors for target detection
- **Limit Switches**: 2 switches for range calibration and safety
- **Controller**: Arduino (Uno/Mega compatible)

---

## Pin Configuration

### Standard Pin Assignment (Consistent Across All Files)

| Component | Pin Number | Arduino Pin Name | Mode | Notes |
|-----------|------------|------------------|------|-------|
| **Encoder A** | 2 | Digital 2 | INPUT | Interrupt-capable (INT0) |
| **Encoder B** | 3 | Digital 3 | INPUT | Interrupt-capable (INT1) |
| **Motor PWM** | 11 | Digital 11 (PWM) | OUTPUT | ENA pin on H-bridge |
| **Motor Direction 1** | 12 | Digital 12 | OUTPUT | IN2 pin on H-bridge |
| **Motor Direction 2** | 13 | Digital 13 | OUTPUT | IN3 pin on H-bridge |
| **Left Limit Switch** | 8 | Digital 8 | INPUT_PULLUP | Active LOW |
| **Right Limit Switch** | 9 | Digital 9 | INPUT_PULLUP | Active LOW |
| **Proximity Sensor 1** | A0 | Analog 0 | INPUT | Target position 1 |
| **Proximity Sensor 2** | A1 | Analog 1 | INPUT | Target position 2 |
| **Proximity Sensor 3** | A2 | Analog 2 | INPUT | Target position 3 |
| **Proximity Sensor 4** | A3 | Analog 3 | INPUT | Target position 4 |

### Code Example (Standard Pin Definitions)
```cpp
// ENCODER
#define ENCODER_A 2
#define ENCODER_B 3

// MOTOR CONTROL (H-Bridge)
#define MOTOR_ENA 11  // PWM pin for speed control
#define MOTOR_IN2 12  // Direction control 1
#define MOTOR_IN3 13  // Direction control 2

// LIMIT SWITCHES
#define LIMIT_LEFT 8   // Left limit (zero position)
#define LIMIT_RIGHT 9  // Right limit (max range)

// PROXIMITY SENSORS
#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3
```

---

## Motor Control System

### H-Bridge Control Logic

The system uses a standard H-bridge motor driver (L298N) with the following control scheme:

| IN2 (Pin 12) | IN3 (Pin 13) | ENA (Pin 11 PWM) | Motor Action | Direction |
|--------------|--------------|------------------|--------------|-----------|
| HIGH | LOW | PWM Value | Move LEFT | Toward position 0 |
| LOW | HIGH | PWM Value | Move RIGHT | Toward negative positions |
| LOW | LOW | Any | BRAKE | Stop |
| HIGH | HIGH | Any | BRAKE | Stop |

### Motor Direction Convention

**CRITICAL**: All code files use the following consistent convention:

- **Positive Voltage → Move LEFT** (toward encoder position 0, toward left limit switch)
- **Negative Voltage → Move RIGHT** (toward more negative encoder positions, toward right limit)

### Code Implementation

```cpp
void setMotorVoltage(float voltage) {
  // Limit voltage to ±10V range
  voltage = constrain(voltage, -10.0, 10.0);

  // Convert voltage to PWM (0-255)
  int pwmValue = abs(voltage) * 25.5;  // 10V → 255

  if (voltage > 0) {
    // Move LEFT (positive direction)
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_ENA, pwmValue);
  }
  else if (voltage < 0) {
    // Move RIGHT (negative direction)
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
```

### Voltage to PWM Conversion

- **Voltage Range**: ±10V
- **PWM Range**: 0-255
- **Conversion Factor**: 25.5 PWM units per volt
- **Example**: 5V → 127.5 PWM, 10V → 255 PWM

---

## Encoder Configuration

### Specifications

- **Motor Encoder**: 64 CPR (Counts Per Revolution)
- **Gearbox Ratio**: 30:1 planetary gearbox
- **Effective Resolution**: 64 × 30 = **1920 CPR**
- **Encoder Type**: Quadrature (2-channel)
- **Encoding**: Both rising and falling edges detected (4× resolution available)

### Encoder Reading Method

Two implementations found in the codebase:

#### 1. Encoder Library (Recommended - Modern Code)
```cpp
#include <Encoder.h>

Encoder motorEncoder(ENCODER_A, ENCODER_B);

void setup() {
  // Library handles interrupt setup automatically
}

void loop() {
  long position = motorEncoder.read();
}
```

#### 2. Manual Interrupt Method (Example Code)
```cpp
volatile long encoderPosition = 0;

void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
}

void encoderISR() {
  // Read both channels
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  // Determine direction and update count
  // (Implementation varies by code file)
}
```

### Position Convention

- **Zero Position**: At left limit switch
- **Positive Movement**: Encoder count increases when moving RIGHT
- **Negative Movement**: Encoder count decreases when moving LEFT
- **Typical Range**: 0 to approximately -1800 encoder counts (varies by mechanical setup)

---

## Limit Switch System

### Configuration

- **Type**: Mechanical limit switches (normally open or normally closed)
- **Arduino Mode**: `INPUT_PULLUP`
- **Active State**: **LOW** (0) when pressed
- **Inactive State**: **HIGH** (1) when not pressed

### Pin Assignments

- **Left Limit**: Pin 8 (defines zero position)
- **Right Limit**: Pin 9 (defines maximum range)

### Usage in Code

```cpp
void setup() {
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);
}

bool isLeftLimitHit() {
  return digitalRead(LIMIT_LEFT) == LOW;
}

bool isRightLimitHit() {
  return digitalRead(LIMIT_RIGHT) == LOW;
}
```

### Calibration Procedure

From `PID_AutoTune_Enhanced.ino`:

1. **Home to Left Limit**:
   - Apply positive voltage (move LEFT)
   - Wait for left limit switch to activate
   - Zero the encoder position

2. **Find Right Limit**:
   - Apply negative voltage (move RIGHT)
   - Record encoder position when right limit activates
   - Calculate total range

3. **Store Calibration**:
   - Save left position (typically 0)
   - Save right position (typically -1800 to -2000)
   - Save to EEPROM for persistence

### Safety Features

All code files include limit switch monitoring:
- Stop motor immediately if limit hit during operation
- Prevent movement commands beyond calibrated range
- Emergency stop capability

---

## Proximity Sensor Configuration

### Sensor Specifications

- **Count**: 4 sensors
- **Type**: Analog proximity/distance sensors
- **Connection**: Analog inputs A0-A3
- **Purpose**: Detect zombie position and movement direction

### Sensor Positions

Each sensor is mounted at a fixed position corresponding to target locations:

| Sensor | Pin | Target Position | Encoder Position (Typical) |
|--------|-----|-----------------|----------------------------|
| Sensor 1 | A0 | Target 1 | -74 counts |
| Sensor 2 | A1 | Target 2 | -307 counts |
| Sensor 3 | A2 | Target 3 | -547 counts |
| Sensor 4 | A3 | Target 4 | -1080 counts |

### Sensor Reading and Processing

From `UpdatedGameCodeNov17.ino`:

```cpp
struct SensorData {
  int rawValue;           // Raw analog reading (0-1023)
  float filteredValue;    // Low-pass filtered value
  int direction;          // FORWARD, BACKWARD, or STOPPED
  float derivative;       // Rate of change
  unsigned long lastUpdate;
};

SensorData sensors[4];

// Low-pass filter coefficient
const float alpha = 0.925;  // Higher = more filtering

void updateSensors() {
  for (int i = 0; i < 4; i++) {
    // Read raw value
    int raw = analogRead(A0 + i);

    // Apply low-pass filter
    sensors[i].filteredValue = alpha * sensors[i].filteredValue +
                               (1 - alpha) * raw;

    // Calculate derivative (rate of change)
    float derivative = sensors[i].filteredValue - sensors[i].rawValue;

    // Determine direction
    if (derivative > 2.0) {
      sensors[i].direction = FORWARD;   // Moving toward sensor
    } else if (derivative < -2.0) {
      sensors[i].direction = BACKWARD;  // Moving away from sensor
    } else {
      sensors[i].direction = STOPPED;   // Not moving
    }

    sensors[i].rawValue = raw;
  }
}
```

### Direction Detection Constants

```cpp
const int FORWARD = 1;    // Zombie moving toward sensor
const int BACKWARD = -1;  // Zombie moving away from sensor
const int STOPPED = 0;    // Zombie not moving
```

### Activation Threshold

From multiple code files, sensors typically detect zombies when:
- **Raw Value**: > 300-400 (out of 1023)
- **Filtered Value**: Used for direction detection
- **Debouncing**: Required to prevent false triggers

---

## Target Positions

### Encoder Positions (from UpdatedGameCodeNov17.ino)

| Target | Encoder Position | Proximity Sensor | Distance from Zero |
|--------|------------------|------------------|-------------------|
| Target 1 | -74 counts | Sensor 1 (A0) | Closest to left |
| Target 2 | -307 counts | Sensor 2 (A1) | Second position |
| Target 3 | -547 counts | Sensor 3 (A2) | Third position |
| Target 4 | -1080 counts | Sensor 4 (A3) | Farthest right |
| Wait Position | -547 counts | (Target 3) | Default waiting |

### Code Implementation

```cpp
// Target position definitions
long TARGET_1_POSITION = -74;
long TARGET_2_POSITION = -307;
long TARGET_3_POSITION = -547;
long TARGET_4_POSITION = -1080;

// Default waiting position
long WAIT_POSITION = TARGET_3_POSITION;

// Array for easy iteration
long targetPositions[4] = {
  TARGET_1_POSITION,
  TARGET_2_POSITION,
  TARGET_3_POSITION,
  TARGET_4_POSITION
};
```

### Distance Calculation

```cpp
long calculateDistance(int targetIndex) {
  long currentPosition = motorEncoder.read();
  long targetPosition = targetPositions[targetIndex];
  return abs(currentPosition - targetPosition);
}
```

---

## PID Controller Values

### Comparison Across Code Files

| Code File | Kp | Ki | Kd | Notes |
|-----------|-----|-----|-----|-------|
| **UpdatedGameCodeNov17.ino** | 0.020 | 0.005 | 0.004 | Most used, stable |
| **ME350_PID_FINAL_CORRECTED2.ino** | 0.020 | 0.005 | 0.004 | Same as above |
| **PID_VALUES_DEFINED.ino** | 0.020 | 0.005 | 0.004 | Same as above |
| **MotDirLimSwitchDirRangeCode.ino** | 0.020 | 0.005 | 0.004 | Same as above |
| **MostRecentTestCodeNov17.ino** | 0.28 | 0.015 | 0.012 | Aggressive tuning |
| **ME350_PID_Controller_TargetPractice.ino** | 0.1 | 0.0 | 0.0 | P-only (example) |

### Recommended Values (Conservative)

Based on most stable implementation:

```cpp
float KP = 0.020;  // Proportional gain
float KI = 0.005;  // Integral gain
float KD = 0.004;  // Derivative gain
```

### PID Control Loop Implementation

```cpp
// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// Deadband
const long DEADBAND = 5;  // encoder counts

// Control period
const unsigned long CONTROL_PERIOD = 10;  // ms (100 Hz)

void updatePID() {
  long currentPosition = motorEncoder.read();

  // Calculate error
  error = targetPosition - currentPosition;

  // Apply deadband
  if (abs(error) < DEADBAND) {
    error = 0;
  }

  // Calculate integral (with anti-windup)
  integral += error * (CONTROL_PERIOD / 1000.0);
  integral = constrain(integral, -1000, 1000);

  // Calculate derivative
  derivative = (error - lastError) / (CONTROL_PERIOD / 1000.0);

  // Calculate PID output
  float pidOutput = KP * error + KI * integral + KD * derivative;

  // Add friction compensation
  float frictionComp = 0;
  if (error < 0) {
    frictionComp = -FRICTION_LEFT;  // Moving RIGHT
  } else if (error > 0) {
    frictionComp = FRICTION_RIGHT;  // Moving LEFT
  }

  // Calculate final voltage
  float voltage = pidOutput + frictionComp;

  // Apply to motor
  setMotorVoltage(voltage);

  // Store for next iteration
  lastError = error;
}
```

### Tuning Guidelines

From `PID_AutoTune_Enhanced/ANALYSIS.md`:

**Conservative Tuning** (Recommended):
- Settling time: 2-4 seconds
- Overshoot: <10%
- Steady-state error: <5 encoder counts
- Stability margin: Very high

**Aggressive Tuning**:
- Settling time: <1 second
- Overshoot: ~40%
- Steady-state error: <2 encoder counts
- Stability margin: Lower (test carefully)

---

## Friction Compensation

### Purpose

Friction compensation adds a feedforward term to overcome static and kinetic friction, improving PID performance and reducing steady-state error.

### Implementation Variations Found

#### 1. Single-Value Friction (Basic)
```cpp
const float FRICTION = 4.0;  // Volts
```
Used in: MotDirLimSwitchDirRangeCode.ino, ME350_PID_FINAL_CORRECTED2.ino

#### 2. Directional Friction (Better)
```cpp
const float FRICTION_LEFT = 1.55;   // Volts (moving LEFT)
const float FRICTION_RIGHT = 2.9;   // Volts (moving RIGHT)
```
Used in: MostRecentTestCodeNov17.ino

#### 3. Adaptive Friction (Most Advanced)
```cpp
// From UpdatedGameCodeNov17.ino
const float BASE_FRICTION_LEFT = 2.2;
const float BASE_FRICTION_RIGHT = 0.25;

float adaptiveFrictionLeft = BASE_FRICTION_LEFT;
float adaptiveFrictionRight = BASE_FRICTION_RIGHT;

// Boost applied when target not reached
const float FRICTION_BOOST_AMOUNT = 0.2;
```

#### 4. Stribeck Friction Model (PID_AutoTune_Enhanced)
```cpp
struct FrictionModel {
  float staticLeft;      // Static friction (breakaway)
  float staticRight;
  float coulombLeft;     // Kinetic friction
  float coulombRight;
  float viscous;         // Velocity-dependent
  float stribeckVel;     // Stribeck velocity
  bool calibrated;
};
```

### Recommended Friction Compensation Logic

```cpp
float frictionComp = 0;

if (abs(error) > DEADBAND) {
  if (error < 0) {
    // Moving RIGHT (negative direction)
    frictionComp = -FRICTION_LEFT;
  } else {
    // Moving LEFT (positive direction)
    frictionComp = FRICTION_RIGHT;
  }
}

// Add to PID output
float voltage = pidOutput + frictionComp;
```

### Friction Characterization Procedure

From `PID_AutoTune_Enhanced.ino`:

1. **Home to left limit**
2. **Move RIGHT** with incrementing voltage until motion detected
3. **Record FRICTION_RIGHT** value
4. **Move to right limit**
5. **Move LEFT** with incrementing voltage until motion detected
6. **Record FRICTION_LEFT** value
7. **Save to EEPROM** for persistence

---

## State Machine Architecture

### Three-State System

All code files implement the same basic state machine structure from the ME350 example code:

```
┌─────────────┐
│  CALIBRATE  │ ──────┐
└─────────────┘       │
                      ▼
            ┌─────────────────────┐
            │ CHOOSE_ACTIVE_TARGET│◄────┐
            └─────────────────────┘     │
                      │                 │
                      ▼                 │
            ┌─────────────────────┐     │
            │   MOVE_TO_TARGET    │─────┘
            └─────────────────────┘
```

### State Definitions

```cpp
enum State {
  CALIBRATE = 1,
  CHOOSE_ACTIVE_TARGET = 2,
  MOVE_TO_TARGET = 3
};

State currentState = CALIBRATE;
```

### State 1: CALIBRATE

**Purpose**: Initialize system, find zero position

**Actions**:
1. Apply constant positive voltage (move LEFT)
2. Wait for left limit switch activation
3. Check for zero velocity (stopped at limit)
4. Zero the encoder position
5. Transition to CHOOSE_ACTIVE_TARGET

**Code**:
```cpp
case CALIBRATE:
  if (!digitalRead(LIMIT_LEFT)) {  // Limit switch pressed (active LOW)
    // Check if stopped (low velocity)
    long currentPos = motorEncoder.read();
    if (abs(currentPos - lastCalibrationPos) < 2) {
      // Stopped at limit
      motorEncoder.write(0);  // Zero position
      currentState = CHOOSE_ACTIVE_TARGET;
      setMotorVoltage(0);
    }
    lastCalibrationPos = currentPos;
  } else {
    // Continue moving to left limit
    setMotorVoltage(5.0);  // Constant voltage LEFT
  }
  break;
```

### State 2: CHOOSE_ACTIVE_TARGET

**Purpose**: Determine which zombie to target

**Logic**:
1. Read all 4 proximity sensors
2. Filter sensor values
3. Determine direction for each zombie (FORWARD/BACKWARD/STOPPED)
4. Calculate distance from current position to each target
5. **Priority**: Select closest FORWARD-moving zombie
6. If no forward-moving zombies, go to wait position
7. Set target position
8. Transition to MOVE_TO_TARGET

**Code** (from UpdatedGameCodeNov17.ino):
```cpp
case CHOOSE_ACTIVE_TARGET:
  updateSensors();

  int closestTarget = -1;
  long minDistance = 999999;

  // Find closest FORWARD-moving zombie
  for (int i = 0; i < 4; i++) {
    if (sensors[i].direction == FORWARD) {
      long distance = calculateDistance(i);
      if (distance < minDistance) {
        minDistance = distance;
        closestTarget = i;
      }
    }
  }

  if (closestTarget >= 0) {
    // Target found
    targetPosition = targetPositions[closestTarget];
    activeTarget = closestTarget;
  } else {
    // No targets, go to wait position
    targetPosition = WAIT_POSITION;
    activeTarget = -1;
  }

  currentState = MOVE_TO_TARGET;
  break;
```

### State 3: MOVE_TO_TARGET

**Purpose**: Execute PID control to reach target position

**Actions**:
1. Run PID control loop
2. Monitor proximity sensor for zombie activation
3. If zombie detected (sensor > threshold):
   - Record hit
   - Increment score
   - Return to CHOOSE_ACTIVE_TARGET
4. If position reached and stable:
   - Wait for activation time
   - If no activation, return to CHOOSE_ACTIVE_TARGET

**Code**:
```cpp
case MOVE_TO_TARGET:
  updatePID();  // Run PID controller

  // Check if zombie activated LED
  if (activeTarget >= 0) {
    if (sensors[activeTarget].rawValue > ACTIVATION_THRESHOLD) {
      // Zombie hit!
      score++;
      currentState = CHOOSE_ACTIVE_TARGET;
      activationTime = millis();
    }
  }

  // Check if position stable
  if (abs(error) < DEADBAND) {
    if (millis() - positionReachedTime > WAIT_TIME) {
      // Position held, no activation
      currentState = CHOOSE_ACTIVE_TARGET;
    }
  } else {
    positionReachedTime = millis();  // Reset timer
  }
  break;
```

---

## Competition Requirements

### Overview

The ME350 competition consists of **three rounds** per run, each with different rules and scoring.

### Round 1: Standard Play

- **Duration**: 40 seconds
- **Speed**: Normal zombie speed
- **Reset Conditions**:
  - LED activation (zombie stops when LED lights)
  - Front limit switch contact (zombie stops when hitting barrier)
- **Scoring**: 1 point per zombie stopped
- **Strategy**: Balance speed and accuracy

### Round 2: Fast Play

- **Duration**: 40 seconds
- **Speed**: Faster zombie speed than Round 1
- **Reset Conditions**:
  - LED activation
  - Front limit switch contact
- **Scoring**: 1 point per zombie stopped
- **Strategy**: Requires faster PID response, predictive targeting

### Round 3: Survival Mode

- **Duration**: Until first zombie reaches front limit switch
- **Speed**: Increases progressively throughout round
- **Reset Conditions**:
  - **LED activation ONLY** (limit switch does NOT count)
  - Zombies that hit limit switch end the round (failure)
- **Scoring**: 1 point per zombie stopped via LED
- **Strategy**: Critical to hit zombies before they reach the limit
- **End Condition**: First zombie to hit front limit switch ends Round 3

### Scoring System

```cpp
int round1Score = 0;
int round2Score = 0;
int round3Score = 0;
int totalScore = 0;

// After each round
totalScore = round1Score + round2Score + round3Score;
```

### Round Management Code Structure

```cpp
enum Round {
  ROUND_1 = 1,
  ROUND_2 = 2,
  ROUND_3 = 3,
  COMPLETE = 4
};

Round currentRound = ROUND_1;
unsigned long roundStartTime;

void checkRoundTransition() {
  unsigned long elapsed = millis() - roundStartTime;

  switch (currentRound) {
    case ROUND_1:
      if (elapsed > 40000) {  // 40 seconds
        currentRound = ROUND_2;
        roundStartTime = millis();
      }
      break;

    case ROUND_2:
      if (elapsed > 40000) {  // 40 seconds
        currentRound = ROUND_3;
        roundStartTime = millis();
      }
      break;

    case ROUND_3:
      // Check for limit switch hit (failure condition)
      if (digitalRead(LIMIT_RIGHT) == LOW) {
        currentRound = COMPLETE;
        setMotorVoltage(0);  // Stop
      }
      break;
  }
}
```

### Special Handling for Round 3

**CRITICAL**: In Round 3, limit switch activation is a **failure condition**, not a scoring condition.

```cpp
if (currentRound == ROUND_3) {
  // Only LED activation counts as success
  if (activeTarget >= 0 && sensors[activeTarget].rawValue > THRESHOLD) {
    round3Score++;
  }

  // Limit switch ends the round
  if (digitalRead(LIMIT_RIGHT) == LOW) {
    // Round 3 failed - zombie reached barrier
    gameOver = true;
  }
}
```

---

## Code File Summary

### Working/Production Code

#### 1. UpdatedGameCodeNov17.ino
- **Status**: Most sophisticated competition implementation
- **Features**:
  - Complete 3-state machine
  - Adaptive friction compensation
  - Low-pass filtered sensor readings
  - Direction detection (FORWARD/BACKWARD/STOPPED)
  - Closest forward-moving zombie selection
  - Score tracking
- **PID**: Kp=0.020, Ki=0.005, Kd=0.004
- **Use Case**: Primary competition code

#### 2. MostRecentTestCodeNov17.ino
- **Status**: Test version with aggressive tuning
- **Features**:
  - EEPROM calibration storage
  - Aggressive PID gains
  - Adaptive boost mechanism
  - Calibration mode
- **PID**: Kp=0.28, Ki=0.015, Kd=0.012
- **Use Case**: Testing faster response

#### 3. ME350_PID_FINAL_CORRECTED2.ino
- **Status**: Earlier stable version
- **Features**:
  - Basic state machine
  - Relay oscillation auto-tune function
  - Single friction value
- **PID**: Kp=0.020, Ki=0.005, Kd=0.004
- **Use Case**: Reference implementation

### Auto-Tuning Code

#### 4. PID_AutoTune_Enhanced.ino
- **Status**: Comprehensive PID tuning tool
- **Features**:
  - Range calibration
  - Friction characterization (Stribeck model)
  - Multiple tuning methods:
    - Ziegler-Nichols (Classic)
    - Tyreus-Luyben (Conservative)
    - Cohen-Coon (Fast)
    - Step Response (Lambda)
  - EEPROM storage
  - Extensive diagnostics
  - Step response analysis
- **Use Case**: System identification and PID tuning

### Example/Reference Code

#### 5. ME350_PID_Controller_TargetPractice.ino
- **Status**: Official example code provided
- **Features**:
  - Basic structure template
  - Interrupt-based encoder
  - P-only control
  - Simple state machine
- **PID**: Kp=0.1, Ki=0.0, Kd=0.0
- **Use Case**: Learning/reference

---

## Recommendations

### For Competition Use

1. **Use UpdatedGameCodeNov17.ino as base**
   - Most robust target switching logic
   - Proven stable PID values
   - Adaptive friction compensation

2. **Add Round Management**
   - Implement 3-round structure
   - Different rules per round
   - Round 3 special handling (LED-only)

3. **Consider Auto-Tuning First**
   - Run PID_AutoTune_Enhanced to characterize system
   - Use Conservative (Tyreus-Luyben) tuning
   - Copy resulting values to competition code

### For Testing and Development

1. **Always calibrate before testing**
   - Run CALIBRATE state on startup
   - Verify encoder zero position

2. **Monitor limit switches**
   - Never disable safety checks
   - Stop immediately on unexpected limit hit

3. **Test PID gains incrementally**
   - Start with conservative values
   - Increase gradually
   - Monitor overshoot and settling time

---

## Additional Notes

### EEPROM Storage Map

From `PID_AutoTune_Enhanced.ino`:

| Address | Data | Size (bytes) |
|---------|------|--------------|
| 0 | Calibration Flag | 1 |
| 1-4 | KP (float) | 4 |
| 5-8 | KI (float) | 4 |
| 9-12 | KD (float) | 4 |
| 13-16 | FRICTION_LEFT (float) | 4 |
| 17-20 | FRICTION_RIGHT (float) | 4 |
| 21-24 | LEFT_LIMIT (long) | 4 |
| 25-28 | RIGHT_LIMIT (long) | 4 |

### Serial Communication

All code uses **115200 baud** for serial communication.

```cpp
void setup() {
  Serial.begin(115200);
}
```

### Timing Constants

- **Control Loop Period**: 10 ms (100 Hz)
- **Sensor Update Period**: 10 ms
- **State Machine Period**: 10 ms (runs in main loop)

---

## Document Version

**Version**: 1.0
**Date**: November 17, 2024
**Author**: ME350 Project Analysis
**Based on**: 6 Arduino code files + comprehensive documentation review
