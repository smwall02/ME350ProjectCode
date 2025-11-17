# AI Assistant Reference - ME350 Project

**Purpose:** Quick reference guide for AI assistants working on this ME350 mechatronics project

---

## Project Overview

**Name:** ME350 "Plants vs Zombies" Target Interception System

**Goal:** Control a linear actuator mechanism to intercept moving targets (zombies) using PID control and proximity sensors

**Competition Structure:**
- **Round 1:** 40s, normal speed, LED OR limit switch = point
- **Round 2:** 40s, faster speed, LED OR limit switch = point
- **Round 3:** Survival, LED ONLY = point, limit switch = GAME OVER

---

## Critical File Locations

### Documentation (READ THESE FIRST!)
```
System Documentation/
  └── ME350_System_Configuration.md     ← COMPREHENSIVE REFERENCE (900 lines)

In Progress/
  ├── PID_Tuning_Improved/
  │   ├── PID_AutoTune_ME350.ino        ← Auto-tune tool
  │   └── README.md                      ← Tuning guide
  ├── ME350_Competition_StateMachine_v2.ino  ← LATEST COMPETITION CODE (with setup mode)
  └── ME350_Competition_README.md        ← Competition code guide

Project Information/
  ├── Example Code/
  │   └── ME350_LinkageCode_V5.1 (1).pdf ← Official example code
  └── General Reference Documents/
      └── ME_350__PID_Controllers_11_16_19.pdf ← PID theory
```

### Working Code (Reference Implementations)
```
Working or Ongoing values/
  ├── UpdatedGameCodeNov17/              ← Most sophisticated existing implementation
  ├── PID_AutoTune_Enhanced/             ← Advanced auto-tune (older version)
  └── MostRecentTestCodeNov17/           ← Test code with aggressive tuning
```

---

## Hardware Configuration (STANDARDIZED ACROSS ALL FILES)

### Pin Assignments

| Component | Pin | Arduino Name | Mode | Notes |
|-----------|-----|--------------|------|-------|
| **Encoder A** | 2 | Digital 2 | INPUT | Interrupt-capable (INT0) |
| **Encoder B** | 3 | Digital 3 | INPUT | Interrupt-capable (INT1) |
| **Motor PWM** | 11 | Digital 11 | OUTPUT | ENA on H-bridge, analogWrite 0-255 |
| **Motor Dir 1** | 12 | Digital 12 | OUTPUT | IN2 on H-bridge |
| **Motor Dir 2** | 13 | Digital 13 | OUTPUT | IN3 on H-bridge |
| **Left Limit** | 8 | Digital 8 | INPUT_PULLUP | Active LOW when pressed |
| **Right Limit** | 9 | Digital 9 | INPUT_PULLUP | Active LOW when pressed |
| **Sensor 1** | A0 | Analog 0 | INPUT | Lane 1, reads 0-1023 |
| **Sensor 2** | A1 | Analog 1 | INPUT | Lane 2, reads 0-1023 |
| **Sensor 3** | A2 | Analog 2 | INPUT | Lane 3, reads 0-1023 |
| **Sensor 4** | A3 | Analog 3 | INPUT | Lane 4, reads 0-1023 |

### Motor Direction Convention

**CRITICAL:**
- **Positive voltage** → Move LEFT → Encoder increases → Toward position 0
- **Negative voltage** → Move RIGHT → Encoder decreases → Toward negative positions

```cpp
// H-bridge control
if (voltage > 0) {
  // Move LEFT
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
}
else if (voltage < 0) {
  // Move RIGHT
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
}
```

### Encoder Specifications

- **Motor CPR:** 64 counts per revolution
- **Gearbox Ratio:** 30:1
- **Effective CPR:** 1920 counts per revolution
- **Typical Range:** 0 (left limit) to approximately -1800 (right limit)
- **Zero Position:** Left limit switch

---

## CRITICAL: Sensor Logic (COMMON MISTAKE!)

### How Zombies Move

Zombies move on LINEAR TRACKS from **BACK** → **FRONT** (toward RIGHT limit switch/goal)

### Sensor Value Behavior

As a zombie passes a sensor:

```
Time:    T1     T2     T3     T4     T5
Zombie:  Far ← Closer → AT → Leaving → Far
Value:   50    200    600    300     50
         ↑ INCREASING ↑     ↓ DECREASING ↓
```

### Direction Detection

```cpp
// Calculate derivative
float derivative = filteredValue - lastFilteredValue;

if (derivative > 2.0) {
  direction = APPROACHING;  // Value INCREASING = zombie coming toward sensor
}
else if (derivative < -2.0) {
  direction = LEAVING;      // Value DECREASING = zombie moving past sensor
}
else {
  direction = STOPPED;      // Value stable
}
```

### Targeting Logic

**TARGET zombies that are:**
- ✅ APPROACHING (sensor value increasing) - We can still intercept
- ✅ High stable value (at sensor) - Intercept now

**DO NOT target zombies that are:**
- ❌ LEAVING (sensor value decreasing) - Too late, already passing

```cpp
// CORRECT target selection
for (int i = 0; i < 4; i++) {
  if (sensors[i].rawValue > THRESHOLD && sensors[i].direction == APPROACHING) {
    // This zombie is approaching - we should target it!
  }
}
```

**Common mistake:** Confusing game direction with sensor direction
- Game FORWARD = toward goal (value decreasing after passing sensor)
- Sensor APPROACHING = toward sensor (value increasing) ← THIS is what we target

---

## Key Variables

### Target Positions (Encoder Values)

```cpp
long TARGET_1_POSITION = -74;    // Lane 1 (closest to left)
long TARGET_2_POSITION = -307;   // Lane 2
long TARGET_3_POSITION = -547;   // Lane 3
long TARGET_4_POSITION = -1080;  // Lane 4 (closest to right)
long WAIT_POSITION = -547;       // Default waiting position
```

**NOTE:** These must be measured for each specific setup! Use PID auto-tune tool to find actual positions.

### PID Gains

```cpp
float KP = 0.020;  // Proportional gain
float KI = 0.005;  // Integral gain
float KD = 0.004;  // Derivative gain
```

**Values vary across implementations:**
- Conservative: Kp ≈ 0.018-0.022
- Aggressive: Kp ≈ 0.28

**ALWAYS run auto-tune first!** Use `PID_AutoTune_ME350.ino`

### Friction Compensation

```cpp
float FRICTION_LEFT = 2.2;   // Volts to overcome friction moving LEFT (RIGHT direction)
float FRICTION_RIGHT = 0.25; // Volts to overcome friction moving RIGHT (LEFT direction)
```

**NOTE:** Direction naming is confusing!
- `FRICTION_LEFT` is applied when moving RIGHT (negative error, need negative voltage)
- `FRICTION_RIGHT` is applied when moving LEFT (positive error, need positive voltage)

**Better naming in newer code:**
```cpp
float FRICTION_MOVING_RIGHT = 2.2;  // Applied when error < 0
float FRICTION_MOVING_LEFT = 0.25;  // Applied when error > 0
```

### Sensor Configuration

```cpp
const int ACTIVATION_THRESHOLD = 400;  // Sensor value to detect zombie (0-1023)
const float ALPHA = 0.925;             // Low-pass filter coefficient (higher = more filtering)
const float MOVEMENT_THRESHOLD = 2.0;   // Derivative threshold for direction detection
```

### Timing

```cpp
const unsigned long CONTROL_PERIOD = 10;     // ms (100 Hz update rate)
const long DEADBAND = 5;                     // Encoder counts - position tolerance
const unsigned long WAIT_TIME = 1000;        // ms - wait at target before giving up
const unsigned long ROUND_1_DURATION = 40000; // ms (40 seconds)
const unsigned long ROUND_2_DURATION = 40000; // ms (40 seconds)
```

---

## State Machine Architecture

### Three States

```
CALIBRATE → CHOOSE_ACTIVE_TARGET ⇄ MOVE_TO_TARGET
                     ↑__________________|
```

### State Descriptions

1. **CALIBRATE**
   - Move to left limit switch
   - Zero encoder position
   - Run once at startup
   - Transition: → CHOOSE_ACTIVE_TARGET

2. **CHOOSE_ACTIVE_TARGET**
   - Read all sensors
   - Detect direction (APPROACHING/LEAVING/STOPPED)
   - Select closest APPROACHING zombie
   - If none, go to wait position
   - Transition: → MOVE_TO_TARGET

3. **MOVE_TO_TARGET**
   - Run PID control to reach target
   - Monitor sensor for activation (LED lights)
   - Allow dynamic target switching
   - If activated: record score → CHOOSE_ACTIVE_TARGET
   - If timeout: → CHOOSE_ACTIVE_TARGET

---

## Common Tasks

### Task: Tune PID Gains

1. Upload `In Progress/PID_Tuning_Improved/PID_AutoTune_ME350.ino`
2. Open Serial Monitor (115200 baud)
3. Run commands in order:
   ```
   R  ← Calibrate range
   F  ← Characterize friction
   Z  ← Auto-tune (select option 1: Conservative)
   T  ← Test with manual position
   ```
4. Note the final Kp, Ki, Kd values
5. Copy to competition code

### Task: Calibrate Target Positions

1. Upload PID auto-tune code
2. Run `R` command to calibrate
3. Manually move mechanism to each sensor position
4. For each position, run `P` command to print current encoder value
5. Record positions for each target
6. Update `targetPositions[]` array in competition code

### Task: Modify Competition Code

**Latest version:** `In Progress/ME350_Competition_StateMachine_v2.ino`

**Setup mode:**
- Upload code, open Serial Monitor
- Use `P` command to set each target position
- Use `G` command to set PID gains
- Use `F` command to set friction values
- Use `V` to view configuration
- Use `S` to start competition

### Task: Debug Sensor Issues

1. Print raw sensor values:
   ```cpp
   Serial.print("S1:");
   Serial.print(sensors[0].rawValue);
   Serial.print(" S2:");
   Serial.print(sensors[1].rawValue);
   // etc...
   ```

2. Check direction detection:
   ```cpp
   Serial.print(" Dir:");
   Serial.print(sensors[i].direction == APPROACHING ? "APP" :
                sensors[i].direction == LEAVING ? "LEAV" : "STOP");
   ```

3. Monitor derivative:
   ```cpp
   float derivative = sensors[i].filteredValue - sensors[i].lastFilteredValue;
   Serial.print(" Deriv:");
   Serial.print(derivative);
   ```

---

## Code Patterns

### Reading Limit Switches

```cpp
// Active LOW with INPUT_PULLUP
pinMode(LIMIT_LEFT, INPUT_PULLUP);

if (digitalRead(LIMIT_LEFT) == LOW) {
  // Limit switch PRESSED
}
else {
  // Limit switch NOT pressed
}
```

### PID Control with Friction

```cpp
// Calculate PID output
float pidOutput = KP * error + KI * integral + KD * derivative;

// Add friction compensation
float frictionComp = 0;
if (error < -DEADBAND) {
  frictionComp = -FRICTION_LEFT;  // Moving RIGHT
}
else if (error > DEADBAND) {
  frictionComp = FRICTION_RIGHT;  // Moving LEFT
}

float voltage = pidOutput + frictionComp;
setMotorVoltage(voltage);
```

### Low-Pass Filter

```cpp
const float ALPHA = 0.925;  // Filter coefficient

// Apply filter
filteredValue = ALPHA * filteredValue + (1.0 - ALPHA) * rawValue;

// Calculate derivative
float derivative = filteredValue - lastFilteredValue;
```

---

## Troubleshooting

### Problem: Motor doesn't move

**Check:**
1. Motor power supply (12V connected?)
2. Pin connections (11, 12, 13)
3. H-bridge enable signal
4. Call to `setMotorVoltage()` with non-zero value

### Problem: Wrong direction

**Check:**
1. IN2/IN3 pins might be swapped
2. Verify: positive voltage should move LEFT (toward position 0)
3. If backwards, swap pin definitions or swap wires

### Problem: Encoder doesn't count

**Check:**
1. Encoder power (5V)
2. Pins 2 and 3 connections
3. Encoder library installed
4. Print `motorEncoder.read()` to verify

### Problem: Targets wrong zombies

**Check:**
1. Verify sensor direction detection logic
2. Print derivative values - should be positive when approaching
3. Ensure targeting APPROACHING, not LEAVING
4. Check target positions match sensor locations

### Problem: Oscillates at target

**Solution:**
1. Reduce Kp (by 20%)
2. Reduce Ki (by 50%)
3. Increase Kd (by 20%)
4. Or re-run auto-tune with Conservative preset

---

## Development Workflow

### Recommended Order

1. **Read documentation:**
   - `System Documentation/ME350_System_Configuration.md`
   - `In Progress/ME350_Competition_README.md`

2. **Run PID auto-tune:**
   - Upload `PID_AutoTune_ME350.ino`
   - Calibrate: R → F → Z → T
   - Record gains

3. **Calibrate target positions:**
   - Manually position at each sensor
   - Record encoder values

4. **Configure competition code:**
   - Upload `ME350_Competition_StateMachine_v2.ino`
   - Use setup mode to configure
   - Test before competition

5. **Test and iterate:**
   - Verify sensor detection
   - Test target selection logic
   - Verify scoring
   - Test all 3 rounds

---

## Important Notes for AI Assistants

### Do NOT:
- ❌ Assume sensor logic without verification
- ❌ Confuse game direction (toward goal) with sensor direction (toward sensor)
- ❌ Use FORWARD to mean "toward goal" - use APPROACHING for "toward sensor"
- ❌ Hardcode positions without user verification
- ❌ Skip auto-tuning (gains vary per setup!)

### DO:
- ✅ Verify sensor direction logic carefully
- ✅ Use latest code: `ME350_Competition_StateMachine_v2.ino`
- ✅ Read `System Documentation/ME350_System_Configuration.md` first
- ✅ Check existing implementations in `Working or Ongoing values/`
- ✅ Follow standard pin configuration
- ✅ Test incrementally

### When Asked to Modify Code:

1. **Identify which version** to modify (latest is v2)
2. **Read the relevant section** in documentation first
3. **Check existing implementations** for similar functionality
4. **Preserve standard pin configuration**
5. **Test logic carefully** (especially sensor direction!)
6. **Add clear comments** explaining sensor behavior

---

## Quick Reference: Sensor Direction Logic

```cpp
// ✅ CORRECT - Target zombies approaching sensor
if (sensors[i].rawValue > THRESHOLD && sensors[i].direction == APPROACHING) {
  // Sensor value INCREASING = zombie coming toward us = GOOD TARGET
}

// ❌ WRONG - This targets zombies that are already passing!
if (sensors[i].rawValue > THRESHOLD && sensors[i].direction == LEAVING) {
  // Sensor value DECREASING = zombie moving away = TOO LATE
}
```

**Remember:** We want zombies that are COMING TOWARD the sensor (value increasing), NOT zombies that are LEAVING the sensor (value decreasing).

---

## Version History

- **v1:** Initial competition code (no setup mode)
- **v2:** Added setup mode for configuration before competition
- **Latest:** `ME350_Competition_StateMachine_v2.ino`

---

## Contact/Resources

- **ME 350 Course Materials:** See `Project Information/` folder
- **PID Theory:** `General Reference Documents/ME_350__PID_Controllers_11_16_19.pdf`
- **Example Code:** `Example Code/ME350_LinkageCode_V5.1 (1).pdf`
- **System Config:** `System Documentation/ME350_System_Configuration.md`

---

**Last Updated:** November 2024
**AI Assistant:** Follow this guide to avoid common mistakes and work efficiently on this project.
