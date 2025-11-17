# ME350 Competition State Machine - User Guide

## Overview

This is the main competition code for the ME350 "Plants vs Zombies" project. It implements a three-state machine with support for the full three-round competition structure, adaptive target selection, and PID position control.

## State Machine Architecture

### Three States

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

### State Descriptions

#### 1. CALIBRATE
- **Purpose**: Find left limit switch and zero encoder position
- **Actions**:
  - Apply constant +5V to move LEFT
  - Wait for left limit switch activation
  - Check for zero velocity (motor stopped)
  - Zero the encoder position
  - Transition to CHOOSE_ACTIVE_TARGET
- **Runs**: Once at startup

#### 2. CHOOSE_ACTIVE_TARGET
- **Purpose**: Select which zombie to intercept
- **Logic**:
  - Read all 4 proximity sensors
  - Determine direction (FORWARD/BACKWARD/STOPPED) for each zombie
  - Calculate distance from current position to each target
  - **Priority**: Select closest FORWARD-moving zombie
  - If no forward-moving zombies, go to wait position
- **Runs**: After calibration and after each target engagement

#### 3. MOVE_TO_TARGET
- **Purpose**: Execute PID control to reach target
- **Actions**:
  - Run PID controller at 100 Hz
  - Monitor proximity sensor for zombie activation (LED lights)
  - Allow dynamic target switching if closer zombie appears
  - If zombie activates LED: increment score, return to CHOOSE_ACTIVE_TARGET
  - If position reached with no activation: return to CHOOSE_ACTIVE_TARGET
- **Runs**: Continuously until target hit or timeout

## Competition Rounds

### Round 1: Standard Play
- **Duration**: 40 seconds
- **Speed**: Normal zombie speed
- **Scoring**: LED activation OR front limit switch = 1 point
- **End Condition**: Time expires

### Round 2: Fast Play
- **Duration**: 40 seconds
- **Speed**: Faster zombie speed
- **Scoring**: LED activation OR front limit switch = 1 point
- **End Condition**: Time expires

### Round 3: Survival Mode
- **Duration**: Until first zombie reaches front limit
- **Speed**: Increases progressively
- **Scoring**: LED activation ONLY = 1 point
- **End Condition**: Front limit switch hit (FAILURE)

**CRITICAL**: In Round 3, the front limit switch **ends the game**, it does NOT count as a successful hit!

## Setup Instructions

### 1. Install Required Libraries

```cpp
#include <Encoder.h>  // Paul Stoffregen's Encoder library
```

**Installation:**
- Open Arduino IDE
- Sketch → Include Library → Manage Libraries
- Search "Encoder" by Paul Stoffregen
- Click Install

### 2. Configure PID Gains

**IMPORTANT**: You must run the PID auto-tune code first!

1. Upload and run `PID_AutoTune_ME350.ino`
2. Follow the calibration sequence (R → F → Z → T)
3. Note the final PID gains from auto-tune
4. Copy the values into this competition code:

```cpp
// Line 103-105 in ME350_Competition_StateMachine.ino
float KP = 0.020;  // Replace with your tuned Kp
float KI = 0.005;  // Replace with your tuned Ki
float KD = 0.004;  // Replace with your tuned Kd
```

### 3. Configure Friction Compensation

Copy friction values from auto-tune results:

```cpp
// Line 120-121 in ME350_Competition_StateMachine.ino
float FRICTION_LEFT = 2.2;   // Replace with your measured value
float FRICTION_RIGHT = 0.25; // Replace with your measured value
```

### 4. Configure Target Positions

After running the PID auto-tune calibration, you'll know your encoder range. Adjust target positions based on where your proximity sensors are mounted:

```cpp
// Line 77-80 in ME350_Competition_StateMachine.ino
long TARGET_1_POSITION = -74;    // Closest to left
long TARGET_2_POSITION = -307;   // Second position
long TARGET_3_POSITION = -547;   // Third position
long TARGET_4_POSITION = -1080;  // Farthest right
```

**How to determine these values:**
1. Manually position your mechanism at each sensor location
2. Read the encoder value (use PID auto-tune 'P' command)
3. Record the position for each target
4. Update the code with actual values

### 5. Set Wait Position

This is where the mechanism waits when no targets are active:

```cpp
// Line 82 in ME350_Competition_StateMachine.ino
long WAIT_POSITION = TARGET_3_POSITION;  // Usually center position
```

## Testing Before Competition

### Pre-Competition Checklist

- [ ] PID gains tuned using auto-tune code
- [ ] Friction compensation measured
- [ ] Target positions calibrated
- [ ] All 4 proximity sensors working
- [ ] Both limit switches functional
- [ ] Encoder counting correctly (no drift)
- [ ] Motor direction correct (positive = LEFT)
- [ ] Serial monitor shows proper state transitions
- [ ] Tested with moving objects at all 4 positions

### Test Procedure

1. **Upload Code**
   - Open `ME350_Competition_StateMachine.ino`
   - Upload to Arduino
   - Open Serial Monitor at 115200 baud

2. **Verify Calibration**
   - Watch for "CALIBRATION COMPLETE" message
   - Verify encoder reads 0 at left limit

3. **Test Sensor Detection**
   - Wave hand in front of each sensor
   - Watch Serial Monitor sensor values
   - Should see values increase when object approaches
   - Should detect FORWARD direction

4. **Test Target Engagement**
   - Move object toward sensor (FORWARD)
   - Mechanism should move to that position
   - Hold object near sensor to simulate LED activation
   - Should print "*** HIT! Target X ***"
   - Should increment score

5. **Test Round Transitions**
   - Let Round 1 run for 40 seconds
   - Should print "ROUND 1 COMPLETE"
   - Should transition to Round 2
   - Repeat for Round 2
   - Round 3 should print "LED ONLY" warning

## Understanding the Code

### Key Features

#### 1. Direction Detection

The code uses a low-pass filter and derivative to determine if zombies are moving FORWARD (approaching sensor) or BACKWARD (moving away):

```cpp
const float ALPHA = 0.925;  // Filter coefficient (higher = more smoothing)

// In updateSensors():
sensors[i].filteredValue = ALPHA * sensors[i].filteredValue + (1.0 - ALPHA) * raw;
float derivative = sensors[i].filteredValue - sensors[i].lastFilteredValue;

if (derivative > 2.0) {
  sensors[i].direction = FORWARD;
}
```

**Why this matters:** The code only targets zombies moving FORWARD to avoid wasting time on zombies that are moving away.

#### 2. Adaptive Friction Boost

If the mechanism reaches a target position but doesn't get an activation (zombie missed), it increases friction compensation for the next move:

```cpp
if (millis() - positionReachedTime > WAIT_TIME) {
  adaptiveFrictionLeft += FRICTION_BOOST_AMOUNT;
  adaptiveFrictionRight += FRICTION_BOOST_AMOUNT;
}
```

This helps overcome static friction on subsequent moves.

#### 3. Dynamic Target Switching

While moving to a target, the code continuously checks if a closer FORWARD-moving zombie appears:

```cpp
void checkForBetterTarget() {
  // If new target is >20% closer, switch to it
  if (newTargetDistance < currentTargetDistance * 0.8) {
    currentTargetPosition = targetPositions[i];
    activeTarget = i;
  }
}
```

This maximizes scoring opportunities by always prioritizing the closest threat.

#### 4. Round-Specific Scoring

The code handles different rules for each round:

```cpp
void recordHit() {
  switch (currentRound) {
    case ROUND_1:
      round1Score++;
      break;
    case ROUND_2:
      round2Score++;
      break;
    case ROUND_3:
      // ONLY LED counts in Round 3
      round3Score++;
      break;
  }
}
```

## Serial Monitor Output

### During Operation

You'll see periodic status updates every 2 seconds:

```
Round 1 | State: MOVE_TO_TARGET | Pos: -315 | Target: 2 (-307) | Score: 5 | Sensors: 123,456,789,234
```

**Reading the output:**
- **Round**: Current competition round (1, 2, or 3)
- **State**: Current state machine state
- **Pos**: Current encoder position
- **Target**: Active target number and position
- **Score**: Total score so far
- **Sensors**: Raw values from all 4 sensors (0-1023)

### Events

Key events are printed immediately:

```
Target selected: 3 at position -547
*** HIT! Target 3 ***
Switching target: 2 -> 1
Position reached, no activation. Choosing new target.

========================================
=== ROUND 1 COMPLETE ===
Score: 12
========================================
```

### Final Score

At the end of Round 3:

```
========================================
=== FINAL SCORE ===
========================================
Round 1: 12
Round 2: 15
Round 3: 8
----------------------------------------
TOTAL:   35
========================================
```

## Tuning Parameters

### Sensor Detection

```cpp
const int ACTIVATION_THRESHOLD = 400;  // Line 140
```
- Increase if sensors are too sensitive (false triggers)
- Decrease if zombies aren't being detected
- Typical range: 300-600

### Direction Detection Sensitivity

```cpp
const float ALPHA = 0.925;  // Line 137
```
- Higher (closer to 1.0) = more filtering, slower response
- Lower (closer to 0.0) = less filtering, faster but noisier
- Typical range: 0.85-0.95

### Derivative Threshold

```cpp
if (derivative > 2.0) {  // Line 385
  sensors[i].direction = FORWARD;
}
```
- Increase if getting false FORWARD detections
- Decrease if not detecting FORWARD movement
- Typical range: 1.0-5.0

### PID Deadband

```cpp
const long DEADBAND = 5;  // Line 111
```
- Larger = more tolerance for position error
- Smaller = more precise positioning
- Typical range: 3-10 encoder counts

### Wait Time at Target

```cpp
const unsigned long WAIT_TIME = 1000;  // Line 168 (milliseconds)
```
- How long to wait at target position before giving up
- Increase if zombies are slow to activate LED
- Decrease for faster target switching
- Typical range: 500-2000ms

### Target Switch Cooldown

```cpp
const unsigned long TARGET_SWITCH_COOLDOWN = 200;  // Line 171 (milliseconds)
```
- Minimum time between target switches
- Prevents rapid switching oscillations
- Typical range: 100-500ms

### Friction Boost

```cpp
const float FRICTION_BOOST_AMOUNT = 0.2;  // Line 123 (volts)
```
- How much to increase friction on missed targets
- Increase if mechanism gets stuck often
- Decrease if overshooting after misses
- Typical range: 0.1-0.5V

## Troubleshooting

### Problem: Mechanism doesn't calibrate

**Symptoms:**
- Stays in CALIBRATE state
- Motor doesn't move
- Never reaches left limit

**Solutions:**
1. Check motor wiring (pins 11, 12, 13)
2. Verify motor power supply (12V)
3. Test limit switch manually (should read LOW when pressed)
4. Check H-bridge enable (ENA pin 11)

### Problem: Doesn't detect zombies

**Symptoms:**
- State stuck in CHOOSE_ACTIVE_TARGET
- Always goes to wait position
- Sensor values don't change

**Solutions:**
1. Check proximity sensor wiring (A0-A3)
2. Lower `ACTIVATION_THRESHOLD` (try 200)
3. Verify sensor power (5V)
4. Print sensor values to debug:
   ```cpp
   Serial.print("Sensor 1: ");
   Serial.println(sensors[0].rawValue);
   ```

### Problem: Wrong target selected

**Symptoms:**
- Moves to wrong position
- Targets BACKWARD-moving zombies
- Ignores closest zombie

**Solutions:**
1. Check target position values (may be swapped)
2. Verify sensor numbering (Sensor 1 = A0, etc.)
3. Adjust direction detection threshold
4. Print direction values to debug

### Problem: Doesn't score hits

**Symptoms:**
- Reaches target but no "HIT!" message
- Score doesn't increment
- Stays at position indefinitely

**Solutions:**
1. Check if sensor value exceeds `ACTIVATION_THRESHOLD`
2. Verify zombie actually triggers LED
3. Print sensor value at target position:
   ```cpp
   Serial.print("Active sensor: ");
   Serial.println(sensors[activeTarget].rawValue);
   ```

### Problem: Mechanism oscillates

**Symptoms:**
- Never settles at target
- Continuous back-and-forth motion
- Error doesn't go to zero

**Solutions:**
1. Reduce Kp (by 20%)
2. Reduce Ki (by 50%)
3. Increase Kd (by 20%)
4. Or re-run auto-tune with Conservative preset

### Problem: Slow response

**Symptoms:**
- Takes >3 seconds to reach target
- Zombies escape before interception
- Loses points to timeouts

**Solutions:**
1. Increase Kp (by 20%)
2. Re-run auto-tune with Classic or Aggressive preset
3. Check for mechanical binding
4. Verify friction compensation is correct

### Problem: Round 3 ends immediately

**Symptoms:**
- Round 3 starts then immediately shows "FAILED"
- No time to intercept zombies

**Cause:** Front limit switch is already pressed or wiring issue

**Solutions:**
1. Verify right limit switch is NOT pressed at Round 3 start
2. Check limit switch wiring (should be INPUT_PULLUP, active LOW)
3. Ensure mechanism is not at right limit when Round 3 begins

## Code Modifications

### Change Round Durations

```cpp
// Line 59-60
const unsigned long ROUND_1_DURATION = 40000;  // 40 seconds
const unsigned long ROUND_2_DURATION = 40000;  // 40 seconds
```

Change these values to adjust round lengths (in milliseconds).

### Disable Round Structure (Continuous Operation)

Comment out the round transition check:

```cpp
void loop() {
  // ...
  // checkRoundTransition();  // Commented out
}
```

### Add Additional Sensors

To add more than 4 sensors:

1. Define additional pins:
   ```cpp
   #define PROX_SENSOR_5 A4
   ```

2. Increase array sizes:
   ```cpp
   SensorData sensors[5];  // Was 4
   long targetPositions[5];  // Was 4
   ```

3. Update loops:
   ```cpp
   for (int i = 0; i < 5; i++) {  // Was 4
   ```

### Change Status Print Interval

```cpp
// Line 178
const unsigned long STATUS_PRINT_INTERVAL = 2000;  // milliseconds
```

Reduce for more frequent updates, increase to reduce serial spam.

## Performance Tips

1. **Optimize sensor placement**: Mount sensors at positions that give maximum warning time
2. **Tune aggressively for Round 2**: May want separate PID gains for faster zombies
3. **Center your wait position**: Minimizes worst-case travel distance
4. **Test direction detection**: Adjust derivative threshold for your zombie speed
5. **Monitor serial output**: Watch for patterns in missed targets

## Integration with Auto-Tune Code

### Recommended Workflow

1. **Run PID auto-tune first** (`PID_AutoTune_ME350.ino`)
   - Command: R (calibrate range)
   - Command: F (characterize friction)
   - Command: Z (auto-tune PID, select Conservative)
   - Command: T (test gains)

2. **Record calibration values**:
   - Note Kp, Ki, Kd
   - Note FRICTION_LEFT, FRICTION_RIGHT
   - Note LEFT_LIMIT, RIGHT_LIMIT

3. **Measure target positions**:
   - Place mechanism at each sensor
   - Use auto-tune 'H' command to check position
   - Record encoder value for each target

4. **Update competition code**:
   - Copy PID gains (lines 103-105)
   - Copy friction values (lines 120-121)
   - Update target positions (lines 77-80)

5. **Upload and test**:
   - Upload competition code
   - Verify calibration
   - Test each sensor/target
   - Run full competition simulation

## Additional Resources

- **System Configuration**: See `System Documentation/ME350_System_Configuration.md`
- **PID Tuning Guide**: See `In Progress/PID_Tuning_Improved/README.md`
- **Project Description**: See original competition rules PDF
- **State Machine Tutorial**: See Lab 11 documentation

## Version Information

**Version:** 1.0
**Compatible with:** Arduino Uno, Mega
**Required Libraries:** Encoder (Paul Stoffregen)
**Control Frequency:** 100 Hz (10ms period)
**Based on:** ME350 example code + best practices from existing implementations
