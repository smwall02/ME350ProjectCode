# ME350 PID Auto-Tune System - User Guide

## Overview

This PID auto-tuning tool provides a systematic approach to finding optimal PID controller gains for the ME350 mechatronics project. It implements the industry-standard Ziegler-Nichols relay oscillation method along with comprehensive system characterization.

## Features

- ✅ **Automatic Range Calibration**: Finds limit switch positions and total travel range
- ✅ **Directional Friction Characterization**: Measures static friction in both directions
- ✅ **Ziegler-Nichols Auto-Tuning**: Relay oscillation method with multiple tuning presets
- ✅ **EEPROM Storage**: Persistent calibration data across power cycles
- ✅ **Step Response Analysis**: System dynamics identification
- ✅ **Manual Position Testing**: Verify PID performance before deploying
- ✅ **Standardized Pin Configuration**: Matches all ME350 project code

## Hardware Requirements

### Pin Configuration

This code uses the standard ME350 pin assignment:

| Component | Pin | Notes |
|-----------|-----|-------|
| Encoder A | 2 | Interrupt-capable |
| Encoder B | 3 | Interrupt-capable |
| Motor PWM | 11 | ENA on H-bridge |
| Motor Direction 1 | 12 | IN2 on H-bridge |
| Motor Direction 2 | 13 | IN3 on H-bridge |
| Left Limit Switch | 8 | INPUT_PULLUP, active LOW |
| Right Limit Switch | 9 | INPUT_PULLUP, active LOW |

### Required Libraries

```cpp
#include <Encoder.h>  // Paul Stoffregen's Encoder library
#include <EEPROM.h>   // Built-in Arduino EEPROM library
```

**Install Encoder library:**
1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for "Encoder" by Paul Stoffregen
4. Click Install

## Quick Start Guide

### Step 1: Upload Code

1. Open `PID_AutoTune_ME350.ino` in Arduino IDE
2. Select your board (Arduino Uno/Mega)
3. Upload to your Arduino
4. Open Serial Monitor at **115200 baud**

### Step 2: Initial Calibration Sequence

Run these commands in order:

```
R  → Calibrate Range
F  → Characterize Friction
Z  → Auto-Tune PID
T  → Test Position Control
```

### Step 3: Workflow Details

#### Command: R - Range Calibration

**What it does:**
- Moves to left limit switch
- Zeros encoder position
- Moves to right limit switch
- Calculates total range
- Saves to EEPROM

**Expected output:**
```
Left limit found, encoder zeroed.
Right limit found at: -1834 counts
Total range: 1834 encoder counts
Range calibration complete!
```

**Troubleshooting:**
- If "Timeout" error: Check limit switch wiring
- If "Range too small": Verify both limit switches working

#### Command: F - Friction Characterization

**What it does:**
- Measures minimum voltage to overcome static friction moving RIGHT
- Measures minimum voltage to overcome static friction moving LEFT
- Saves values to EEPROM

**Expected output:**
```
RIGHT friction voltage: 1.55 V
LEFT friction voltage: 2.90 V
```

**Why this matters:**
Friction compensation is added to the PID output to overcome static friction, reducing steady-state error and improving response.

#### Command: Z - Ziegler-Nichols Auto-Tune

**What it does:**
1. Moves to center of range
2. Applies relay control (±5V)
3. System oscillates around center
4. Collects 16 oscillation peaks
5. Calculates ultimate gain (Ku) and period (Tu)
6. Offers tuning presets

**Expected output:**
```
Peaks collected: 20
Average amplitude: 156 encoder counts
Ultimate period (Tu): 1.234 seconds
Ultimate gain (Ku): 0.1234

=== TUNING OPTIONS ===
1. Conservative (30% of ZN) - RECOMMENDED
2. Classic Ziegler-Nichols (Full ZN)
3. Aggressive (80% of ZN)
4. PD-Only (No integral)
5. Cancel
```

**Tuning Preset Guide:**

| Preset | Use Case | Performance | Stability |
|--------|----------|-------------|-----------|
| **Conservative** | Initial testing, precise positioning | Settling: 2-4s, Overshoot: <10% | Very High ✅ |
| **Classic ZN** | General purpose | Settling: 1-2s, Overshoot: ~25% | Moderate |
| **Aggressive** | Speed-critical | Settling: <1s, Overshoot: ~40% | Lower ⚠️ |
| **PD-Only** | Variable loads, anti-windup | Settling: varies, Steady-state error possible | High |

**Recommendation:** Start with **Conservative** (option 1). This provides stable, reliable control with minimal overshoot.

**Troubleshooting:**
- **"Insufficient peaks"**:
  - Run friction characterization first (F command)
  - Check for mechanical binding
  - Verify encoder connections
- **Irregular oscillations**:
  - Check motor power supply
  - Verify limit switches not interfering
- **Very large/small Ku**:
  - May indicate measurement error
  - Re-run calibration (R and F commands)

#### Command: T - Manual Position Test

**What it does:**
- Prompts for target encoder position
- Runs PID controller for 10 seconds
- Logs position, error, integral, voltage every 100ms
- Calculates settling time

**Example usage:**
```
Enter target position (-1834 to 0):
-500

Time(s),Position,Error,Integral,Voltage
0.10,-10,490,49.00,12.345
0.20,-45,455,95.50,11.234
0.30,-98,402,135.50,10.123
...
2.50,-498,2,0.20,2.950
2.60,-500,0,0.00,2.900

=== TEST COMPLETE ===
Final position: -500
Final error: 0
Settling time: 2.35 seconds
```

**What to look for:**
- ✅ **Good**: Smooth approach, minimal overshoot, settles quickly
- ⚠️ **Warning**: Large overshoot (>30%), slow settling (>5s)
- ❌ **Problem**: Oscillation continues, doesn't reach target

## All Commands Reference

| Command | Function | When to Use |
|---------|----------|-------------|
| **R** | Calibrate Range | First time setup, after mechanical changes |
| **F** | Friction Characterization | After range calibration, if friction changes |
| **Z** | Ziegler-Nichols Auto-Tune | To find optimal PID gains |
| **S** | Step Response | Advanced: system identification |
| **T** | Manual Position Test | Verify PID performance |
| **H** | Home to Left | Return to zero position |
| **P** | Print Status | View all current settings |
| **C** | Clear Calibration | Reset EEPROM to defaults |
| **?** | Help | Show command list |

## Understanding the Results

### Good Auto-Tune Indicators

✅ **16+ peaks collected** without timeout
✅ **Consistent oscillation amplitude** (peaks within 20% of average)
✅ **Ku between 0.05 - 2.0** (typical range for this system)
✅ **Tu between 0.5 - 3.0 seconds** (reasonable period)

### Problem Indicators

❌ **Timeout before collecting peaks** → Check friction characterization
❌ **Very large Ku (>5.0)** → System may be under-damped, check mechanical
❌ **Very small Ku (<0.01)** → Measurement error, re-run calibration
❌ **Irregular oscillations** → Mechanical binding or encoder issues

## Integrating Results into Your Project

After successful auto-tuning, copy the PID gains and friction values to your main competition code:

### Step 1: Note Your Values

From the Serial Monitor after auto-tuning:

```
=== NEW PID GAINS ===
Kp = 0.018765
Ki = 0.003421
Kd = 0.002145

=== FRICTION COMPENSATION ===
LEFT (moving RIGHT): 1.550 V
RIGHT (moving LEFT): 2.900 V
```

### Step 2: Copy to Your Code

```cpp
// In your main ME350 competition code:

// PID Gains (from auto-tune)
float KP = 0.018765;  // From auto-tune result
float KI = 0.003421;  // From auto-tune result
float KD = 0.002145;  // From auto-tune result

// Friction Compensation (from characterization)
float FRICTION_LEFT = 1.550;   // Moving RIGHT (negative direction)
float FRICTION_RIGHT = 2.900;  // Moving LEFT (positive direction)

// In your PID control function:
float frictionComp = 0;
if (error < -DEADBAND) {
  frictionComp = -FRICTION_LEFT;  // Moving RIGHT
}
else if (error > DEADBAND) {
  frictionComp = FRICTION_RIGHT;  // Moving LEFT
}

float voltage = pidOutput + frictionComp;
```

## Advanced Usage

### Step Response Analysis (S Command)

The step response command applies a constant 5V input and records position over time. This data can be used to:

- Estimate system time constant
- Calculate DC gain
- Verify motor/encoder operation
- Generate plots for analysis

**Output format:**
```
Time(ms),Position(counts)
0,0
50,-12
100,-28
150,-47
...
```

You can copy this data into Excel or Python for plotting.

### Modifying Tuning Parameters

If you need to adjust the auto-tune algorithm, edit these constants in the code:

```cpp
const float TEST_VOLTAGE = 5.0;     // Relay amplitude (line 467)
const long HYSTERESIS = TOTAL_RANGE / 6;  // Oscillation band (line 468)
const int REQUIRED_PEAKS = 16;      // Minimum peaks (line 469)
const unsigned long TIMEOUT = 60000; // Max test time (line 470)
```

**When to modify:**
- System not oscillating → Increase `TEST_VOLTAGE`
- Oscillations too large → Decrease `TEST_VOLTAGE` or `HYSTERESIS`
- Takes too long → Decrease `REQUIRED_PEAKS` (minimum 8 recommended)

## Troubleshooting Guide

### Problem: "ERROR: Must calibrate range first"

**Solution:** Run the `R` command before attempting other operations.

### Problem: Motor doesn't move during calibration

**Possible causes:**
- Motor power supply not connected
- H-bridge not powered
- Wiring incorrect (IN2/IN3 swapped)
- Motor driver fault

**Debug steps:**
1. Verify 12V power to motor driver
2. Check Arduino 5V to driver logic
3. Verify pin connections (11, 12, 13)
4. Test motor manually with simple voltage

### Problem: Encoder count doesn't change

**Possible causes:**
- Encoder not powered (needs 5V)
- A/B channels not connected
- Encoder damaged

**Debug steps:**
1. Verify 5V to encoder
2. Check pins 2 and 3 connections
3. Manually rotate motor, watch Serial Monitor position

### Problem: Limit switches don't work

**Configuration:** Active LOW with `INPUT_PULLUP`
- When **pressed**: reads LOW (0)
- When **not pressed**: reads HIGH (1)

**Debug steps:**
1. Test with multimeter (should be normally open)
2. Verify pin 8 and 9 connections
3. Check switch mechanical operation
4. Measure voltage at pins (should be 5V when open, 0V when closed)

### Problem: Auto-tune oscillations hit limit switches

**Solution:**
- The oscillation amplitude is too large
- Reduce `TEST_VOLTAGE` in the code (try 3.0V instead of 5.0V)
- Or reduce `HYSTERESIS` to make oscillation band smaller

### Problem: Results vary between runs

**Possible causes:**
- Mechanical friction varies (stiction)
- Encoder position drift
- Temperature changes

**Solutions:**
1. Run auto-tune multiple times, average results
2. Ensure system is at steady-state temperature
3. Check for mechanical binding or loose components
4. Re-characterize friction (F command)

## PID Tuning Theory

### How Ziegler-Nichols Works

The relay oscillation method:

1. **System oscillates** with on/off control (relay)
2. **Measures natural frequency** (period Tu)
3. **Measures amplitude** of oscillation
4. **Calculates ultimate gain** Ku = 4V/(π × amplitude)
5. **Applies ZN formulas**:
   - Kp = 0.6 × Ku
   - Ki = 1.2 × Ku / Tu
   - Kd = 0.075 × Ku × Tu

### Fine-Tuning After Auto-Tune

If auto-tune results aren't perfect, manually adjust:

**Too much overshoot:**
```cpp
KP = KP * 0.8;  // Reduce by 20%
KD = KD * 1.5;  // Increase by 50%
```

**Too slow response:**
```cpp
KP = KP * 1.2;  // Increase by 20%
KD = KD * 0.8;  // Decrease by 20%
```

**Continuous oscillation:**
```cpp
KP = KP * 0.7;  // Reduce by 30%
KI = KI * 0.5;  // Reduce by 50%
KD = KD * 1.2;  // Increase by 20%
```

**Steady-state error:**
```cpp
KI = KI * 2.0;  // Double integral gain
// OR increase friction compensation
```

## Safety Features

The code includes multiple safety features:

✅ **Limit switch monitoring** - Motor stops if limit hit during operation
✅ **Voltage limiting** - Output constrained to ±10V
✅ **Timeout protection** - Auto-tune won't run forever
✅ **Range validation** - Prevents targeting positions beyond limits
✅ **EEPROM magic number** - Prevents loading corrupt data

## EEPROM Data Persistence

All calibration data is automatically saved and loaded:

**Saved data:**
- PID gains (Kp, Ki, Kd)
- Friction values (left and right)
- Range limits (left and right positions)
- Calibration validity flag

**To reset:** Use the `C` command to clear EEPROM and restore defaults.

## Performance Expectations

### Conservative Tuning (Recommended)

- **Settling time:** 2-4 seconds
- **Overshoot:** <10%
- **Steady-state error:** <5 encoder counts
- **Stability:** Very high (won't oscillate)

**Best for:** Initial testing, competition use, reliable operation

### Classic Ziegler-Nichols

- **Settling time:** 1-2 seconds
- **Overshoot:** ~25%
- **Steady-state error:** <2 encoder counts
- **Stability:** Moderate (may oscillate slightly)

**Best for:** Balanced performance when you need faster response

### Aggressive Tuning

- **Settling time:** <1 second
- **Overshoot:** ~40%
- **Steady-state error:** <2 encoder counts
- **Stability:** Lower (requires careful testing)

**Best for:** Speed-critical applications, experienced users

## Example Session

Here's a complete tuning session from start to finish:

```
========================================
   ME350 PID AUTO-TUNE SYSTEM
========================================
System not calibrated.
Run 'R' command to calibrate range.

> R
=== RANGE CALIBRATION ===
Finding left and right limit positions...
Moving to left limit...
Left limit found, encoder zeroed.
Moving to right limit...
Right limit found at: -1834 counts
Total range: 1834 encoder counts
Range calibration complete!

> F
=== FRICTION CHARACTERIZATION ===
Measuring friction for RIGHT movement...
RIGHT friction voltage: 1.55 V
Measuring friction for LEFT movement...
LEFT friction voltage: 2.90 V
=== Friction Characterization Complete ===

> Z
=== ZIEGLER-NICHOLS AUTO-TUNE ===
Press 'Y' to continue...
> Y
Moving to center position: -917
Starting relay oscillation test...
Peaks collected: 4
Peaks collected: 8
Peaks collected: 12
Peaks collected: 16
Peaks collected: 20

=== AUTO-TUNE RESULTS ===
Peaks collected: 16
Average amplitude: 156 encoder counts
Ultimate period (Tu): 1.234 seconds
Ultimate gain (Ku): 0.1234

Select tuning method (1-5):
> 1
Applying CONSERVATIVE gains (30% of ZN)...

=== NEW PID GAINS ===
Kp = 0.022248
Ki = 0.007236
Kd = 0.001707

Gains saved to EEPROM.
Use 'T' command to test these gains.

> T
Enter target position (-1834 to 0):
> -500

Moving to position: -500
Time(s),Position,Error,Integral,Voltage
0.10,-10,490,49.00,12.345
...
2.60,-500,0,0.00,2.900

=== TEST COMPLETE ===
Final position: -500
Final error: 0
Settling time: 2.35 seconds
```

## Tips for Best Results

1. **Run calibration in order**: R → F → Z → T
2. **Start conservative**: Use Conservative tuning first
3. **Test thoroughly**: Run multiple position tests before using in competition
4. **Re-calibrate if needed**: If mechanism changes, re-run R and F
5. **Save your results**: Write down final gains before power cycling
6. **Check EEPROM loading**: Use P command after reset to verify data loaded

## Support and Documentation

For more information, see:
- `System Documentation/ME350_System_Configuration.md` - Complete hardware and software reference
- `PID_AutoTune_Enhanced/ANALYSIS.md` - Detailed code analysis
- ME350 course materials - PID theory and control fundamentals

## Version Information

**Version:** 1.0
**Compatible with:** Arduino Uno, Mega
**Encoder Library:** Paul Stoffregen's Encoder library
**Tested with:** ME350 standard motor/gearbox setup (30:1, 64 CPR)
