# Enhanced PID Auto-Tuning System for ME 350 Project

## Overview

This Arduino sketch provides comprehensive PID controller tuning and system identification for the ME 350 mechatronics project. It implements multiple industry-standard tuning methods and diagnostic tools.

## Features

### 1. **Range Calibration**
- Automatically finds left and right limit switch positions
- Calculates total encoder travel range
- Stores limits in EEPROM for persistence

### 2. **Friction Characterization**
- Determines minimum voltage needed to overcome friction in both directions
- Accounts for directional asymmetry (LEFT vs RIGHT movement)
- Critical for accurate PID control

### 3. **Ziegler-Nichols Auto-Tune**
- Relay-based oscillation method
- Automatically calculates ultimate gain (Ku) and period (Tu)
- Provides four tuning options:
  - **Classic Ziegler-Nichols**: Full ZN gains
  - **Conservative (Recommended)**: 30% of ZN for stable operation
  - **Aggressive**: 80% of ZN for faster response
  - **PD-Only**: No integral term to avoid windup

### 4. **Step Response Analysis**
- Applies step voltage input
- Records position over time
- Estimates system time constant and gain
- Useful for understanding system dynamics

### 5. **Manual Position Testing**
- Test current PID gains with specific position targets
- Real-time monitoring of position, error, and integral term
- Measures settling time and final accuracy

### 6. **EEPROM Storage**
- Saves all calibration data persistently
- Automatic loading on startup
- Can be cleared and recalibrated

## Hardware Setup

### Pin Configuration (Standard across project)

```
ENCODER:
- Encoder A: Pin 2
- Encoder B: Pin 3

MOTOR CONTROL:
- ENA (PWM): Pin 11
- IN2: Pin 12
- IN3: Pin 13

LIMIT SWITCHES:
- Left Limit: Pin 8 (INPUT_PULLUP)
- Right Limit: Pin 9 (INPUT_PULLUP)

PROXIMITY SENSORS (optional):
- Sensor 1: A0
- Sensor 2: A1
- Sensor 3: A2
- Sensor 4: A3
```

### Motor Direction Convention
- **Positive Voltage** → Move LEFT (toward encoder position 0)
- **Negative Voltage** → Move RIGHT (toward more negative positions)

## Usage Instructions

### Initial Setup (First Time)

1. **Upload the sketch** to your Arduino
2. **Open Serial Monitor** at 115200 baud
3. **Run these commands in order:**
   ```
   R - Calibrate Range
   F - Characterize Friction
   Z - Ziegler-Nichols Auto-Tune
   T - Test with Manual Position
   ```

### Command Reference

| Command | Function | Description |
|---------|----------|-------------|
| `R` | Calibrate Range | Find left/right limits and total travel |
| `F` | Friction Characterization | Measure breakaway voltages |
| `Z` | Ziegler-Nichols Tune | Auto-tune PID using oscillation method |
| `S` | Step Response | Analyze system dynamics with step input |
| `T` | Manual Test | Test current gains with target position |
| `H` | Home | Return to left limit and zero encoder |
| `P` | Print Status | Display all current settings and state |
| `C` | Clear Calibration | Erase EEPROM saved data |
| `?` | Help | Show command list |

## Auto-Tuning Process Explained

### Ziegler-Nichols Relay Method

1. **System moves to center position**
2. **Applies relay control** with hysteresis band
3. **Oscillations develop** around center
4. **Collects 16 peaks** (skips first 4 for settling)
5. **Calculates:**
   - Amplitude of oscillation
   - Average period (Tu)
   - Ultimate gain (Ku = 4V / (π × amplitude))

6. **Computes PID gains** using ZN formulas:
   - **Kp = 0.6 × Ku**
   - **Ki = 1.2 × Ku / Tu**
   - **Kd = 0.075 × Ku × Tu**

### Tuning Options

#### Conservative (Recommended for initial testing)
- Uses 30% of ZN gains
- Very stable, minimal overshoot
- Slower response time
- Best for: Initial testing, critical positioning

#### Classic Ziegler-Nichols
- Full ZN formula gains
- Moderate overshoot (~25%)
- Good balance of speed and stability
- Best for: General purpose control

#### Aggressive
- Uses 80% of ZN gains
- Fast response
- More overshoot
- Best for: Speed-critical applications

#### PD-Only
- No integral term
- Prevents integral windup
- May have steady-state error
- Best for: Systems with changing loads

## Understanding Results

### Good Auto-Tune Indicators
- ✓ Collects 16+ peaks without timeout
- ✓ Consistent oscillation amplitude
- ✓ Ku value between 0.5 - 5.0
- ✓ Period (Tu) between 0.5 - 3.0 seconds

### Problem Indicators
- ✗ Timeout before collecting enough peaks
- ✗ Very large or very small Ku values
- ✗ Irregular oscillations
- ✗ System hitting limits during test

**If problems occur:**
1. Verify friction was characterized first
2. Check mechanical binding
3. Ensure encoder is connected properly
4. Try reducing TEST_VOLTAGE in code (currently 5.0V)

## Interpreting PID Gains

### Proportional (Kp)
- **Higher** → Faster response, more overshoot
- **Lower** → Slower response, less overshoot
- **Typical range:** 0.01 - 0.5

### Integral (Ki)
- **Higher** → Faster elimination of steady-state error, more oscillation
- **Lower** → Slower error elimination, more stable
- **Typical range:** 0.001 - 0.05

### Derivative (Kd)
- **Higher** → More damping, less overshoot
- **Lower** → Less damping, more overshoot
- **Typical range:** 0.001 - 0.02

## Fine-Tuning After Auto-Tune

After auto-tuning, you may want to adjust gains manually:

1. **Too much overshoot?**
   - Decrease Kp by 20%
   - Increase Kd by 50%

2. **Too slow?**
   - Increase Kp by 20%
   - Decrease Kd by 20%

3. **Oscillates continuously?**
   - Decrease Kp by 30%
   - Decrease Ki by 50%
   - Increase Kd by 20%

4. **Steady-state error?**
   - Increase Ki (double current value)
   - Or add friction compensation

## Troubleshooting

### "Range too small" error
- Check that both limit switches are working
- Verify motor can move full range
- Check for mechanical obstructions

### "Insufficient peaks" during auto-tune
- Friction may not be characterized correctly
- TEST_VOLTAGE may be too low
- Check for mechanical binding
- Increase timeout or decrease HYSTERESIS

### "Target not reached" in manual test
- PID gains may need adjustment
- Friction compensation may be incorrect
- Check for mechanical resistance
- Verify encoder is counting correctly

### Encoder count drift
- Check encoder wiring
- Verify power supply is stable
- Run homing ('H') command to recalibrate

## Safety Features

- **Limit switch monitoring** - Auto-stop at limits
- **Timeout protection** - Prevents infinite loops
- **Voltage limiting** - Max ±10V to motor
- **Position bounds checking** - Prevents runaway

## Data Storage

All calibration data is automatically saved to EEPROM:
- PID gains (Kp, Ki, Kd)
- Friction values (Left, Right)
- Range limits (Left, Right)
- Calibration flag

**To reset:** Use 'C' command to clear EEPROM

## Advanced Usage

### Accessing Last Tune Results

The `lastTuneResults` structure stores:
```cpp
- Ku (Ultimate gain)
- Tu (Ultimate period)
- amplitude (Oscillation amplitude)
- peakCount (Number of peaks)
- valid (Results validity flag)
```

View with 'P' (Print Status) command

### Modifying Tuning Parameters

In code, you can adjust:
- `TEST_VOLTAGE` - Relay amplitude (default 5.0V)
- `HYSTERESIS` - Oscillation band (default: range/6)
- `REQUIRED_PEAKS` - Minimum peaks needed (default 16)
- `TIMEOUT` - Maximum test time (default 60s)

## Integration with Other Code

To use the tuned PID gains in your main project:

1. Run auto-tune with this code
2. Note the final Kp, Ki, Kd values
3. Copy these values to your project code
4. Copy FRICTION_LEFT and FRICTION_RIGHT values
5. Use the same friction compensation logic

## References

- **Ziegler-Nichols Tuning:** Classic PID tuning method (1942)
- **ME 350 PID Documentation:** See `Project Information/General Reference Documents/ME_350__PID_Controllers_11_16_19.pdf`
- **Control Theory:** Feedback control fundamentals

## Credits

Enhanced PID Auto-Tuning System
ME 350 Mechatronics Project
Implements industry-standard control algorithms
Compatible with all project motor configurations
