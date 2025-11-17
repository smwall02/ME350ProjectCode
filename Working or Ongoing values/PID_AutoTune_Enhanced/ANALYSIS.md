# Code Analysis and Improvements Summary

## Project Analysis

### Files Reviewed

1. **MotDirLimSwitchDirRangeCode.ino** - Basic PID with auto-tune
2. **UpdatedGameCodeNov17.ino** - Plants vs Zombies game code with state machine
3. **ME350_PID_FINAL_CORRECTED2.ino** - PID final version with corrections
4. **MostRecentTestCodeNov17.ino** - Most sophisticated with EEPROM and adaptive boost
5. **PID_VALUES_DEFINED.ino** - Similar to FINAL_CORRECTED2
6. **ME350_PID_Controller_TargetPractice.ino** - Example code with interrupt-based encoder

### Documentation Reviewed

- **ME_350__PID_Controllers_11_16_19.pdf** - Comprehensive PID theory from U of Michigan
- Lab slides and tutorials (10+ documents)
- Control theory lecture slides

## Standardized Pin Configuration

All modern code files use consistent pinout:

```
ENCODER:        Pins 2 & 3
MOTOR PWM:      Pin 11 (ENA)
MOTOR DIR:      Pins 12 & 13 (IN2, IN3)
LIMIT LEFT:     Pin 8
LIMIT RIGHT:    Pin 9
PROX SENSORS:   A0, A1, A2, A3
```

**Motor Direction Convention:**
- Positive voltage → LEFT (toward position 0)
- Negative voltage → RIGHT (toward negative positions)

## Issues Identified

### 1. Inconsistent PID Values

| File | Kp | Ki | Kd | Notes |
|------|-----|-----|-----|-------|
| MotDirLimSwitch | 0.020 | 0.005 | 0.004 | Basic tuning |
| UpdatedGameCode | 0.020 | 0.005 | 0.004 | Same as above |
| FINAL_CORRECTED2 | 0.020 | 0.005 | 0.004 | Same as above |
| MostRecentTest | 0.28 | 0.015 | 0.012 | Much more aggressive |
| PID_VALUES_DEFINED | 0.020 | 0.005 | 0.004 | Basic again |
| Example Code | 0.1 | 0.0 | 0.0 | P-only control |

**Problem:** No systematic tuning methodology applied consistently.

**Solution:** Enhanced auto-tune code provides systematic ZN method.

### 2. Friction Compensation Variations

| File | Approach | Values |
|------|----------|--------|
| MotDirLimSwitch | Single value | 4.0V |
| UpdatedGameCode | Directional + adaptive | LEFT: 2.2V, RIGHT: 0.25V bias |
| FINAL_CORRECTED2 | Single value | 4.0V |
| MostRecentTest | Directional + calibration | LEFT: 1.55V, RIGHT: 2.9V |
| PID_VALUES_DEFINED | Single value | 4.0V |

**Problem:** Inconsistent friction models, some ignore directional asymmetry.

**Solution:** Enhanced code measures both directions independently.

### 3. Auto-Tune Method Inconsistencies

**Existing Implementations:**

1. **Relay Oscillation Method** (in FINAL_CORRECTED2 and PID_VALUES_DEFINED)
   - Moves to center
   - Creates oscillations with hysteresis
   - Calculates Ku and Tu
   - Applies 35% safety factor
   - **Issues:** Fixed hysteresis, limited tuning options

2. **Manual Calibration** (in MostRecentTest)
   - User-driven position recording
   - No automatic PID calculation
   - **Issues:** Time-consuming, not repeatable

3. **None** (in UpdatedGameCode, Example)
   - Hard-coded values
   - **Issues:** Not optimized for specific system

**Solution:** Enhanced code provides:
- Robust relay method with automatic Ku/Tu calculation
- Four tuning presets (Conservative, Classic, Aggressive, PD-only)
- Step response analysis
- Comprehensive friction characterization

### 4. Encoder Implementation

**Two approaches found:**

1. **Encoder Library** (Modern files)
   - Clean, reliable
   - Handles all edge cases
   - Used in most recent code

2. **Manual Interrupt** (Example code)
   - Bit manipulation
   - Requires careful coding
   - Prone to errors

**Solution:** Enhanced code uses Encoder library (best practice).

### 5. Control Loop Timing

| File | Control Period | Implementation |
|------|----------------|----------------|
| MotDirLimSwitch | 10ms (100 Hz) | Fixed timing with millis() |
| UpdatedGameCode | 10ms (100 Hz) | Fixed timing |
| MostRecentTest | 10ms (100 Hz) | Fixed timing |
| Example Code | Variable | No fixed timing |

**Solution:** Enhanced code maintains 10ms control period (100 Hz).

## Enhanced Auto-Tune Code Features

### Improvements Over Existing Code

1. **Multiple Tuning Methods**
   - Classic Ziegler-Nichols
   - Conservative (30% of ZN) - Recommended for stability
   - Aggressive (80% of ZN) - For speed
   - PD-Only - Prevents integral windup

2. **Comprehensive System Characterization**
   - Range calibration (automatic limit finding)
   - Bidirectional friction measurement
   - Step response analysis
   - System gain identification

3. **Robust Data Collection**
   - Collects 16 oscillation peaks (vs 16 required)
   - Skips first 4 peaks for settling
   - Validates results before applying
   - Timeout protection

4. **Persistent Storage**
   - EEPROM storage of all calibration data
   - Automatic loading on startup
   - Clear command to reset

5. **Safety Features**
   - Limit switch monitoring
   - Position bounds checking
   - Voltage limiting
   - Timeout protection
   - User confirmation before applying gains

6. **Diagnostic Tools**
   - Real-time position test monitoring
   - Detailed result analysis
   - Step response data collection
   - System status display

### Key Algorithm: Ziegler-Nichols Relay Method

**Process:**
1. System oscillates with relay control (±5V)
2. Hysteresis band = range / 6
3. Detects peaks at center crossing
4. Calculates amplitude and period
5. Computes ultimate gain: `Ku = 4V / (π × amplitude)`
6. Applies ZN formulas with safety factor

**ZN Formulas (Classic):**
- Kp = 0.6 × Ku
- Ki = 1.2 × Ku / Tu
- Kd = 0.075 × Ku × Tu

**Conservative Modification:**
- Multiply all gains by 0.3 (30%)
- Provides very stable operation
- Minimal overshoot
- Excellent for initial testing

## Recommendations for Use

### Initial Setup Workflow

1. **Upload enhanced auto-tune code**
2. **Run: R** (Range Calibration)
   - Finds left/right limits
   - Measures total travel

3. **Run: F** (Friction Characterization)
   - Determines LEFT friction voltage
   - Determines RIGHT friction voltage

4. **Run: Z** (Auto-Tune)
   - Performs oscillation test
   - Select "Conservative" tuning (option 2)

5. **Run: T** (Manual Test)
   - Test gains with specific position
   - Verify performance

6. **Fine-tune if needed**
   - Adjust individual gains based on results
   - Re-run auto-tune with different preset

### Integration with Main Project

After tuning, copy these values to your main code:

```cpp
// From enhanced auto-tune results:
float KP = [tuned value];
float KI = [tuned value];
float KD = [tuned value];
float FRICTION_LEFT = [measured value];
float FRICTION_RIGHT = [measured value];

// Use same friction compensation logic:
float frictionComp = 0;
if (error < 0) {
  frictionComp = -FRICTION_LEFT;  // Moving RIGHT
} else {
  frictionComp = FRICTION_RIGHT;   // Moving LEFT
}
```

## Comparison with Documentation

### Alignment with ME 350 PID Theory Document

The enhanced code implements concepts from the reference document:

✓ **Feedback control** - Position error drives controller
✓ **Proportional term** - Immediate response to error
✓ **Integral term** - Eliminates steady-state error
✓ **Derivative term** - Dampens oscillations
✓ **Friction compensation** - Feedforward term
✓ **Anti-windup** - Integral limiting
✓ **Ziegler-Nichols method** - Classic auto-tune

### Extensions Beyond Documentation

✓ Directional friction compensation
✓ EEPROM persistent storage
✓ Multiple tuning presets
✓ Step response analysis
✓ Automatic range calibration

## Performance Expectations

### Conservative Tuning (Recommended)
- **Settling time:** 2-4 seconds
- **Overshoot:** <10%
- **Steady-state error:** <5 encoder counts
- **Stability margin:** Very high

### Classic Ziegler-Nichols
- **Settling time:** 1-2 seconds
- **Overshoot:** ~25%
- **Steady-state error:** <2 encoder counts
- **Stability margin:** Moderate

### Aggressive Tuning
- **Settling time:** <1 second
- **Overshoot:** ~40%
- **Steady-state error:** <2 encoder counts
- **Stability margin:** Lower (test carefully)

## Testing Checklist

Before using in main project:

- [ ] Range calibration completed (R command)
- [ ] Friction characterized (F command)
- [ ] Auto-tune successful (Z command, 16 peaks collected)
- [ ] Ku value reasonable (0.5 - 5.0)
- [ ] Period reasonable (0.5 - 3.0 seconds)
- [ ] Conservative gains tested (T command)
- [ ] No limit switch hits during normal operation
- [ ] Encoder counting correctly (no drift)
- [ ] Friction compensation effective
- [ ] Settings saved to EEPROM

## Common Problems and Solutions

### Problem: "Insufficient peaks" during auto-tune
**Causes:**
- Friction not characterized
- TEST_VOLTAGE too low
- Mechanical binding
- Hysteresis too large

**Solutions:**
- Run friction characterization first
- Increase TEST_VOLTAGE (in code, line 352)
- Check mechanical assembly
- Reduce hysteresis (in code, line 351)

### Problem: Large overshoot after tuning
**Causes:**
- Using aggressive or classic tuning
- System dynamics different from model
- Friction compensation too high

**Solutions:**
- Use conservative tuning
- Manually reduce Kp by 20%
- Increase Kd by 50%
- Recheck friction values

### Problem: Steady-state error
**Causes:**
- Ki too low
- Friction compensation incorrect
- Deadband too large

**Solutions:**
- Increase Ki (double current value)
- Rerun friction characterization
- Reduce DEADBAND (in code, line 46)

### Problem: Continuous oscillation
**Causes:**
- Gains too high
- Derivative term too low
- System delay

**Solutions:**
- Use conservative tuning
- Decrease Kp and Ki by 30%
- Increase Kd by 20%
- Add low-pass filter to derivative

## Future Enhancements

Potential additions to consider:

1. **Gain Scheduling**
   - Different gains at different positions
   - Adapts to varying system dynamics

2. **Adaptive Control**
   - Real-time gain adjustment
   - Learns optimal parameters

3. **Velocity Feedforward**
   - Pre-compensate for desired velocity
   - Faster tracking of moving targets

4. **Advanced Tuning Methods**
   - Cohen-Coon method
   - Lambda tuning
   - IMC-based tuning

5. **Data Logging**
   - SD card storage
   - Plot generation
   - Performance analysis

## Conclusion

The enhanced PID auto-tune code provides:

✓ Systematic, repeatable tuning process
✓ Multiple tuning options for different needs
✓ Comprehensive system characterization
✓ Persistent calibration storage
✓ Safety features and diagnostics
✓ Clear documentation and usage guide

It consolidates the best features from all existing code files while adding robust auto-tuning capabilities and following best practices from control theory literature.

**Recommendation:** Use this code for initial system characterization and PID tuning, then integrate the resulting parameters into your main project code.
