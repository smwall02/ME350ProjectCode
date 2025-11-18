# UpdatedGameCodeNov17 - Improvements Summary

## Overview
This document summarizes the improvements made to the Plants vs Zombies competition code based on the reference implementation. The improvements focus on better limit switch handling, enhanced PID control, and persistent calibration storage.

## Key Improvements

### 1. **Integrated PID Auto-Tune Mode** 🆕
- **Press 'T' at any time** to enter PID Tuning Mode without switching programs
- **Ziegler-Nichols Auto-Tune**: Industry-standard relay oscillation method
  - Automatically finds ultimate gain (Ku) and period (Tu)
  - Offers 3 tuning presets: Conservative (30%), Classic (100%), Aggressive (80%)
  - All results saved to EEPROM automatically
- **Test Mode**: Test current PID gains with data logging
  - CSV output: Time, Position, Error, Voltage
  - 5-second test move to Lane 3
- **Manual Update**: Manually enter Kp, Ki, Kd values
- **View Settings**: Display current PID values and last auto-tune results
- **Seamless Integration**: Pauses competition, then returns when done

#### Tuning Mode Commands
```
T - Enter PID Tuning Mode (from main menu)
  Z - Ziegler-Nichols Auto-Tune
  T - Test current PID gains
  U - Update PID gains manually
  V - View current settings
  Q - Quit tuning mode
```

#### Auto-Tune Process
1. System moves to Lane 3 (center position)
2. Applies relay oscillation with 5V amplitude
3. Collects 20+ peak crossings
4. Calculates Ku and Tu
5. Offers 3 tuning presets
6. Saves selected gains to EEPROM

#### Benefits
- **No program switching**: Tune PID without uploading different code
- **Scientific tuning**: Industry-standard Ziegler-Nichols method
- **Multiple presets**: Choose aggressiveness level
- **Instant testing**: Test new gains immediately
- **Persistent storage**: All changes saved automatically

### 2. **Improved Limit Switch Handling**
- **Soft Homing**: Implemented gentle approach to limit switches with stable position detection
- **Hold and Stabilize**: System now holds position for 300ms with stable tick detection before zeroing encoder
- **Multiple Zeroing**: Encoder is zeroed multiple times (3x) to ensure value sticks
- **Active HIGH Logic**: Properly configured limit switches with INPUT_PULLUP
- **Voltage Parameters**:
  - `CALIBRATE_EXTRA_VOLTAGE = 0.6V`: Extra voltage to overcome friction during homing
  - `CALIBRATE_MIN_VOLTAGE = 2.5V`: Minimum safe homing voltage
  - `CALIBRATE_HOLD_TIME = 300ms`: Time to hold at limit before zeroing
  - `CALIBRATE_STABLE_TICKS = 3`: Required stable readings

### 3. **Enhanced PID Control**

#### Better Friction Compensation
- **Directional Friction**: Separate friction values for each direction
  - `FRICTION_LEFT = 2.2V`: For moving toward more negative positions (away from home)
  - `FRICTION_RIGHT = 0.25V`: For moving toward less negative positions (toward home)
- **Adaptive Friction Boost**: Incremental boost if target not reached
  - `FRICTION_BOOST_AMOUNT = 0.2V`: Added when position not reached in time
  - Automatically resets when motion detected
- **Smart Scaling**: Friction compensation scales with error magnitude and velocity

#### Voltage Capping
Progressive voltage limits based on error magnitude prevent overshoot:
- `error > 800`: 4.0V limit
- `error > 500`: 3.5V limit
- `error > 300`: 3.2V limit
- `error <= 300`: 3.0V limit

#### Anti-Windup
- Zero-crossing detection resets integral term by 50%
- Prevents integral windup during direction changes
- Improves settling time and reduces overshoot

### 4. **EEPROM Calibration Storage**

#### Persistent Storage
All critical calibration values now persist across power cycles:
- PID gains (Kp, Ki, Kd)
- Friction compensation values (left and right)
- Lane positions (all 4 lanes)
- Left/right limit positions

#### EEPROM Layout (Compatible with PID Auto-tune Sketch)
```
Address 0:  Flag (0xAA when valid)
Address 1:  Kp (4 bytes float)
Address 5:  Ki (4 bytes float)
Address 9:  Kd (4 bytes float)
Address 13: Friction Left (4 bytes float)
Address 17: Friction Right (4 bytes float)
Address 21: Left Limit (4 bytes long)
Address 25: Right Limit (4 bytes long)
Address 29: Lane positions (4 * 4 bytes long)
```

#### New Commands
- `L`: Load calibration from EEPROM
- `W`: Save all calibration to EEPROM
- `C`: Calibrate lanes (now auto-saves to EEPROM)

### 5. **Flip Switch Motor Override**
- **Pin 5 Override**: Physical switch for motor enable/disable
- **Active HIGH Logic**: HIGH = enabled, LOW = disabled
- **Safety Feature**: Provides immediate motor shutdown capability
- **Status Messages**: Console feedback when switch state changes

### 6. **Code Structure Improvements**
- Added comprehensive comments explaining friction direction logic
- Separated friction compensation into clear directional components
- Improved variable naming for clarity
- Better organization of calibration constants

## Preserved Features
The following state machine logic was **NOT** modified to preserve existing functionality:
- ✅ State machine structure (CALIBRATE, FIND_RANGE, CHOOSE_ACTIVE_TARGET, MOVE_TO_TARGET)
- ✅ Target selection algorithm (prioritizes closest forward-moving zombie)
- ✅ Sensor filtering and direction detection
- ✅ Dynamic sensor calibration
- ✅ Zombie hit detection logic
- ✅ All competition timing and scoring logic

## Testing Recommendations

### Before Competition
1. **Test Homing**: Verify soft homing works smoothly
   - Use `Z` command to home
   - Check encoder zeroes correctly
   - Verify no hard impacts on limit switch

2. **Calibrate Lanes**: Set up lane positions
   - Use `C` command for manual calibration
   - Verify positions saved with `W` command
   - Test reload with `L` command

3. **PID Tuning**: If needed, adjust PID gains
   - Current values: Kp=0.020, Ki=0.005, Kd=0.004
   - Update in code or via EEPROM
   - Save with `W` command

4. **Friction Check**: Verify friction values
   - Test moves in both directions
   - Adjust `FRICTION_LEFT` and `FRICTION_RIGHT` if needed
   - Save with `W` command

5. **Flip Switch**: Test motor override
   - Connect switch to pin 5
   - Verify motor stops when switch LOW
   - Verify normal operation when switch HIGH

### During Competition
- Flip switch provides emergency stop
- EEPROM values persist across power cycles
- No need to recalibrate between rounds

## Troubleshooting

### Encoder Doesn't Zero Properly
- Increase `CALIBRATE_HOLD_TIME` (currently 300ms)
- Increase `CALIBRATE_STABLE_TICKS` (currently 3)
- Check limit switch wiring (should be active HIGH)

### Poor Position Control
- Check friction values match your hardware
- Verify PID gains loaded from EEPROM
- Try manual PID tuning
- Check voltage limits aren't too restrictive

### EEPROM Not Loading
- Verify flag byte at address 0 is 0xAA
- Use `L` command to manually load
- Check Serial output for load confirmation

## Future Enhancements
Potential improvements for next iteration:
- [ ] Auto-tuning routine for friction compensation
- [ ] Velocity-based control mode
- [ ] Advanced trajectory planning
- [ ] Data logging to EEPROM
- [ ] Bluetooth configuration interface

## File Location
`/Working or Ongoing values/UpdatedGameCodeNov17/UpdatedGameCodeNov17.ino`

## Compatibility
- Hardware: Standard ME350 configuration
- Arduino: Uno/Nano/Mega compatible
- Libraries: Encoder.h, EEPROM.h

---
*Last Updated: 2025-11-18*
*Updated By: Claude (AI Assistant)*
