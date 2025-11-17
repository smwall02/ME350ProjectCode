# ME350 Project Code - Comprehensive Review and Bug Analysis

**Review Date:** November 17, 2024
**Branch:** claude/comprehensive-code-review-1763420670
**Reviewer:** Claude AI Code Analyzer

## Executive Summary

This comprehensive code review analyzed the ME350 "Plants vs Zombies" competition project, focusing on two main Arduino sketches:
1. `ME350_Competition_StateMachine.ino` - Main competition code
2. `PID_AutoTune_ME350.ino` - PID tuning utility

**Overall Assessment:** The code is well-structured and functional, but contains **several critical bugs** that will cause incorrect behavior, particularly in friction compensation and control loop timing.

---

## Critical Bugs Found

### 🔴 **BUG #1: Friction Characterization Swapped Variables**
**Location:** `PID_AutoTune_ME350.ino:524-582`
**Severity:** CRITICAL
**Impact:** Incorrect friction compensation values, causing poor motor control

**Problem:**
```cpp
// Line 506-532: Characterizing RIGHT friction
while (!motionDetected && testVoltage < 8.0) {
  setMotorVoltage(-testVoltage);  // Negative = RIGHT
  // ...
  if (movement > 10) {
    FRICTION_LEFT = testVoltage - 0.2;  // ❌ WRONG! Should be FRICTION_RIGHT
    Serial.print(F("RIGHT friction voltage: "));  // Says RIGHT
    Serial.print(FRICTION_LEFT, 2);  // But assigns to FRICTION_LEFT
```

**And:**
```cpp
// Line 562-588: Characterizing LEFT friction
while (!motionDetected && testVoltage < 8.0) {
  setMotorVoltage(testVoltage);  // Positive = LEFT
  // ...
  if (movement > 10) {
    FRICTION_RIGHT = testVoltage - 0.2;  // ❌ WRONG! Should be FRICTION_LEFT
    Serial.print(F("LEFT friction voltage: "));  // Says LEFT
    Serial.print(FRICTION_RIGHT, 2);  // But assigns to FRICTION_RIGHT
```

**Impact:** The friction values are completely swapped, causing:
- Inadequate friction compensation when moving RIGHT
- Excessive friction compensation when moving LEFT
- Poor positioning accuracy
- Potential oscillation or stiction

**Fix Required:**
```cpp
// Line 524: Change to:
FRICTION_RIGHT = testVoltage - 0.2;  // Correct for RIGHT movement

// Line 580: Change to:
FRICTION_LEFT = testVoltage - 0.2;   // Correct for LEFT movement
```

---

### 🔴 **BUG #2: Control Loop Delta Time Calculation Error**
**Location:** `ME350_Competition_StateMachine.ino:313`
**Severity:** HIGH
**Impact:** Incorrect PID derivative calculation, affecting stability

**Problem:**
```cpp
controlDtSeconds = max((currentTime - lastControlUpdate) / 1000.0, CONTROL_PERIOD / 1000.0);
```

**Issue:** Using `max()` is incorrect. If the loop runs faster than expected (e.g., 5ms instead of 10ms), it will still use 10ms for dt calculation, making the derivative calculation incorrect.

**Example:**
- Actual loop time: 5ms
- `(currentTime - lastControlUpdate) / 1000.0 = 0.005`
- `CONTROL_PERIOD / 1000.0 = 0.01`
- `max(0.005, 0.01) = 0.01` ❌ Wrong! Should use 0.005

**Fix Required:**
```cpp
// Remove this line entirely and just use:
controlDtSeconds = (currentTime - lastControlUpdate) / 1000.0;
```

Or if you want a minimum bound:
```cpp
controlDtSeconds = min((currentTime - lastControlUpdate) / 1000.0, CONTROL_PERIOD / 1000.0);
```

---

### 🟡 **BUG #3: Friction Compensation Variable Naming Confusion**
**Location:** `ME350_Competition_StateMachine.ino:617-623` and `PID_AutoTune_ME350.ino:355-362`
**Severity:** MEDIUM (assuming Bug #1 is fixed)
**Impact:** Confusing variable names that are backwards from documentation

**Problem:** The variable names suggest:
- `FRICTION_LEFT` = friction when moving LEFT
- `FRICTION_RIGHT` = friction when moving RIGHT

But the characterization code (even after fixing Bug #1) measures:
- Moving RIGHT → store in `FRICTION_LEFT`
- Moving LEFT → store in `FRICTION_RIGHT`

This is confusing and error-prone.

**Current Implementation:**
```cpp
// Competition StateMachine line 617-623
if (error < -DEADBAND) {
  // Need to move RIGHT (negative direction)
  frictionComp = -adaptiveFrictionLeft;  // ⚠️ Confusing naming
}
else if (error > DEADBAND) {
  // Need to move LEFT (positive direction)
  frictionComp = adaptiveFrictionRight;  // ⚠️ Confusing naming
}
```

**Recommendation:** Rename variables to match their actual meaning:
- `FRICTION_LEFT` → `FRICTION_FOR_RIGHT_MOTION` (or `FRICTION_NEGATIVE`)
- `FRICTION_RIGHT` → `FRICTION_FOR_LEFT_MOTION` (or `FRICTION_POSITIVE`)

Or better: fix the characterization so the names make sense.

---

### 🟡 **BUG #4: Redundant Direction Check**
**Location:** `ME350_Competition_StateMachine.ino:533`
**Severity:** LOW
**Impact:** Redundant code, minor performance impact

**Problem:**
```cpp
bool directionFlip = (sensors[activeTarget].direction == BACKWARD &&
                      sensors[activeTarget].velocity < -2.0 &&
                      sensors[activeTarget].rawValue > ACTIVATION_THRESHOLD_LOW);
```

The `direction` is already calculated from `velocity` in `updateSensors()` (lines 657-665):
```cpp
if (derivative > 2.0) {
  sensors[i].direction = FORWARD;
}
else if (derivative < -2.0) {
  sensors[i].direction = BACKWARD;
}
```

So checking both `direction == BACKWARD` AND `velocity < -2.0` is redundant - they represent the same condition.

**Fix:** Remove the redundant check:
```cpp
bool directionFlip = (sensors[activeTarget].direction == BACKWARD &&
                      sensors[activeTarget].rawValue > ACTIVATION_THRESHOLD_LOW);
```

---

### 🟡 **BUG #5: Serial Output Formatting Error**
**Location:** `PID_AutoTune_ME350.ino:847-851`
**Severity:** LOW
**Impact:** Incorrect console output formatting

**Problem:**
```cpp
Serial.println(F("1. Conservative (30% ZN)     -> Kp="));
Serial.print(kp1, 4);
Serial.print(F(" Ki="));
Serial.print(ki1, 4);
Serial.print(F(" Kd="));
Serial.println(kd1, 4);
```

Using `println` on the first line prints a newline immediately, causing the values to appear on a separate line.

**Current Output:**
```
1. Conservative (30% ZN)     -> Kp=
0.0036 Ki=0.0018 Kd=0.0004
```

**Expected Output:**
```
1. Conservative (30% ZN)     -> Kp=0.0036 Ki=0.0018 Kd=0.0004
```

**Fix:**
```cpp
Serial.print(F("1. Conservative (30% ZN)     -> Kp="));  // Use print, not println
```

---

## Potential Issues and Warnings

### ⚠️ **ISSUE #1: Limit Switch Wiring Inconsistency**
**Location:** Throughout both files
**Severity:** MEDIUM (depends on hardware)

**Problem:** The code uses `INPUT_PULLUP` for limit switches but checks for `HIGH` when pressed:
```cpp
pinMode(LIMIT_LEFT, INPUT_PULLUP);  // Line 253
// Later...
if (digitalRead(LIMIT_LEFT) == HIGH) {  // Line 368 - pressed
```

With `INPUT_PULLUP`, the standard behavior is:
- Switch open: `HIGH` (pulled up to 5V)
- Switch pressed: `LOW` (connected to GND)

However, the documentation (PinAssignments.md line 16-18) explicitly states switches are HIGH when pressed, which means the hardware is wired to pull HIGH when pressed (connected to VCC when closed), not to GND. This is non-standard.

**Recommendation:**
1. Verify the actual hardware wiring
2. If switches pull to GND when pressed, fix the code to check for `LOW`
3. If switches pull to VCC when pressed, document this clearly and remove `INPUT_PULLUP` (use `INPUT` instead)

---

### ⚠️ **ISSUE #2: Round Transition Requires 5 Second Sensor Quiet Time**
**Location:** `ME350_Competition_StateMachine.ino:742-755`

**Problem:**
```cpp
bool allSensorsStopped() {
  for (int i = 0; i < 4; i++) {
    if (fabs(sensors[i].velocity) > VEL_STOP_THRESH) return false;
  }
  return true;
}
// Round transition requires both time AND 5s quiet:
if (elapsed >= ROUND_1_DURATION && stoppedFor5s) {
```

**Impact:** If any sensor has noise or environmental changes, the round will not transition even after 40 seconds. This could cause the competition to get stuck in Round 1 or Round 2.

**Recommendation:** Consider using a timeout override:
```cpp
if (elapsed >= ROUND_1_DURATION + 10000) {
  // Force transition after 10s grace period
  currentRound = ROUND_2;
}
```

---

### ⚠️ **ISSUE #3: Integral Windup Reduction Overlap**
**Location:** `PID_AutoTune_ME350.ino:1049-1057`

**Problem:**
```cpp
if ((error != 0) && (error * lastError < 0)) {
  integral *= 0.5;  // Reduce on sign flip
}
if (abs(error) > 400) {
  integral *= 0.5;  // Reduce when far
}
if (abs(error) > 600) {
  integral *= 0.95;  // Further reduce when very far
}
```

When `error > 600`, both the 400 and 600 conditions trigger, reducing integral by `0.5 * 0.95 = 0.475` (more than half). This may be intentional, but seems like it could be unintended compounding.

**Recommendation:** Use `else if` or document that this is intentional:
```cpp
if ((error != 0) && (error * lastError < 0)) {
  integral *= 0.5;
}
else if (abs(error) > 600) {
  integral *= 0.95;  // Very far
}
else if (abs(error) > 400) {
  integral *= 0.5;   // Far
}
```

---

### ⚠️ **ISSUE #4: Ziegler-Nichols Ku Calculation Dimensional Error**
**Location:** `PID_AutoTune_ME350.ino:802`

**Problem:**
```cpp
float Ku = (4.0 * TEST_VOLTAGE) / (PI * avgAmplitude);
```

This formula is correct for relay feedback, but:
- `TEST_VOLTAGE` is in volts (5.0V)
- `avgAmplitude` is in encoder counts

The resulting `Ku` has units of `V/counts`, but the PID controller uses dimensionless gains (the error is in counts and output is in volts, so this works out). However, this should be documented.

**Recommendation:** Add comment explaining the dimensional analysis:
```cpp
// Ku has units of V/count, which is correct for our PID implementation
// where error is in encoder counts and output is in volts
float Ku = (4.0 * TEST_VOLTAGE) / (PI * avgAmplitude);
```

---

## Code Quality Observations

### ✅ **Positive Aspects:**

1. **Well-structured state machine** - Clear separation of states with good comments
2. **Comprehensive documentation** - Excellent README and configuration files
3. **EEPROM persistence** - Good use of non-volatile storage for calibration
4. **Multiple tuning methods** - PID autotune offers several tuning strategies
5. **Safety features** - Limit switch monitoring and motor kill switch
6. **Adaptive features** - Friction boost and dynamic target switching
7. **Serial debugging** - Good status output for debugging

### 🔧 **Areas for Improvement:**

1. **Variable naming** - Friction compensation variable names are confusing
2. **Code duplication** - Some calibration logic duplicated between files
3. **Magic numbers** - Some constants could be better documented
4. **Error handling** - Limited error recovery for failed calibration
5. **Lambda functions** - `allSensorsStopped()` redefined every call (line 742)

---

## Testing Recommendations

### Before Competition:

1. **Test friction characterization** after fixing Bug #1
   - Verify `FRICTION_LEFT` and `FRICTION_RIGHT` values make sense
   - Should typically be 1-4V range
   - LEFT and RIGHT should be similar (within 2x of each other)

2. **Verify PID stability** after fixing Bug #2
   - Run multiple position tests
   - Check for oscillation or overshoot
   - Verify settling time <2 seconds

3. **Test round transitions**
   - Ensure rounds transition at exactly 40 seconds
   - Test with noisy sensor environment
   - Verify Round 3 ends correctly on limit switch

4. **Validate limit switch logic**
   - Manually test each limit switch
   - Verify pressed state matches code expectations
   - Test calibration sequence

5. **Sensor testing**
   - Test all 4 proximity sensors
   - Verify direction detection (FORWARD/BACKWARD)
   - Confirm activation thresholds

---

## Recommended Fixes (Priority Order)

### Priority 1: Critical Bugs (Must Fix)
1. ✅ Fix friction characterization variable swap (Bug #1)
2. ✅ Fix control loop dt calculation (Bug #2)

### Priority 2: Important Issues (Should Fix)
3. ✅ Clarify limit switch wiring and fix code if needed (Issue #1)
4. ✅ Add round transition timeout override (Issue #2)
5. ✅ Fix friction variable naming confusion (Bug #3)

### Priority 3: Nice to Have (Optional)
6. ✅ Remove redundant direction check (Bug #4)
7. ✅ Fix serial output formatting (Bug #5)
8. ✅ Fix integral reduction overlap (Issue #3)
9. ✅ Add dimensional analysis comment (Issue #4)

---

## File-Specific Notes

### ME350_Competition_StateMachine.ino

**Purpose:** Main competition code with 3-round state machine

**Strengths:**
- Comprehensive round management
- Good sensor direction detection
- Adaptive friction boost
- Dynamic target switching

**Issues:**
- Control loop dt calculation (Bug #2)
- Friction variable naming (Bug #3)
- Round transition quiet time requirement (Issue #2)

**Lines of Interest:**
- 313: Control dt calculation ❌
- 368: Limit switch check ⚠️
- 617-623: Friction compensation ⚠️
- 742-755: Round transition logic ⚠️

---

### PID_AutoTune_ME350.ino

**Purpose:** PID tuning utility with auto-calibration

**Strengths:**
- Multiple tuning methods (ZN, Tyreus-Luyben, etc.)
- Comprehensive friction characterization
- EEPROM storage
- Step response analysis

**Issues:**
- Friction characterization variable swap (Bug #1) ❌
- Serial formatting (Bug #5)
- Integral windup overlap (Issue #3)
- Ku dimensional units (Issue #4)

**Lines of Interest:**
- 524-526: Friction RIGHT characterization ❌
- 580-582: Friction LEFT characterization ❌
- 802: Ku calculation ⚠️
- 847-851: Serial output formatting ❌
- 1049-1057: Integral reduction ⚠️

---

## Conclusion

The ME350 project code is well-architected and feature-rich, but contains critical bugs that must be addressed before competition use:

1. **Critical:** Friction characterization stores values in the wrong variables
2. **Critical:** Control loop timing calculation is incorrect

These bugs will significantly impact control performance and must be fixed immediately.

The code also has several medium-priority issues around limit switch handling and round transitions that should be addressed through testing and validation.

Overall, with the critical bugs fixed, this code should perform well in the competition environment.

---

## Next Steps

1. Create a new branch for bug fixes
2. Fix Bug #1 (friction characterization)
3. Fix Bug #2 (control dt calculation)
4. Test thoroughly with hardware
5. Address medium-priority issues based on testing results
6. Run full competition simulation
7. Document any configuration changes needed

---

**End of Analysis Report**
