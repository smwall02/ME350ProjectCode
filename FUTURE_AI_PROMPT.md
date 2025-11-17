# Prompt for Future AI Assistants

When working on this ME350 mechatronics project, start by reading this:

---

## Quick Start

```
1. READ FIRST: AI_ASSISTANT_REFERENCE.md (this directory)
2. THEN READ: System Documentation/ME350_System_Configuration.md
3. USE LATEST CODE: In Progress/ME350_Competition_StateMachine_v2.ino
```

---

## Critical Information

### Sensor Logic (MOST COMMON MISTAKE!)

**Zombies move:** BACK → FRONT (toward right limit/goal)

**Sensor behavior as zombie passes:**
- Far away → LOW value (~50)
- **Approaching** → INCREASING value (100, 200, 400, 600) ← **TARGET THESE!**
- At sensor → MAXIMUM value (~800)
- **Leaving** → DECREASING value (600, 400, 200, 100) ← **DON'T TARGET!**
- Passed → LOW value (~50)

```cpp
// ✅ CORRECT
if (sensors[i].rawValue > THRESHOLD && sensors[i].direction == APPROACHING) {
  // Value INCREASING = zombie coming toward us = TARGET THIS
}

// ❌ WRONG - This targets zombies that already passed!
if (sensors[i].direction == LEAVING) {
  // Value DECREASING = zombie moving away = TOO LATE
}
```

### Pin Configuration (STANDARD - DO NOT CHANGE)

```cpp
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_ENA 11    // PWM
#define MOTOR_IN2 12    // Direction 1
#define MOTOR_IN3 13    // Direction 2
#define LIMIT_LEFT 8    // Zero position
#define LIMIT_RIGHT 9   // Goal/front
#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3
```

### Motor Direction

- **Positive voltage** → Move LEFT → Toward encoder position 0
- **Negative voltage** → Move RIGHT → Toward negative encoder positions

### Key Files

**Competition Code (use this):**
- `In Progress/ME350_Competition_StateMachine_v2.ino`

**PID Auto-Tune:**
- `In Progress/PID_Tuning_Improved/PID_AutoTune_ME350.ino`

**Documentation:**
- `AI_ASSISTANT_REFERENCE.md` ← Read this first!
- `System Documentation/ME350_System_Configuration.md`
- `In Progress/ME350_Competition_README.md`

**Example Code (reference):**
- `Project Information/Example Code/ME350_LinkageCode_V5.1 (1).pdf`

**Existing Implementations (reference):**
- `Working or Ongoing values/UpdatedGameCodeNov17/` ← Most sophisticated
- `Working or Ongoing values/PID_AutoTune_Enhanced/`

---

## Before Making Changes

1. ✅ Read `AI_ASSISTANT_REFERENCE.md`
2. ✅ Verify you understand sensor direction logic
3. ✅ Check which version of code to modify (use v2)
4. ✅ Review existing implementations for similar features
5. ✅ Preserve standard pin configuration
6. ✅ Add clear comments explaining your changes

---

## Common User Requests

### "Tune the PID gains"
→ Upload `PID_AutoTune_ME350.ino`, run: R → F → Z → T

### "Set up the competition code"
→ Upload `ME350_Competition_StateMachine_v2.ino`
→ Use setup mode: P (positions), G (gains), F (friction), V (view), S (start)

### "The sensor logic is backwards"
→ Check if targeting APPROACHING (value increasing) vs LEAVING (value decreasing)
→ See "Sensor Logic" section above

### "Add a new feature"
→ Read relevant section in `System Documentation/ME350_System_Configuration.md` first
→ Check `UpdatedGameCodeNov17/` for similar implementations
→ Modify `ME350_Competition_StateMachine_v2.ino`

### "Why does it target the wrong zombies?"
→ Almost always sensor direction logic issue
→ Should target: `sensors[i].direction == APPROACHING`
→ Should NOT target: `sensors[i].direction == LEAVING`

---

## Competition Structure

**Round 1:** 40s, normal speed, LED or limit = point
**Round 2:** 40s, faster, LED or limit = point
**Round 3:** Survival, **LED ONLY** = point, limit = GAME OVER

---

## State Machine

```
SETUP → CALIBRATE → CHOOSE_ACTIVE_TARGET ⇄ MOVE_TO_TARGET
```

**SETUP:** Configure positions/gains (new in v2)
**CALIBRATE:** Zero encoder at left limit
**CHOOSE_ACTIVE_TARGET:** Select closest approaching zombie
**MOVE_TO_TARGET:** PID control, check for LED activation

---

## Default Values (Must Be Tuned!)

```cpp
// Target positions (measure your actual positions!)
long TARGET_1_POSITION = -74;
long TARGET_2_POSITION = -307;
long TARGET_3_POSITION = -547;
long TARGET_4_POSITION = -1080;

// PID gains (run auto-tune!)
float KP = 0.020;
float KI = 0.005;
float KD = 0.004;

// Friction (run characterization!)
float FRICTION_LEFT = 2.2;   // Moving RIGHT
float FRICTION_RIGHT = 0.25; // Moving LEFT
```

---

## Quick Checklist

When user asks you to work on this project:

- [ ] Read `AI_ASSISTANT_REFERENCE.md`
- [ ] Understand sensor direction logic (APPROACHING vs LEAVING)
- [ ] Identify which code file to modify (probably v2)
- [ ] Check existing implementations for reference
- [ ] Preserve standard pin configuration
- [ ] Test logic carefully before committing
- [ ] Add comments explaining changes

---

**Remember:** The #1 mistake is getting sensor direction backwards. Value INCREASING = approaching = target this!
