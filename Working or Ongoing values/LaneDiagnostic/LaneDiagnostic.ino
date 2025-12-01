// ME 350 - Lane Range and Direction Diagnostic Tool
// This code helps troubleshoot targeting logic by monitoring:
// - Lane positions (encoder positions)
// - Proximity sensor ranges (min/max values)
// - Target distances (0.0 = start, 1.0 = impact)
// - Target directions (FORWARD/BACKWARD/STOPPED)
// - Movement tracking

#define PROX_SENSOR_1 A0
#define PROX_SENSOR_2 A1
#define PROX_SENSOR_3 A2
#define PROX_SENSOR_4 A3

// Direction constants
const int FORWARD = 1;   // Moving toward impact point (away from sensor = sensor value decreasing)
const int BACKWARD = -1; // Moving away from impact point (toward sensor = sensor value increasing)
const int STOPPED = 0;   // Not moving

// Lane positions (encoder positions where turret should aim)
long targetPositions[4] = {-109, -376, -629, -1257};

// Proximity sensor ranges for each lane [FAR, CLOSE]
// These will be auto-calibrated during runtime
float ProxRange[4][2] = {
  {615.0, 88.0},   // Lane 1: [far, close]
  {634.0, 124.0},  // Lane 2
  {622.0, 147.0},  // Lane 3
  {590.0, 80.0}    // Lane 4
};

// Sensor filtering
const float alpha = 0.75;
const int stopTimeout = 80;
const int lowerNoiseLimit = 5;
const int upperNoiseLimit = 8;
const int noiseThreshold = 225;

// Proximity sensor structure
struct ProxSensor {
  float currVal;
  float prevVal;
  float smoothVal;
  unsigned long prevChangeTime;
  int pin;
  int direction;
  int forwardCount;
  int backwardCount;
  float minObserved;  // Track minimum (closest) value observed
  float maxObserved;  // Track maximum (farthest) value observed
};

ProxSensor ProxSensors[4];

// Distance tracking (0.0 = start of lane, 1.0 = impact point)
float zombieDistances[4] = {0.0, 0.0, 0.0, 0.0};

// Calibration flags
bool rangeCalibrated[4] = {false, false, false, false};
unsigned long lastCalibrationTime = 0;
const unsigned long CALIBRATION_DURATION = 10000; // 10 seconds to find ranges

// Print timing
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200; // Print every 200ms

//============================================
// SETUP
//============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n\n=========================================="));
  Serial.println(F("ME 350 - Lane Range & Direction Diagnostic"));
  Serial.println(F("=========================================="));
  Serial.println(F("\nThis tool helps troubleshoot targeting logic:"));
  Serial.println(F("- Monitors proximity sensors for all 4 lanes"));
  Serial.println(F("- Auto-calibrates sensor ranges (min/max)"));
  Serial.println(F("- Calculates target distances (0.0 = start, 1.0 = impact)"));
  Serial.println(F("- Detects movement direction (FORWARD/BACKWARD/STOPPED)"));
  Serial.println(F("\nTARGETS START AT BEGINNING OF LANE (distance = 0.0)"));
  Serial.println(F("MOVING AWAY FROM PROXIMITY SENSOR = MOVING TOWARD IMPACT"));
  Serial.println(F("==========================================\n"));
  
  // Initialize sensors
  ProxSensors[0].pin = PROX_SENSOR_1;
  ProxSensors[1].pin = PROX_SENSOR_2;
  ProxSensors[2].pin = PROX_SENSOR_3;
  ProxSensors[3].pin = PROX_SENSOR_4;
  
  for (int i = 0; i < 4; i++) {
    float initialVal = analogRead(ProxSensors[i].pin);
    ProxSensors[i].currVal = initialVal;
    ProxSensors[i].prevVal = initialVal;
    ProxSensors[i].smoothVal = initialVal;
    ProxSensors[i].prevChangeTime = millis();
    ProxSensors[i].direction = STOPPED;
    ProxSensors[i].forwardCount = 0;
    ProxSensors[i].backwardCount = 0;
    ProxSensors[i].minObserved = initialVal;
    ProxSensors[i].maxObserved = initialVal;
  }
  
  lastCalibrationTime = millis();
  
  Serial.println(F("Starting calibration..."));
  Serial.println(F("Place targets at START of lanes, then move them through full range.\n"));
}

//============================================
// LOOP
//============================================
void loop() {
  unsigned long now = millis();
  
  // Update sensor readings
  updateSensors();
  
  // Auto-calibrate ranges during first 10 seconds
  if (now - lastCalibrationTime < CALIBRATION_DURATION) {
    calibrateRanges();
  }
  
  // Print diagnostic info periodically
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    printDiagnosticInfo();
    lastPrintTime = now;
  }
}

//============================================
// UPDATE SENSORS
// Reads proximity sensors and determines direction
//============================================
void updateSensors() {
  unsigned long now = millis();
  
  for (int i = 0; i < 4; i++) {
    // Read raw sensor value
    int rawVal = analogRead(ProxSensors[i].pin);
    
    // Apply exponential moving average filter
    ProxSensors[i].currVal = alpha * ProxSensors[i].currVal + (1.0 - alpha) * rawVal;
    ProxSensors[i].smoothVal = ProxSensors[i].smoothVal * 0.9 + rawVal * 0.1;
    
    // Calculate distance (0.0 = start, 1.0 = impact)
    // Distance increases as sensor value decreases (target moves away from sensor toward impact)
    const float range = ProxRange[i][0] - ProxRange[i][1];
    if (range > 0) {
      // Normalize: higher sensor value = farther from impact = lower distance
      float normalized = (ProxSensors[i].currVal - ProxRange[i][1]) / range;
      zombieDistances[i] = constrain(normalized, 0.0, 1.0);
    } else {
      zombieDistances[i] = 0.0;
    }
    
    // Determine noise threshold
    int sensorNoiseLimit = (ProxSensors[i].currVal >= noiseThreshold) ? upperNoiseLimit : lowerNoiseLimit;
    
    // Calculate change from previous reading
    float change = ProxSensors[i].currVal - ProxSensors[i].prevVal;
    float changeMagnitude = abs(change);
    
    // Direction detection
    if (changeMagnitude < sensorNoiseLimit) {
      // No significant change - target is STOPPED
      if (now - ProxSensors[i].prevChangeTime >= stopTimeout) {
        ProxSensors[i].direction = STOPPED;
        ProxSensors[i].forwardCount = 0;
        ProxSensors[i].backwardCount = 0;
      }
    } else if (change < 0) {
      // Sensor value DECREASING = target moving AWAY from sensor = moving TOWARD impact = FORWARD
      ProxSensors[i].forwardCount++;
      ProxSensors[i].backwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      
      if (ProxSensors[i].forwardCount >= 2) {
        ProxSensors[i].direction = FORWARD;
      }
    } else {
      // Sensor value INCREASING = target moving TOWARD sensor = moving AWAY from impact = BACKWARD
      ProxSensors[i].backwardCount++;
      ProxSensors[i].forwardCount = 0;
      ProxSensors[i].prevVal = ProxSensors[i].currVal;
      ProxSensors[i].prevChangeTime = now;
      
      if (ProxSensors[i].backwardCount >= 2) {
        ProxSensors[i].direction = BACKWARD;
      }
    }
  }
}

//============================================
// CALIBRATE RANGES
// Auto-find min/max sensor values for each lane
//============================================
void calibrateRanges() {
  for (int i = 0; i < 4; i++) {
    float val = ProxSensors[i].smoothVal;
    
    // Update min/max observed values
    if (val < ProxSensors[i].minObserved) {
      ProxSensors[i].minObserved = val;
    }
    if (val > ProxSensors[i].maxObserved) {
      ProxSensors[i].maxObserved = val;
    }
    
    // Use observed min/max as range if they're valid
    if (ProxSensors[i].maxObserved - ProxSensors[i].minObserved > 50) {
      ProxRange[i][0] = ProxSensors[i].maxObserved; // Far (start of lane)
      ProxRange[i][1] = ProxSensors[i].minObserved; // Close (impact point)
      rangeCalibrated[i] = true;
    }
  }
}

//============================================
// PRINT DIAGNOSTIC INFO
// Comprehensive diagnostic output
//============================================
void printDiagnosticInfo() {
  unsigned long now = millis();
  bool inCalibration = (now - lastCalibrationTime < CALIBRATION_DURATION);
  unsigned long calibrationRemaining = (inCalibration) ? (CALIBRATION_DURATION - (now - lastCalibrationTime)) / 1000 : 0;
  
  // Clear screen (send escape sequence)
  Serial.print(F("\033[2J\033[H")); // Clear screen and move cursor to top
  
  Serial.println(F("=========================================="));
  Serial.println(F("  LANE RANGE & DIRECTION DIAGNOSTIC"));
  Serial.println(F("=========================================="));
  
  if (inCalibration) {
    Serial.print(F("\n[ CALIBRATING ] "));
    Serial.print(calibrationRemaining);
    Serial.println(F(" seconds remaining..."));
    Serial.println(F("Move targets through full range for best calibration.\n"));
  } else {
    Serial.println(F("\n[ CALIBRATION COMPLETE ]\n"));
  }
  
  // Print lane information
  for (int i = 0; i < 4; i++) {
    Serial.print(F("------------------------------------------\n"));
    Serial.print(F("LANE "));
    Serial.print(i + 1);
    Serial.print(F(" | Target Position: "));
    Serial.print(targetPositions[i]);
    Serial.println();
    
    // Sensor values
    Serial.print(F("  Raw: "));
    Serial.print(analogRead(ProxSensors[i].pin));
    Serial.print(F(" | Filtered: "));
    Serial.print(ProxSensors[i].currVal, 1);
    Serial.print(F(" | Smooth: "));
    Serial.println(ProxSensors[i].smoothVal, 1);
    
    // Range information
    Serial.print(F("  Range: ["));
    Serial.print(ProxRange[i][0], 1);
    Serial.print(F(" (FAR/START) ←→ "));
    Serial.print(ProxRange[i][1], 1);
    Serial.print(F(" (CLOSE/IMPACT)]"));
    if (rangeCalibrated[i]) {
      Serial.print(F(" ✓"));
    } else {
      Serial.print(F(" [CALIBRATING]"));
    }
    Serial.println();
    
    // Observed range during calibration
    if (inCalibration) {
      Serial.print(F("  Observed Range: ["));
      Serial.print(ProxSensors[i].maxObserved, 1);
      Serial.print(F(" ←→ "));
      Serial.print(ProxSensors[i].minObserved, 1);
      Serial.print(F("] (Range: "));
      Serial.print(ProxSensors[i].maxObserved - ProxSensors[i].minObserved, 1);
      Serial.println(F(")"));
    }
    
    // Distance calculation
    Serial.print(F("  Distance: "));
    Serial.print(zombieDistances[i] * 100, 1);
    Serial.print(F("%"));
    if (zombieDistances[i] < 0.1) {
      Serial.print(F(" [AT START]"));
    } else if (zombieDistances[i] > 0.9) {
      Serial.print(F(" [NEAR IMPACT!]"));
    } else if (zombieDistances[i] > 0.7) {
      Serial.print(F(" [APPROACHING]"));
    }
    Serial.println();
    
    // Direction
    Serial.print(F("  Direction: "));
    if (ProxSensors[i].direction == FORWARD) {
      Serial.print(F("FORWARD →"));
      Serial.print(F(" (moving TOWARD impact point)"));
    } else if (ProxSensors[i].direction == BACKWARD) {
      Serial.print(F("BACKWARD ←"));
      Serial.print(F(" (moving AWAY from impact point)"));
    } else {
      Serial.print(F("STOPPED —"));
      Serial.print(F(" (not moving)"));
    }
    Serial.println();
    
    // Direction counts
    Serial.print(F("  Movement: Fwd="));
    Serial.print(ProxSensors[i].forwardCount);
    Serial.print(F(" | Bwd="));
    Serial.print(ProxSensors[i].backwardCount);
    Serial.println();
    
    // Visual distance bar
    Serial.print(F("  ["));
    int barLength = 40;
    int filled = (int)(zombieDistances[i] * barLength);
    for (int j = 0; j < barLength; j++) {
      if (j < filled) {
        Serial.print(F("#"));
      } else {
        Serial.print(F("-"));
      }
    }
    Serial.print(F("] "));
    Serial.print(F("START"));
    if (zombieDistances[i] > 0.1 && zombieDistances[i] < 0.9) {
      Serial.print(F(" ←→ IMPACT"));
    } else {
      Serial.print(F(" → IMPACT"));
    }
    Serial.println();
  }
  
  Serial.println(F("=========================================="));
  Serial.println(F("\nKEY:"));
  Serial.println(F("  Distance: 0.0 = Start of lane, 1.0 = Impact point"));
  Serial.println(F("  FORWARD = Moving toward impact (away from sensor)"));
  Serial.println(F("  BACKWARD = Moving away from impact (toward sensor)"));
  Serial.println(F("  STOPPED = Not moving"));
  Serial.println(F("\nPress Ctrl+C to stop, reset to restart calibration\n"));
}

