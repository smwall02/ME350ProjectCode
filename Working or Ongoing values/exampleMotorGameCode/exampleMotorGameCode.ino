// ME350 Zombie Defense Sketch - Version 05.1 by Spencer Dickhudt
// updated 12-5-2024
//
//

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES: //
//////////////////////////////////////////////

//** State Machine: **//
// CONSTANTS:
// Definition of states in the state machine
const int CALIBRATE = 1;
const int CHOOSE_ACTIVE_TARGET = 2;
const int MOVE_TO_TARGET = 3;

// VARIABLES:
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int state = CALIBRATE;

//** Proximity Sensors or Potentiometer: **//
// CONSTANTS:
// Definition of proximity sensor Limits for each target:
const int PROXIMITYSENSE1_MAX = 0; // [proximity sensor counts] Value of prox sensor 1 when zombie is closest to photosensor
const int PROXIMITYSENSE1_MIN = 0; // [proximity sensor counts] Value of prox sensor 1 when zombie is closest to prox sensor
const int PROXIMITYSENSE2_MAX = 0; // [proximity sensor counts] Value of prox sensor 2 when zombie is closest to photosensor
const int PROXIMITYSENSE2_MIN = 0; // [proximity sensor counts] Value of prox sensor 2 when zombie is closest to prox sensor
const int PROXIMITYSENSE3_MAX = 0; // [proximity sensor counts] Value of prox sensor 3 when zombie is closest to photosensor
const int PROXIMITYSENSE3_MIN = 0; // [proximity sensor counts] Value of prox sensor 3 when zombie is closest to prox sensor
const int PROXIMITYSENSE4_MAX = 0; // [proximity sensor counts] Value of prox sensor 4 when zombie is closest to photosensor
const int PROXIMITYSENSE4_MIN = 0; // [proximity sensor counts] Value of prox sensor 4 when zombie is closest to prox sensor

const int ProxRange[4][2] = {{PROXIMITYSENSE1_MAX, PROXIMITYSENSE1_MIN},
                              {PROXIMITYSENSE2_MAX, PROXIMITYSENSE2_MIN},
                              {PROXIMITYSENSE3_MAX, PROXIMITYSENSE3_MIN},
                              {PROXIMITYSENSE4_MAX, PROXIMITYSENSE4_MIN}}; // Array holding prox sensor bounds to improve code readability

// Definition of target variables for array indexing
const int TARGET1 = 0; // For indexing into activeTargets array, activeTargets[0] corresponds to Target 1
const int TARGET2 = 1; // For indexing into activeTargets array, activeTargets[1] corresponds to Target 2
const int TARGET3 = 2; // For indexing into activeTargets array, activeTargets[2] corresponds to Target 3
const int TARGET4 = 3; // For indexing into activeTargets array, activeTargets[3] corresponds to Target 4

const int targetArr[4] = { TARGET1, TARGET2, TARGET3, TARGET4 }; // List of target indices to reduce redundant code

// Direction variables to represent the movement of each zombie
const int FORWARD = 1; // representing the zombie moving towards the photosensor
const int BACKWARD = -1; // representing the zombie moving away from the photosensor
const int STOPPED = 0; // representing the zombie stalled

// VARIABLES:
// Variables for updating sensor readings
const float alpha = 0.925; // filter parameter used to weight current and past readings
int stopTimeout = 250; // [millis] time needed to declare a zombie stopped
int noiseLimit = 8; // average spread of proximity readings for a stopped zombie
const int lowerNoiseLimit = 5; // noise limit for near the photosensors
const int upperNoiseLimit = 8; // noise limit for near the prox sensors
const int noiseThreshold = 225; // prox sensor reading at which noise limit switches from upper to lower

// A data structure to hold proximity sensor information:
struct Sensor{
  float currVal; // the most recent proximity sensor reading
  float prevVal; // the last update of proximity sensor reading
  unsigned long prevChangeTime; // [millis] the time at which prevVal last changed
  int pin; // the input pin of the desired prox sensor
  int direction; // the direction at which the zombie on the rail is traveling
  int forwardCount; // number of readings which indicate the zombie is moving forward
  int backwardCount; // number of readings which indicate the zombie is moving backward
};

Sensor ProxSensors[4]; // An array which holds Sensor objects, representing each of the prox sensors
int potValue = 0; // Used to hold the reading of the potentiometer

int activeTargetIndex = -1; // index (in the activeTargets array) of the closest target to the plants
int activeTargetPosition = -1; // [encoder counts] Encoder count of the active target the linkage is moving toward

int minIndex = -1; // tracking index of the zombie who is closest to the plants
float closestZombieDist = 2; // [percentage] records the lowest percentage of rail remaining for any of the zombies
int idx = -1; // index of target, based on targetArr

bool WAIT_POS = true; // tracking if the linkage is moving to the wait position

unsigned long arrivalTime; // timer for tracking a wait period upon reaching a desired position
const int targetActivateTime = 350; // time to activate a target
float Zombies[4]; // An array to hold information on locations of all zombies in play

// float sampleTime = 1.5 // sample time in ms

//** Computation of position and velocity: **//
// CONSTANTS:
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2; // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile int motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0; // [binary] Past and Current A&B values of the encoder (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity = 0; // [encoder counts / seconds] Current motor velocity
int previousMotorPosition = 0; // [encoder counts] Motor position the last time a velocity was computed
long previousVelCompTime = 0; // [microseconds] System clock value the last time a velocity was computed

//** High-level behavior of the controller: **//
// CONSTANTS:
// Target positions:
const int CALIBRATION_VOLTAGE = 0; // [Volt] Motor voltage used during the calibration process
const int TARGET_1_POSITION = 0; // [encoder counts] Motor position corresponding to first target
const int TARGET_2_POSITION = 0; // [encoder counts] Motor position corresponding to second target
const int TARGET_3_POSITION = 0; // [encoder counts] Motor position corresponding to third target
const int TARGET_4_POSITION = 0; // [encoder counts] Motor position corresponding to fourth target
const int WAIT_POSITION = TARGET_3_POSITION; // [encoder counts] Motor position corresponding to a wait position (when no targets are active)
const int LOWER_BOUND = TARGET_1_POSITION; // [encoder counts] Position of the left end stop
const int UPPER_BOUND = TARGET_4_POSITION; // [encoder counts] Position of the right end stop
const int TARGET_BAND = 5; // [encoder counts] "Close enough" range when moving towards a target.

// List of target positions to reduce redundant code
const int targetPos[4] = {TARGET_1_POSITION, TARGET_2_POSITION, TARGET_3_POSITION, TARGET_4_POSITION};

// Timing:
//const long WAIT_TIME = 0; // [microseconds] Time waiting for the target to drop.
// TBD - implement a timer so if the target doesn't drop, the linkage moves to a different target
// VARIABLES:
//unsigned long startWaitTime; // [microseconds] System clock value

//** PID Controller **//
// CONSTANTS:
const float KP = 0; // [Volt / encoder counts] P-Gain
const float KI = 0; // [Volt / (encoder counts * seconds)] I-Gain
const float KD = 0; // [Volt * seconds / encoder counts] D-Gain
const float SUPPLY_VOLTAGE = 0; // [Volt] Supply voltage at the HBridge
const float FRICTION_COMP_VOLTAGE = 0; // [Volt] Voltage needed to overcome friction
// VARIABLES:
int desiredPosition = 0; // [encoder counts] desired motor position
float positionError = 0; // [encoder counts] Position error
float integralError = 0; // [encoder counts * seconds] Integrated position error
float velocityError = 0; // [encoder counts / seconds] Velocity error
float desiredVoltage = 0; // [Volt] Desired motor voltage
int motorCommand = 0; // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0; // [microseconds] Time between this and the previous loop execution. Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0; // [microseconds] System clock value at the moment the loop was started the last time

//** Pin assignment: **//
// CONSTANTS:
const int PIN_NR_ENCODER_A = 2; // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ENCODER_B = 3; // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ON_OFF_SWITCH = 5; // Connected to toggle switch (turns mechanism on and off)
const int PIN_NRL_LIMIT_SWITCH = 8; // Connected to limit switch (mechanism calibration)
const int PIN_NR_PWM_OUTPUT = 11; // Connected to H Bridge (controls motor speed)
const int PIN_NR_PWM_DIRECTION_1 = 12; // Connected to H Bridge (controls motor direction)
const int PIN_NR_PWM_DIRECTION_2 = 13; // Connected to H Bridge (controls motor direction)
const int PIN_PROXSENSE1 = A0; // Connected to proximity sensor 1
const int PIN_PROXSENSE2 = A1; // Connected to proximity sensor 2
const int PIN_PROXSENSE3 = A2; // Connected to proximity sensor 3
const int PIN_PROXSENSE4 = A3; // Connected to proximity sensor 4
const int PIN_POTENTIOMETER = A4; // Connected to potentiometer used to test target positions

// Add this line of code if you want to use two limit switches; const int PIN_NRL_LIMIT_SWITCH_2 = 11
// ^KEEP IN MIND THAT YOU HAVE TO ADD CODE DOWNSTREAM (FOR EXAMPLE YOU NEED TO ADD THIS VARIABLE IN THE DECLARATION SECTION

// End of CONSTANTS AND GLOBAL VARIABLES


/////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables,//
// pin modes, start using libraries, etc. The setup function will only run once, after //
// each powerup or reset of the Arduino board:                                        //
/////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  pinMode(PIN_NR_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_NR_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_NR_ON_OFF_SWITCH, INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH, INPUT);
  pinMode(PIN_PROXSENSE1, INPUT);
  pinMode(PIN_PROXSENSE2, INPUT);
  pinMode(PIN_PROXSENSE3, INPUT);
  pinMode(PIN_PROXSENSE4, INPUT);
  pinMode(PIN_NR_PWM_OUTPUT, OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1, OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2, OUTPUT);
  pinMode(PIN_POTENTIOMETER, INPUT);

  // Turn on the pullup resistors on the encoder channels
  digitalWrite(PIN_NR_ENCODER_A, HIGH);
  digitalWrite(PIN_NR_ENCODER_B, HIGH);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(0, updateMotorPosition, CHANGE); // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPosition, CHANGE); // Interrupt 1 is always attached to digital pin 3

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");

  // Initialize sensor values
  ProxSensors[TARGET1].prevVal = analogRead(PIN_PROXSENSE1);
  ProxSensors[TARGET2].prevVal = analogRead(PIN_PROXSENSE2);
  ProxSensors[TARGET3].prevVal = analogRead(PIN_PROXSENSE3);
  ProxSensors[TARGET4].prevVal = analogRead(PIN_PROXSENSE4);

  // Initialize prox sensor pins
  ProxSensors[TARGET1].pin = PIN_PROXSENSE1;
  ProxSensors[TARGET2].pin = PIN_PROXSENSE2;
  ProxSensors[TARGET3].pin = PIN_PROXSENSE3;
  ProxSensors[TARGET4].pin = PIN_PROXSENSE4;

  // Set initial output to the motor to 0
  analogWrite(PIN_NR_PWM_OUTPUT, 0);
}
// End of function setup()


///////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values, //
// the loop() function does precisely what its name suggests, and loops consecutively,      //
// allowing your program to sense and respond. Use it to actively control the Arduino board. //
///////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Determine the duration it took to execute the last loop. This time is used
  // for integration and for monitoring the loop time via the serial monitor.
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // Speed Computation:
  if ((abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
    // If at least a minimum time interval has elapsed or
    // the motor has travelled through at least a minimum angle ...
    // .. compute a new value for speed:
    // (speed = delta angle [encoder counts] divided by delta time [seconds])
    motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = motorPosition;
    previousVelCompTime = micros();
  }
  // *****************************************************************************//
  // We first update the sensors so our direction information stays accurate

  // Loop through each of the sensors
  for (int i = 0; i < sizeof(targetArr)/sizeof(int); i++) {
    idx = targetArr[i];
    // Updates the current sensor reading using a low pass filter
    ProxSensors[idx].currVal = alpha * ProxSensors[idx].currVal + (1 - alpha) * analogRead(ProxSensors[idx].pin);

    // Adjusts the noise Limit based on distance to sensor (since values are not linear)
    if (ProxSensors[idx].currVal >= noiseThreshold) {
      noiseLimit = upperNoiseLimit;
    } else {
      noiseLimit = lowerNoiseLimit;
    }

    // Check if sensor is stopped
    if (abs(ProxSensors[idx].currVal - ProxSensors[idx].prevVal) < noiseLimit) {
      // if not enough time has passed to declare stopped or already stopped continue loop
      if (millis() - ProxSensors[idx].prevChangeTime < stopTimeout || ProxSensors[idx].direction == STOPPED) {
        ProxSensors[idx].forwardCount = 0;
        ProxSensors[idx].backwardCount = 0;
        continue;
      } else {
        // the prox sensor has been stopped for long enough that we know it is stopped
        ProxSensors[idx].direction = STOPPED;
      }
    } else if (ProxSensors[idx].currVal - ProxSensors[idx].prevVal < 0){
      // Target is moving forward, as readings decrease moving further away from the sensor
      ProxSensors[idx].forwardCount += 1;
      // Target is moving forward
      if (ProxSensors[idx].forwardCount > 3){
        ProxSensors[idx].direction = FORWARD;
        ProxSensors[idx].prevVal = ProxSensors[idx].currVal;
        ProxSensors[idx].prevChangeTime = millis();
      }
    } else {
      // Target must be moving backward
      ProxSensors[idx].backwardCount += 1;
      // Target is moving backward
      if (ProxSensors[idx].backwardCount > 3){
        ProxSensors[idx].direction = BACKWARD;
        ProxSensors[idx].prevVal = ProxSensors[idx].currVal;
        ProxSensors[idx].prevChangeTime = millis();
      }
    }
  }
  // *****************************************************************************//

  //******************************************************************************//
  // The state machine:
  switch (state) {
    //****************************************************************************//
    // In the CALIBRATE state, we move the mechanism to a position outside of the
    // work space (towards the limit switch). Once the limit switch is on and
    // the motor has stopped turning, we know that we are against the end stop
    case CALIBRATE:
      // We don't have to do anything here since this state is only used to set
      // a fixed output voltage. This happens further below.

      // Decide what to do next:
      if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) {
        // We reached the endstop. Update the motor position to the limit:
        // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
        motorPosition = LOWER_BOUND;
        // Reset the error integrator:
        integralError = 0;
        // Calibration is finalized. Transition into DETERMINE_ACTIVE_TARGETS state
        Serial.println("State transition from CALIBRATE to CHOOSE_ACTIVE_TARGET");
        state = CHOOSE_ACTIVE_TARGET;
      }

      // Otherwise we continue calibrating
      break;

    //****************************************************************************//
    // In the DETERMINE_ACTIVE_TARGETS state, we use the most recent reading of the photosensors to
    // calculate the location of each zombie.
    // We then select an active target to move toward based on which zombie is closest to the plants.
    // We default to WAIT_POSITION if no targets are active.
    case CHOOSE_ACTIVE_TARGET:

      // Decide what zombie is closest to the photosensor
      // Current closest zombie is out of bounds
      minIndex = -1;
      // Set distance to 2 to be above any possible percentage values + noise
      closestZombieDist = 2;

      for (int i = 0; i < sizeof(targetArr)/sizeof(int); i++) {
        // Use the target array for the proper indexes into other arrays
        idx = targetArr[i];

        // Calculate the distance to the front of the rail by taking a percentage of how much rail
        // the zombie has left to travel.
        Zombies[idx] = (ProxSensors[idx].currVal - ProxRange[idx][1]) / (ProxRange[idx][0] - ProxRange[idx][1]);

        // Check to see if the zombie is traveling forward and it is closer than the previous zombie
        if (ProxSensors[idx].direction == FORWARD && Zombies[idx] < closestZombieDist) {
          // update the newest minimum value
          closestZombieDist = Zombies[idx];
          // update the index of the active target (in the targetArr array)
          minIndex = i;
        }
      }

      // if the min index points to a valid target, move to that target
      if (minIndex >= 0) {
        activeTargetIndex = targetArr[minIndex];
        activeTargetPosition = targetPos[activeTargetIndex];
        WAIT_POS = false;
        // Serial.println("Setting position to something other than wait");
      } else {
        activeTargetPosition = WAIT_POSITION;
        WAIT_POS = true;
        // Serial.println("Setting position to wait position");
      }

      state = MOVE_TO_TARGET;
      // Serial.println("Switching state to MOVE_TO_TARGET");

      // Otherwise, we stay in DETERMINE_ACTIVE_TARGETS
      break;

    //****************************************************************************//
    // In the MOVE_TO_TARGET state, we select an active target and move toward it, or
    // move toward Target 3 (a default position) if there is no active target
    case MOVE_TO_TARGET:
      // Serial.println("Inside MOVE_TO_TARGET");
      desiredPosition = activeTargetPosition;

      if (motorPosition <= activeTargetPosition + TARGET_BAND && motorPosition >= activeTargetPosition - TARGET_BAND) {
        if (millis() - arrivalTime > targetActivateTime || WAIT_POS){
          state = CHOOSE_ACTIVE_TARGET;
        }
      } else {
        arrivalTime = millis();
      }

      break;

    //****************************************************************************//
    //****************************************************************************//
    // We should never reach the next bit of code, which would mean that the state
    // we are currently in doesn't exist. So if it happens, throw an error and
    // stop the program:
    default:
      Serial.println("Statemachine reached at state that it cannot handle. ABORT!!!!");
      Serial.print("Found the following unknown state: ");
      Serial.println(state);
      while (1); // infinite loop to halt the program
      break;
  }
  // End of the state machine.
  //******************************************************************************//

  //******************************************************************************//
  // Recalibrate if we are in the leftmost position
  if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) {
    // We reached the endstop. Update the motor position to the limit:
    // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
    motorPosition = LOWER_BOUND;
    // Reset the error integrator:
    integralError = 0;
    Serial.println("Limit Switch hit");
  }

  //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on, run the controller:

    //** PID control: **//
    // Compute the position error [encoder counts]
    positionError = desiredPosition - motorPosition;
    // Compute the integral of the position error [encoder counts * seconds]
    integralError = integralError + positionError * (float)(executionDuration) / 1000000;
    // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
    velocityError = 0 - motorVelocity;
    // This is the actual controller function that uses the error in
    // position and velocity and the integrated error and computes a
    // desired voltage that should be sent to the motor:
    desiredVoltage = KP * positionError + KI * integralError + KD * velocityError;

    //** Feedforward terms: **//
    // Compensate for friction. That is, if we know the direction of
    // desired motion, add a base command that helps with moving in this
    // direction:
    if (positionError < -5) {
      desiredVoltage = desiredVoltage - FRICTION_COMP_VOLTAGE;
    }
    if (positionError > +5) {
      desiredVoltage = desiredVoltage + FRICTION_COMP_VOLTAGE;
    }

    // Anti-Wind-Up
    if (abs(desiredVoltage)>SUPPLY_VOLTAGE) {
      // If we are already saturating our output voltage, it does not make
      // sense to keep integrating the error (and thus ask for even higher
      // and higher output voltages). Instead, stop the integrator if the
      // output saturates. We do this by reversing the summation at the
      // beginning of this function block:
      integralError = integralError - positionError * (float)(executionDuration) / 1000000;
    }
    // End of 'if(onOffSwitch==HIGH)'

    // Override the computed voltage during calibration. In this state, we simply apply a
    // fixed voltage to move against one of the end-stops.
    if (state==CALIBRATE) {
      // add calibration code here
    }
  } else {
    // Otherwise, the toggle switch is off, so do not run the controller,
    // stop the motor...
    desiredVoltage = 0;
    // .. and reset the integrator of the error:
    integralError = 0;
    // Produce some debugging output:
    Serial.println("The toggle switch is off. Motor Stopped.");
  }
  // End of else onOffSwitch==HIGH

  //** Send signal to motor **//
  // Convert from voltage to PWM cycle:
  motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
  // Clip values larger than 255
  if (motorCommand > 255) {
    motorCommand = 255;
  }
  // Send motor signals out
  analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
  // Determine rotation direction
  if (desiredVoltage >= 0) {
    // If voltage is positive ...
    // ... turn forward
    digitalWrite(PIN_NR_PWM_DIRECTION_1, LOW); // rotate forward
    digitalWrite(PIN_NR_PWM_DIRECTION_2, HIGH); // rotate forward
  } else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_1, HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_2, LOW); // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//

  // Print out current controller state to Serial Monitor.
  printStateToSerial();
}
// End of main loop
//***********************************************************************//


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino. //
// It is called via an interrupt whenever the value on encoder    //
// channel A or B changes.                                        //
//////////////////////////////////////////////////////////////////////
void updateMotorPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by using a
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++; // increase the encoder count by one
  }
  else if (encoderStatus == 1 || encoderStatus == 7 || encoderStatus == 8 || encoderStatus == 14) {
    // the encoder status does not match a bit pattern that requires counting up by one.
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--; // decrease the encoder count by one
  }
}
// End of function updateMotorPosition()


//////////////////////////////////////////////////////////////////////
// This function sends a status of the controller to the serial   //
// monitor. Each character will take 85 microseconds to send, so //
// be selective in what you write out:                             //
//////////////////////////////////////////////////////////////////////
void printStateToSerial() {
  //*********************************************************************//
  // Send a status of the controller to the serial monitor.
  // Each character will take 85 microseconds to send, so be selective
  // in what you write out:

  //Serial.print("State Number: [CALIBRATE = 1; DETERMINE_ACTIVE_TARGETS = 2; MOVE_TO_TARGET = 3]: ");
  Serial.print("State#: ");
  Serial.print(state);

  //Serial.print("Power switch [on/off]: ");
  //Serial.print(" PWR: ");
  //Serial.print(digitalRead(PIN_NR_ON_OFF_SWITCH));

  //Serial.print(" Motor Position [encoder counts]: ");
  Serial.print(" MP: ");
  Serial.print(motorPosition);

  //Serial.print(" Motor Velocity [encoder counts / seconds]: ");
  Serial.print(" MV: ");
  Serial.print(motorVelocity);

  //Serial.print(" Encoder Status [4 bit value]: ");
  //Serial.print(" ES: ");
  //Serial.print(encoderStatus);

  //Serial.print(" Target Position [encoder counts]: ");
  Serial.print(" DP: ");
  Serial.print(desiredPosition);

  // //Serial.print(" Position Error [encoder counts]: ");
  // Serial.print(" PE: ");
  // Serial.print(positionError);

  // //Serial.print(" Integrated Error [encoder counts * seconds]: ");
  // Serial.print(" IE: ");
  // Serial.print(integralError);

  // //Serial.print(" Velocity Error [encoder counts / seconds]: ");
  // Serial.print(" VE: ");
  // Serial.print(velocityError);

  //Serial.print(" Desired Output Voltage [Volt]: ");
  Serial.print(" DV: ");
  Serial.print(desiredVoltage);

  //Serial.print(" Motor Command [0-255]: ");
  //Serial.print(" MC: ");
  //Serial.print(motorCommand);

  //Serial.print(" Execution Duration [microseconds]: ");
  //Serial.print(" ED: ");
  //Serial.print(executionDuration);

  //Serial.print(" Zombie Location [% of rail left to travel]: ");
  Serial.print(" ZL: ");
  for (int i = 0; i < sizeof(Zombies)/sizeof(float); i++) {
    Serial.print(Zombies[i]);
    Serial.print(" ");
  }

  //Serial.print(" Proxsensor Direction[STOPPED or FORWARD or BACKWARD]: ");
  Serial.print(" DR: ");
  for (int i = 0; i < sizeof(Zombies)/sizeof(float); i++) {
    Serial.print(ProxSensors[i].direction);
    Serial.print(" ");
  }

  // ALWAYS END WITH A NEWLINE. SERIAL MONITOR WILL CRASH IF NOT
  Serial.println(); // new line
}
// End of Serial Out

//////////////////////////////////////////////////////////////////////
// This function returns the average of the integer array          //
// pointed to by array_ptr and of length len.                      //
//////////////////////////////////////////////////////////////////////
float average (int * array_ptr, int len) {
  long sum = 0L;
  for (int i = 0; i < len; i++) {
    sum += array_ptr[i];
  }
  return ((float) sum)/len;
}
// End of average
