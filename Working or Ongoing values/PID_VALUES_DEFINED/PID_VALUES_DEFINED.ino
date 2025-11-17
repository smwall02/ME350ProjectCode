// ME350 Target Practice Sketch

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////


// VARIABLES:

//** Computation of position and velocity: **//
// CONSTANTS: 
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2;                  // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000;              // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile int motorPosition = 0;                    // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0;                    // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity        = 0;                    // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0;                    // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0;                    // [microseconds] System clock value the last time a velocity was computed 

//** High-level behavior of the controller:  **//
// CONSTANTS:
// Target positions:
const int TARGET_1_POSITION    = 100;                    // [encoder counts] Motor position corresponding to first target
const int TARGET_2_POSITION    = 348;                  // [encoder counts] Motor position corresponding to second target
const int TARGET_3_POSITION    = 582;                  // [encoder counts] Motor position corresponding to third target
const int TARGET_4_POSITION    = 1223;                 // [encoder counts] Motor position corresponding to fourth target
const int WAIT_POSITION        = 0;                  // [encoder counts] Motor position corresponding to a wait position
const int LOWER_BOUND          = TARGET_1_POSITION;    // [encoder counts] Position of the left end stop
const int UPPER_BOUND          = TARGET_4_POSITION;    // [encoder counts] Position of the right end stop
const int TARGET_BAND          = 5;                   // [encoder counts] "Close enough" range when moving towards a target.
// VARIABLES:
int activeTargetPosition = TARGET_3_POSITION;             // [encoder counts] position of the currently active target

//** PID Controller  **//
// CONSTANTS:
const float KP             = 0.4145775255/2;                      // [Volt / encoder counts] P-Gain
const float KI             =  0.1;                     // [Volt / (encoder counts * seconds)] I-Gain
const float KD             = 0;                     // [Volt * seconds / encoder counts] D-Gain
const float SUPPLY_VOLTAGE = 5.0;                  // [Volt] Supply voltage at the HBridge
const float FRICTION_COMP_VOLTAGE = 1.7;                     // [Volt] Voltage needed to overcome friction
// VARIABLES:
int  targetPosition  = 0;                           // [encoder counts] desired motor position
float positionError  = 0;                           // [encoder counts] Position error
float integralError  = 0;                           // [encoder counts * seconds] Integrated position error
float velocityError  = 0;                           // [encoder counts / seconds] Velocity error
float desiredVoltage = 0;                           // [Volt] Desired motor voltage
int   motorCommand   = 0;                           // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0;                // [microseconds] Time between this and the previous loop execution.  Used for integrals and derivatives
unsigned long lastExecutionTime = 0;                // [microseconds] System clock value at the moment the loop was started the last time

//** Pin assignment: **//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ON_OFF_SWITCH    = 5;
const int PIN_NRL_LIMIT_SWITCH_1  = 8;
const int PIN_NRL_LIMIT_SWITCH_2  = 9;
const int PIN_NR_PWM_OUTPUT       = 11;
const int PIN_NR_PWM_DIRECTION_1  = 12;
const int PIN_NR_PWM_DIRECTION_2  = 13;
// End of CONSTANTS AND GLOBAL VARIABLES


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  pinMode(PIN_NR_ENCODER_A,        INPUT_PULLUP);
  pinMode(PIN_NR_ENCODER_B,        INPUT_PULLUP);
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH_1,  INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH_2,  INPUT);
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);

  // Turn on the pullup resistors on the encoder channels
  // (the other sensors already have physical resistors on the breadboard) 
  digitalWrite(PIN_NR_ENCODER_A, HIGH);  
  digitalWrite(PIN_NR_ENCODER_B, HIGH);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(0, updateMotorPositionAndVelocity, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPositionAndVelocity, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");

  // Initialize outputs:
  // Set initial output to the motor to 0
  analogWrite(PIN_NR_PWM_OUTPUT, 0);
}
// End of function setup()


////////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values,   //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to sense and respond. Use it to actively control the Arduino board.  //
//////////////////////////////////////////////////////////////////////////////////////////////// 
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
    motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                            (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = motorPosition;
    previousVelCompTime   = micros();
  }
  
  //******************************************************************************//

      // Set the target position to the desired encoder count:
      targetPosition = activeTargetPosition;

      // Check if you are near target position
      if (motorPosition>=targetPosition-TARGET_BAND && motorPosition<=targetPosition+TARGET_BAND && motorVelocity==0) {
        Serial.println("You are in the band of targetPosition");
      } 
      // Otherwise we continue moving 

  //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on run the controller:

    //** PID control: **//  
    // Compute the position error [encoder counts]
    positionError = targetPosition - motorPosition;
    // Compute the integral of the position error  [encoder counts * seconds]
    integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
    // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
    velocityError = 0 - motorVelocity;
    // This is the actual controller function that uses the error in 
    // position and velocity and the integrated error and computes a
    // desired voltage that should be sent to the motor:
    desiredVoltage = KP * positionError +  
                     KI * integralError +
                     KD * velocityError;
 
    //** Feedforward terms: **//
    // Compensate for friction.  That is, if we now the direction of 
    // desired motion, add a compensation voltage that helps with moving in this
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
      // and higher output voltages).  Instead, stop the inegrator if the 
      // output saturates. We do this by reversing the summation at the 
      // beginning of this function block:
      integralError = integralError - positionError * (float)(executionDuration) / 1000000; 
    }
    // End of 'if(onOffSwitch==HIGH)'
    
  } else { 
    // Otherwise, the toggle switch is off, so do not run the controller, 
    // stop the motor...
    desiredVoltage = 0; 
    // .. and reset the integrator of the error:
    integralError = 0;
    // Produce some debugging output:
    Serial.println("The toggle switch is off.  Motor Stopped.");
  } 
  // End of  else onOffSwitch==HIGH
  
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
    digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
  } else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//


  //*********************************************************************//
  // Send a status of the controller to the serial monitor.  
  // Each character will take 85 microseconds to send, so be
  // selective in what you write out:
  Serial.print("      Position [encoder counts]: ");
  Serial.print("  P: "); 
  Serial.println(motorPosition);
//  // End of Serial Out
  //*********************************************************************//
}
// End of main loop


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPositionAndVelocity() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++;         // increase the encoder count by one
  } 
  else if (encoderStatus == 1 || encoderStatus == 7 || encoderStatus == 8 || encoderStatus == 14) {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--;         // decrease the encoder count by one
  }
}
// End of function updateMotorPosition()
