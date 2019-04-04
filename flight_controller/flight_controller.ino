/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) : 
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send min throttle
 *     5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
 *
 * @author lobodol <grobodol@gmail.com>
 */

 // ---------------- Motor Control ---------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
boolean ESC_READY = false;
char data;
// ---------------------------------------------------------------------------

// ---------------- Gyro and Accelerometer ---------------------------------------
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <SoftwareSerial.h>
Madgwick filter;
//angles
float roll, pitch, heading;
//time
unsigned long microsPerReading, microsPrevious, microsPreviousAngle, microsPreviousPID, microsNow;

unsigned long microsAddThrottle; //for testing take off with no transmitted instructions

// ---------------- flight instructions ---------------------------------------
#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

// ------------- Global variables used for PID controller --------------------
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float adjustmentPeriod;

// used for putting derivative through low pass filter
float previous_filtered_delta_err[3] = {0, 0, 0};          // Error deltas in that order   : Yaw, Pitch, Roll

unsigned int pulse_length_esc1 = 1000,
        pulse_length_esc2 = 1000,
        pulse_length_esc3 = 1000,
        pulse_length_esc4 = 1000;
 /*
 *  - Yaw      : degree/sec
 *  - Pitch    : degree
 *  - Roll     : degree
 *  - Throttle : µs
 *
 * @var float[]
 */
 
float instruction[4];

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  
  // Gryo and Accel setup -----------------------------------------------------
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPreviousAngle = micros();
  microsPreviousPID = micros();
  microsPrevious = micros();

  // motor setup --------------------------------------------------------------
  motA.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
 /*
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 */

  // initial instructions ------------------------------------------------------
  instruction[YAW] = 0;
  instruction[PITCH] = 0;
  instruction[ROLL] = 0;
  instruction[THROTTLE] = 1000;

  // period of flight adjustment in microseconds (recalculate fight every 'adjustmentPeriod' microseconds)
  adjustmentPeriod = 40000;
  
}

void loop() {
  // current time
  microsNow = micros();
  
  // set esc to programmable mode if not already
  if (ESC_READY == false && Serial.available()){
      setupESC();
  }
  
  // this is the flight control loop
  else if(ESC_READY == true){

      // update every adjustmentPeriod seconds (max pulse length for esc is 2 milliseconds or 2000 microseconds)
      if(microsNow - microsPreviousPID >= adjustmentPeriod) {
          // get angles
          getAngles();
          //printAngles();
          
          // 3. Translate received data into usable values
          getFlightInstruction();
          
          // 4. Calculate errors comparing received instruction with measures
          calculateErrors();
    
          // 5. Calculate motors speed with PID controller
          pidController();
    
          // 6. Apply motors speed
          applyMotorSpeed();

          //update previous time
          microsPreviousPID = microsNow;
      }// end update block
      
  } // end else if block
  
}//end loop function

//read current angle of drone
void getAngles(){
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;

  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  
}
//helper of getAngles()
float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}
//helper of getAngles()
float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void printAngles() {

  //print rate in micro seconds
  unsigned long printRate = 1000000;
  
  if (microsNow - microsPreviousAngle >= printRate) {
      Serial.print("Orientation: ");
      Serial.print(heading);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(roll);

      microsPreviousAngle = microsNow;
  }
}

//This must be done before motor outputs can be programmed
//read the directions at the top of this document
void setupESC() {

  data = Serial.read();
  switch (data) {
      // 0
      case 48 : Serial.println("Sending minimum throttle");
                motA.writeMicroseconds(MIN_PULSE_LENGTH);
                motB.writeMicroseconds(MIN_PULSE_LENGTH);
                motC.writeMicroseconds(MIN_PULSE_LENGTH);
                motD.writeMicroseconds(MIN_PULSE_LENGTH);
      break;

      // 1
      case 49 : Serial.println("Sending maximum throttle");
                motA.writeMicroseconds(MAX_PULSE_LENGTH);
                motB.writeMicroseconds(MAX_PULSE_LENGTH);
                motC.writeMicroseconds(MAX_PULSE_LENGTH);
                motD.writeMicroseconds(MAX_PULSE_LENGTH);
      break;

      // 2
      
      case 50 :
                ESC_READY = true;
                //get rid of the rest of this,including 
                //test function when ready for flight
                Serial.print("ESC's ready in 3");
                delay(1000);
                Serial.print(" 2");
                delay(1000);
                Serial.println(" 1...");
                delay(1000);
                Serial.println("\ntaking off!\n");
                delay(2000);
                //test();          
      break;
  }
  
}//end setupESC

/**
 * Test function: send min throttle to max throttle to each ESC.
 * NOT USED *****
 */
void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
        motC.writeMicroseconds(i);
        motD.writeMicroseconds(i);
        
        delay(2000);
    }

    Serial.println("STOP");
    motA.writeMicroseconds(MIN_PULSE_LENGTH);
    motB.writeMicroseconds(MIN_PULSE_LENGTH);
    motC.writeMicroseconds(MIN_PULSE_LENGTH);
    motD.writeMicroseconds(MIN_PULSE_LENGTH);
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 *
 **** actually A and D are counter-clockwise and B & C are clockwise
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 *
 * @return void
 */
void pidController() {
    float Kp[3]                 = {4.0, 1.3, 1.3};    // P coefficients in that order    : Yaw, Pitch, Roll
    float Ki[3]                 = {0.02, 0.04, 0.04}; // I coefficients in that order    : Yaw, Pitch, Roll
    float Kd[3]                 = {0, 18, 18};        // D coefficients in that order    : Yaw, Pitch, Roll
    float delta_err[3]          = {0, 0, 0};          // Error deltas in that order      : Yaw, Pitch, Roll
    float filtered_delta_err[3] = {0, 0, 0};          // filtered deltas in that order   : Yaw, Pitch, Roll
    float yaw_pid               = 0;
    float pitch_pid             = 0;
    float roll_pid              = 0;
    float r                     = .90; //smoothing factor, smaller r for greater smoothing *can be tuned

    // Initialize motor commands with throttle
    pulse_length_esc1 = instruction[THROTTLE];
    pulse_length_esc2 = instruction[THROTTLE];
    pulse_length_esc3 = instruction[THROTTLE];
    pulse_length_esc4 = instruction[THROTTLE];

    // Do not calculate anything if throttle is 0
    if (instruction[THROTTLE] >= 1040) {
        // Calculate sum of errors : Integral coefficients
        error_sum[YAW]   += errors[YAW];
        error_sum[PITCH] += errors[PITCH];
        error_sum[ROLL]  += errors[ROLL];

        // Calculate error delta : Derivative coefficients
        delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
        delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
        delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];
        
        // apply low pass filter on derivative of proportional error for pitch and roll (reduce noise)
        filtered_delta_err[PITCH] = (1-r)*previous_filtered_delta_err[PITCH] + r*delta_err[PITCH];
        filtered_delta_err[ROLL]  = (1-r)*previous_filtered_delta_err[ROLL]  + r*delta_err[ROLL];
        
        // Save current error as previous_error for next time
        previous_error[YAW]   = errors[YAW];
        previous_error[PITCH] = errors[PITCH];
        previous_error[ROLL]  = errors[ROLL];
        previous_filtered_delta_err[PITCH] = filtered_delta_err[PITCH];
        previous_filtered_delta_err[ROLL]  = filtered_delta_err[ROLL];
        
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (filtered_delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (filtered_delta_err[ROLL]  * Kd[ROLL]);

        // cancel out yaw
        yaw_pid = 0;
        
        // Calculate pulse duration for each ESC
        pulse_length_esc1 = instruction[THROTTLE] + roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc2 = instruction[THROTTLE] - roll_pid + pitch_pid + yaw_pid;
        pulse_length_esc3 = instruction[THROTTLE] + roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc4 = instruction[THROTTLE] - roll_pid - pitch_pid - yaw_pid;
    }

    //clamp integral component to defend against error buildup (1600 is arbitrary, should experiment if there are problems)
    //-------------------------------------------------------------------------------------
    //pitch
    if( (pulse_length_esc1 >= 1600 || pulse_length_esc2 >= 1600 || pulse_length_esc3 >= 1600 || pulse_length_esc4 >= 1600)
        && pitch_pid/errors[PITCH] >= 0) { //saturate and error and previous error have same sign
        error_sum[PITCH] = 0;
     }
     //roll
    if( (pulse_length_esc1 >= 1600 || pulse_length_esc2 >= 1600 || pulse_length_esc3 >= 1600 || pulse_length_esc4 >= 1600)
        && roll_pid/errors[ROLL] >= 0) { //saturate and error and previous error have same sign
        error_sum[ROLL] = 0;
     }
    
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

/**
 * Make sure that given value is not over min_value/max_value range.
 *
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 * @return float
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

//read the current flight instruction into the system
void getFlightInstruction() {

      //initialized for takeoff
      instruction[YAW] = 0;
      instruction[PITCH] = 0; //negative forward, positive backward
      instruction[ROLL] = 0; //negative left, positive right

      unsigned int increasePeriod = 400000; // every 400 milliseconds
      if ((microsNow - microsAddThrottle) >= increasePeriod && instruction[THROTTLE] < 1700) {
          instruction[THROTTLE] = instruction[THROTTLE] + 5;
          microsAddThrottle = microsNow;
      }
}

/**
 * Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
 *
 * @return void
 */
void calculateErrors() {
    //errors[YAW]   = instruction[YAW]   - measures[YAW];
    errors[YAW]   =   0;
    errors[PITCH] = instruction[PITCH] - pitch;
    errors[ROLL]  = instruction[ROLL]  - roll;
}

// write pulselength calculated by PID to ESC's
void applyMotorSpeed() {

    //print rate in micro seconds
    unsigned long printRate = 2000000;
    // print the esc outputs to monitor
    if (microsNow - microsPrevious >= printRate) {
        Serial.print("motA: ");
        Serial.print(pulse_length_esc1);
        Serial.print("  motB: ");
        Serial.print(pulse_length_esc2);
        Serial.print("  motC: ");
        Serial.print(pulse_length_esc3);
        Serial.print("  motD: ");
        Serial.print(pulse_length_esc4);
        Serial.print("  Throttle: ");
        Serial.println(instruction[THROTTLE]);
      
        /*  
        //error info for PID
        Serial.print("Pitch Error: ");
        Serial.print(errors[PITCH]);
        Serial.print(" Roll Error: ");
        Serial.println(errors[ROLL]);
  
        Serial.print("Pitch Error Sum: ");
        Serial.print(error_sum[PITCH]);
        Serial.print(" Roll Error Sum: ");
        Serial.println(error_sum[ROLL]);
        */
        
        microsPrevious = microsNow;
    }

     
    //write pulse length to each esc (apply speed)
    motA.writeMicroseconds(pulse_length_esc1);
    motB.writeMicroseconds(pulse_length_esc2);
    motC.writeMicroseconds(pulse_length_esc3);
    motD.writeMicroseconds(pulse_length_esc4);
    
}

