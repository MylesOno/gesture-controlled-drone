#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <SoftwareSerial.h>
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
boolean calibration_done = false;
float y_shake = 0, z_shake = 0;
float min_roll = 0, max_roll  = 0, min_pitch = 0, max_pitch = 0;
float regZeroMaxP = 10.0;
float reg1MaxP = 25.0;
float reg2MaxP = 40.0;
float regZeroMaxR = 10.0;
float reg1MaxR = 25.0;
float reg2MaxR = 40.0;
int outputStream[] = {0,0,0};
SoftwareSerial mySerial(0, 1); // RX, TX




void setup() {
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  pinMode(1, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  mySerial.begin(9600);
  mySerial.write("BYE");
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;
  
  

  if(!calibration_done){
    microsNow = micros();
    if((microsNow - microsPrevious) <= 5000000){
      //Serial.print("Calibration, please hold steady "); 
      calibrate();
    }
    else{
      calibration_done = true;
      //Serial.print("==================done with calibration. ");
      microsPrevious = micros();
    }
  }

  //post calibration activity.  This is the flight stage
  else{
  
    // check if it's time to read data and update the filter
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
  
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

      printRegion(roll,pitch);
//      mySerial.write("BYE");
//      Serial.write("mySerial available");
      
      
       
//      Serial.println("WRITING");
      
//      mySerial.write(outputStream[2]);
//      mySerial.write(outputStream[1]);

//============ convert int to binary array
const byte numPins = 3;

byte pins1[] = {7,8,9};
byte pins2[] = {10,11,12};
byte num1 = outputStream[1]+3;
byte num2 = outputStream[2]+3;
  for (byte i=0; i<numPins; i++) {
    byte state = bitRead(num1, i);
    digitalWrite(pins1[i], state);
    Serial.print(state);
    
  }
  Serial.println();
  for (byte i=0; i<numPins; i++) {
    byte state = bitRead(num2, i);
    digitalWrite(pins2[i], state);
    Serial.print(state);
    
  }
  Serial.println();
  Serial.println(num1);
  Serial.println(num2);

//================

      
    
      Serial.print("Orientation: ");
      Serial.print(heading);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(roll);

      // increment previous time, so we keep proper pace
      microsPrevious = microsPrevious + microsPerReading;
    }
  }
  
}

void printRegion(float roll, float pitch){
  
  
  
  //pitch region 0 
  if(pitch < regZeroMaxP && pitch > -1.0*regZeroMaxP){
    //Serial.println("Pitch 0");
    outputStream[2] = 0;
  }else if(pitch > regZeroMaxP && pitch < reg1MaxP){
    //Serial.println("Pitch 1");
    outputStream[2] = 1;
    regZeroMaxP = 7.0;
    reg1MaxP = 28;
    reg2MaxP = 40;
  }else if(pitch > reg1MaxP && pitch < reg2MaxP){
    //Serial.println("Pitch 2");
    outputStream[2] = 2;
    regZeroMaxP = 10;
    reg1MaxP = 22;
    reg2MaxP = 43;
  }else if(pitch > reg2MaxP){
//    Serial.println("Pitch 3");
    outputStream[2] = 3;
    regZeroMaxP = 10;
    reg1MaxP = 25;
    reg2MaxP = 37;
  }else if(pitch < -1.0*regZeroMaxP && pitch > -1.0*reg1MaxP){
//    Serial.println("Pitch -1");
    outputStream[2] = -1;
    regZeroMaxP = 7;
    reg1MaxP = 28;
    reg2MaxP = 40;
  }else if(pitch < -1.0*reg1MaxP && pitch > -1.0*reg2MaxP){
//    Serial.println("Pitch -2");
    outputStream[2] = -2;
    regZeroMaxP = 10;
    reg1MaxP = 22;
    reg2MaxP = 43;
  }else if(pitch < -1.0*reg2MaxP ){
//    Serial.println("Pitch -3");
    outputStream[2] = -3;
    regZeroMaxP = 10;
    reg1MaxP = 25;
    reg2MaxP = 37;
  }

  if(roll < regZeroMaxR && roll > -1.0*regZeroMaxR){
//    Serial.println("Roll 0");
    outputStream[1] = 0;
  }else if(roll > regZeroMaxR && roll < reg1MaxR){
//    Serial.println("Roll 1");
    outputStream[1] = 1;
    regZeroMaxR = 7;
    reg1MaxR = 28;
    reg2MaxR = 40;
  }else if(roll > reg1MaxR && roll < reg2MaxR){
//    Serial.println("Roll 2");
    outputStream[1] = 2;
    regZeroMaxR = 10;
    reg1MaxR = 22;
    reg2MaxR = 43;
  }else if(roll > reg2MaxR){
//    Serial.println("Roll 3");
    outputStream[1] = 3;
    regZeroMaxR = 10;
    reg1MaxR = 25;
    reg2MaxR = 37;
  }else if(roll < -1.0*regZeroMaxR && roll > -1.0*reg1MaxR){
//    Serial.println("Roll -1");
    outputStream[1] = -1;
    regZeroMaxR = 7;
    reg1MaxR = 28;
    reg2MaxR = 40;
  }else if(roll < -1.0*reg1MaxR && roll > -1.0*reg2MaxR){
//    Serial.println("Roll -2");
    outputStream[1] = -2;
    regZeroMaxP = 10;
    reg1MaxP = 22;
    reg2MaxP = 43;
  }else if(roll < -1.0*reg2MaxR){
//    Serial.println("Roll -3");
    outputStream[1] = -3;
    regZeroMaxR = 10;
    reg1MaxR = 25;
    reg2MaxR = 37;
  }
//  Serial.print(outputStream[0]);
//  Serial.print(" ");
//  Serial.print(outputStream[1]);
//  Serial.print(" ");
//  Serial.print(outputStream[2]);
//  Serial.println(" ");
}

void calibrate(){
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

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
    
    if(pitch > max_pitch){
      max_pitch = pitch;
    }
    if(roll > max_roll){
      max_roll = roll;
    }
    y_shake = max_roll;
    z_shake = max_pitch;
    if(y_shake > 5){
      y_shake = 5;
    }
    if(z_shake > 5){
      z_shake = 5;
    }
//    Serial.print(" ");
//    Serial.print(z_shake);
//    Serial.print(" ");
//    Serial.println(y_shake);
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
