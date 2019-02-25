#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
boolean calibration_done = true;
float y_shake = 0, z_shake = 0;
boolean current_pitch[3], current_roll[3];
float min_roll = 0, max_roll  = 0, min_pitch = 0, max_pitch = 0;

void setup() {
  Serial.begin(9600);

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
      Serial.print("Calibration, please hold steady "); 
      calibrate();
    }
    else{
      calibration_done = true;
      Serial.print("==================done with calibration. ");
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
    Serial.print(" ");
    Serial.print(z_shake);
    Serial.print(" ");
    Serial.println(y_shake);
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
