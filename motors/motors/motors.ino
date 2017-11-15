#include <Servo.h>
#include <PS2X_lib.h>
#include <TaskScheduler.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <pid_controller.h>


#define PS2_DAT        6  //yellowwhite    
#define PS2_CMD        2  //orange, blue ground, whiteblue power, 
#define PS2_SEL        3  //whitegreen
#define PS2_CLK        4  //green

#define pressures   false
#define rumble      false

#define forward_spin 95
#define backward_spin 80
#define SerialDebug true  // Set to true to get Serial output for debugging
#define AHRS true

volatile PS2X ps2x;
double kp=0.01, ki=0.005, kd=0.005;

PID mag_pid(kp, ki, kd, 5, -5);
int mag_diff_values = 0;

MS5803 sensor(ADDRESS_HIGH);

MPU9250 myIMU;
Madgwick filter;
volatile Servo motorR;
volatile Servo motorL;
volatile Servo motorB;
byte vibrate = 0;

//Create variables to store results
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
double base_altitude = 335.0;

volatile int initial_x = 0;
volatile int initial_y = 0;
volatile int initial_z = 0;
volatile int x_val_diff = 0;
volatile int y_val_diff = 0;
volatile int z_val_diff = 0;
volatile int x_current = 0;
volatile int y_current = 0;
volatile int z_current = 0;

// Callback methods prototypes
void readGamepadCallback();
void updateMotorBCallback();
void updateMotorRLCallback();
void updateImuCallback();
void updatePressureSensorCallback();

//Tasks
Task readGamepad(50, TASK_FOREVER, &readGamepadCallback);
Task updateMotorB(125, TASK_FOREVER, &updateMotorBCallback);
Task updateMotorLR(125, TASK_FOREVER, &updateMotorRLCallback);
Task updateImuTask(150, TASK_FOREVER, &updateImuCallback);
Task updatePressureSensor(1000, TASK_FOREVER, &updatePressureSensorCallback);

Scheduler runner;

void readGamepadCallback() {
  ps2x.read_gamepad(false, vibrate);
  
  x_current = map(ps2x.Analog(PSS_LX), 0, 255, 0, 180);
//  Serial.println("current x");
//  Serial.println(x_current);
  y_current = map(ps2x.Analog(PSS_LY), 0, 255, 0, 180);
//  Serial.println("current y");
//  Serial.println(y_current);
  z_current = map(ps2x.Analog(PSS_RY), 0, 255, 0, 180);
//  Serial.println("current z");
//  Serial.println(z_current);
  if(ps2x.ButtonPressed(PSB_START))
  {
    initial_x = map(ps2x.Analog(PSS_LX), 0, 255, 0, 180);
    Serial.println("initial x");
    Serial.println(initial_x);
    initial_y = map(ps2x.Analog(PSS_LY), 0, 255, 0, 180);
    Serial.println("initial y");
    Serial.println(initial_y);
    initial_z = map(ps2x.Analog(PSS_RY), 0, 255, 0, 180);
    Serial.println("initial z");
    Serial.println(initial_z);
    updateMotorB.enable();
    updateMotorLR.enable();
    updateImuTask.disable();
    updatePressureSensor.disable();
  }
  if(ps2x.ButtonPressed(PSB_SELECT))
  {
    if(updateMotorB.isEnabled())
    {
      updateMotorB.disable();
      updateMotorLR.disable();
      updateImuTask.enable();
      updatePressureSensor.enable();
    }
    else
    {
      updateMotorB.enable();
      updateMotorLR.enable();
      updateImuTask.disable();
      updatePressureSensor.disable();
    }
  }

}

void updateMotorBCallback() {
  if((z_current - initial_z) < 25 && (z_current - initial_z) >= 0)
  {
   motorB.write(90); 
  }
  else if((z_current - initial_z) > -25 && (z_current-initial_z) < 0)
  {
    motorB.write(90);
  }
  else
  {
    z_val_diff = (z_current - initial_z) / 5;
    motorB.write(90 + z_val_diff);
  }

//  Serial.println(90 + z_val_diff);
}

void updateMotorRLCallback() {
  
 if(ps2x.Button(PSB_R1)) 
 {
   Serial.println("R1 being pressed right now");
   if(x_current > initial_x)
   {
      x_val_diff = (x_current - initial_x) / 5;
      motorL.write(110);
      motorR.write(110 + x_val_diff);
    }
    else if(x_current < initial_x)
    {
      x_val_diff = (initial_x - x_current) / 10;
      motorL.write(110 + x_val_diff);
      motorR.write(110);
    }
    else
    {
      motorR.write(110);
      motorL.write(110);
    }
   }
   else if(ps2x.Button(PSB_L1))
   {
    Serial.println("L1 being pressed right now");
    if(x_current < initial_x)
    {
      x_val_diff = (initial_x - x_current) / 10;
      motorL.write(70);
      motorR.write(70 - x_val_diff);
    }
    else if(x_current > initial_x)
    {
      x_val_diff = (x_current - initial_x) / 10;
      motorL.write(70 - x_val_diff);
      motorR.write(70);
    }
    else
    {
      motorR.write(70);
      motorL.write(70);
    }
   }
   if(!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_R1))
   {
      motorR.write(90);
      motorL.write(90);
   }
}

void updateImuCallback() {
// If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  filter.updateIMU(myIMU.gx, myIMU.gy, myIMU.gz, myIMU.ax, myIMU.ay, myIMU.az);

    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 9.62;
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
//        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print("Yaw: ");
        Serial.println(myIMU.yaw, 2);
//        Serial.print(", ");
//        Serial.print(myIMU.pitch, 2);
//        Serial.print(", ");
//        Serial.println(myIMU.roll, 2);

//        Serial.print("rate = ");
//        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
//        Serial.println(" Hz");
      }


      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)


    mag_diff_values = mag_pid.calculateOutput(0, myIMU.yaw);
    Serial.println(mag_diff_values);
    
    if(mag_diff_values < 0)
    {
      motorL.write(90 - mag_diff_values);
      motorR.write(90);
    }
    else if (mag_diff_values > 0)
    {
      motorL.write(90);
      motorR.write(90 + mag_diff_values);
    }

    
//  if((myIMU.yaw) > 5)
//  {
//     motorL.write(100);
//     motorR.write(90);
//  }
//  else if((myIMU.yaw) < -5)
//  {
//     motorR.write(100);
//     motorL.write(90);
//  }
//  else
//  {
//    motorR.write(90);
//    motorL.write(90);
//  }
}

void updatePressureSensorCallback() {
  // To measure to higher degrees of precision use the following sensor settings:
  // ADC_256 
  // ADC_512 
  // ADC_1024
  // ADC_2048
  // ADC_4096

  
  // Read pressure from the sensor in mbar.
  pressure_abs = sensor.getPressure(ADC_4096);
  
  // Let's do something interesting with our data.
  
  // Convert abs pressure with the help of altitude into relative pressure
  // This is used in Weather stations.
  pressure_relative = sealevel(pressure_abs, base_altitude);
  
  // Taking our baseline pressure at the beginning we can find an approximate
  // change in altitude based on the differences in pressure.   
  altitude_delta = altitude(pressure_abs , pressure_baseline);

  Serial.print("Pressure abs (mbar)= ");
  Serial.println(pressure_abs);
   
  Serial.print("Pressure relative (mbar)= ");
  Serial.println(pressure_relative); 
  
  Serial.print("Altitude change (m) = ");
  Serial.println(altitude_delta); 
}


// Set everything up
void setup()
{
  // MOTOR Setup *************************************************************************************************************************************
  // Put the motor to Arduino pins 8,9,10
  motorL.attach(8);
  motorR.attach(9);
  motorB.attach(10);

  // Initialize motors to lowest value
  motorL.write(90);
  motorR.write(90);
  motorB.write(90);

  // Required for I/O from Serial monitor
 
  Serial.begin(9600);
//  Serial.println("before delay");
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  ps2x.read_gamepad(false, vibrate);
//  initial_x = map(ps2x.Analog(PSS_LX), 0, 255, 0, 180);
//  Serial.println("initial x");
//  Serial.println(initial_x);
//  initial_y = map(ps2x.Analog(PSS_LY), 0, 255, 0, 180);
//  Serial.println("initial y");
//  Serial.println(initial_y);
//  initial_z = map(ps2x.Analog(PSS_RY), 0, 255, 0, 180);
//  Serial.println("initial z");
//  Serial.println(initial_z);

  // MOTOR Setup **************************************************************************************************************************************

  // IMU Setup ****************************************************************************************************************************************
  Wire.begin();
  filter.begin(50);
// Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
//    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    myIMU.magBias[0] = 344.11;
    myIMU.magBias[1] = 203.31;
    myIMU.magBias[2] = 117.70;
    myIMU.magScale[0] = 0.95;
    myIMU.magScale[1] = 1.03;
    myIMU.magScale[2] = 1.19;
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
//    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }


  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  // IMU Setup *****************************************************************************************************************************************

  // Pressure Sensor Setup *****************************************************************************************************************************

      //Retrieve calibration constants for conversion math.
    sensor.reset();
    sensor.begin();
    
    pressure_baseline = sensor.getPressure(ADC_4096);

  
  // Pressure Sensor Setup *****************************************************************************************************************************

    
  runner.init();
  Serial.println("Initialized scheduler");
  runner.addTask(readGamepad);
  runner.addTask(updateMotorB);
  runner.addTask(updateMotorLR);
  runner.addTask(updateImuTask);
  runner.addTask(updatePressureSensor);

  readGamepad.enable();
}


void loop()
{
 runner.execute();
   //delay(1000);
}



 double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}
  

