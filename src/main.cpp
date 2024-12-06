/**
 * This example will read the magnetometer off of the Pololu Zumo32U4 and calculate the heading.
 * First the min and max readings have to be found. This is done continuously. 
 * Therefore the heading will not be precise befor the Zumo have spun a couple of rounds.
 * 
 * !If the motors area spun by hand or by the Zumo,
 * !you will have to disable the min and max updates as they will be thrown off by the big disturbances.
 * 
 * The raw readings, the min and max values and the calculated heading are all sent over serial.
 * You can use this: https://github.com/CieNTi/serial_port_plotter.git tool to plot the data.
 * 
 * */

#include <Wire.h>
#include <LSM303.h>
#include <Zumo32U4.h>
#include <Arduino.h>

struct MagnetometerReading {
    float x;
    float y;
};


Zumo32U4IMU imu;

// Transformation matrix (2x2)
const float transformationMatrix[2][2] = {
    {0.00078615, -0.00055139},  // Replace with your computed values
    {0.00125332, 0.00178694}   // Replace with your computed values
};

// Calibration parameters
const float offsetX = -10.0; // Replace with your actual offset for X
const float offsetY = 5.0;   // Replace with your actual offset for Y


MagnetometerReading calibrateMagnetometer(float rawX, float rawY) {
    // Step 1: Offset correction
    float centeredX = rawX - offsetX;
    float centeredY = rawY - offsetY;

    // Step 2: Apply the transformation matrix
    float calibratedX = transformationMatrix[0][0] * centeredX + transformationMatrix[0][1] * centeredY;
    float calibratedY = transformationMatrix[1][0] * centeredX + transformationMatrix[1][1] * centeredY;

    Serial.println('$' + String(imu.m.x) + ' ' + String(imu.m.y) + ' ' + String(imu.m.z) + ' ' + String(calibratedX) + ' ' + String(calibratedY) + ';');

    // Return the calibrated values as a struct
    MagnetometerReading calibratedReading = {calibratedX, calibratedY};
    return calibratedReading;
}


// Function to convert magnetometer readings to degrees off North
float calculateHeading()
{
  //Wait for data ready
    while (!imu.magDataReady())
  {
  }

  //Read data
    imu.readMag();
  
// Calibrate the readings
    MagnetometerReading calibrated = calibrateMagnetometer(imu.m.x, imu.m.y);

  // Calculate the heading in radians
  float heading = atan2(calibrated.x, calibrated.y);

  // Convert radians to degrees
  float headingDegrees = heading * (180.0 / M_PI);

  // Normalize to 0-360 degrees
  if (headingDegrees < 0)
  {
    headingDegrees += 360.0;
  }

  return headingDegrees;
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
}

void loop()
{

calculateHeading();

//    Serial.println('$' + String(imu.m.x) + ' ' + String(imu.m.y) + ' ' + String(imu.m.z) + ' ' + String(heading) + ';');

  delay(100);
}