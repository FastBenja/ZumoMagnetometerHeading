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

Zumo32U4IMU imu;

int16_t maxX = -10000, maxY = 0, minX = 0, minY = 10000;

// Function to convert magnetometer readings to degrees off North
float calculateHeading(float magX, float magY, float xMin, float xMax, float yMin, float yMax)
{
  // Normalize the X and Y values to the range -1 to 1
  float normX = 2 * (magX - xMin) / (xMax - xMin) - 1;
  float normY = 2 * (magY - yMin) / (yMax - yMin) - 1;

  // Calculate the heading in radians
  float heading = atan2(normY, normX);

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
  if (imu.magDataReady())
  {
    imu.readMag();

    // Get magnetometer readings
    int16_t x = imu.m.x;
    int16_t y = imu.m.y;
    int16_t z = imu.m.z;

    if (x > maxX)
    {
      maxX = x;
    }
    if (y > maxY)
    {
      maxY = y;
    }
    if (x < minX)
    {
      minX = x;
    }
    if (y < minY)
    {
      minY = y;
    }

    // Calculate the heading
    float heading = calculateHeading(x, y, minX, maxX, minY, maxY);
    
    Serial.println('$' + String(x) + ' ' + String(y) + ' ' + String(z) + ' ' + String(heading) + ' ' + String(minX) + ' ' + String(maxX) + ' ' + String(minY) + ' ' + String(maxY) + ';');
  }
  delay(100);
}