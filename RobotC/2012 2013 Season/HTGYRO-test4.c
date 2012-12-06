#pragma config(Sensor, S3,     HTGYRO,         sensorI2CCustomFastSkipStates)
#pragma config(Motor,  motorB,          rightMotor,    tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorNormal, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: HTGYRO-test2.c 98 2012-08-04 09:02:53Z xander $
 */

/**
 * HTGYRO-driver.h provides an API for the HiTechnic Gyroscopic Sensor.  This program
 * demonstrates how to use that API.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.08 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 06 April 2012
 * version 0.1
 */

//#include "drivers/HTGYRO-driver-no-drift.h"

void turnTask(float turnDegrees, int motorSpeed)
{
  float rotSpeed = 0;
  float heading = 0;

  // Calibrate the gyro, make sure you hold the sensor still
  HTGYROstartCal(HTGYRO);

  // Get current program time in milliseconds.
  int lastPgmTime = nPgmTime;
  int currentPgmTime = nPgmTime;

  // Begin moving robot
  if (turnDegrees > 0)
  {
    motor[leftMotor] = motorSpeed;
    motor[rightMotor] = motorSpeed * -1;
  } else
  {
    motor[rightMotor] = motorSpeed;
    motor[leftMotor] = motorSpeed * -1;
  }

  while (abs(heading) < abs(turnDegrees))
  {

    // Read the current rotation speed
    if (lastPgmTime + 20 < nPgmTime)
    {

      currentPgmTime = nPgmTime;

      rotSpeed = HTGYROreadRot(HTGYRO);

	    // Calculate the new heading by adding the amount of degrees
	    // we've turned in the last 20ms
	    // If our current rate of rotation is 100 degrees/second,
	    // then we will have turned 100 * (20/1000) = 2 degrees since
	    // the last time we measured.
	    heading += rotSpeed * (((float)currentPgmTime - (float)lastPgmTime) / 1000);

	    lastPgmTime = currentPgmTime;

	    // Display our current heading on the screen
	    nxtDisplayCenteredBigTextLine(3, "%3.2f", heading);
   }

  }

  // stop motors
  motor[leftMotor] = 0;
  motor[rightMotor] = 0;


}

task main ()
{

  int speed = 35;

  PlayTone(784, 15);
  turnTask(180, speed);
  wait1Msec(1000);

  PlayTone(784, 15);
  turnTask(-180, speed);
  wait1Msec(1000);

  PlayTone(784, 15);
  turnTask(360, speed);
  wait1Msec(1000);

  PlayTone(784, 15);
  turnTask(-360, speed);
  wait1Msec(1000);

}