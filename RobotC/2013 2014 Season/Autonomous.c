#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     armMotor,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rightMotor,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     scoopMotor,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     leftMotor,     tmotorTetrix, openLoop)

#include "JoystickDriver.c"

void initializeRobot()
{
  // Place code here to initialize servos to starting positions.

  if (nAvgBatteryLevel < 8000) {  // brick under 8.0 volts
	  PlayImmediateTone(1000, 100);  // play a warning tone
    PlaySoundFile("NxtBatteryLow.rso");
  }

  if (externalBatteryAvg < 12000) {  // main battery under 12.0
    int i;
    // play a warning siren once for
    // every tenth of a volt too low
    for (i = 0; i < (14000 - externalBatteryAvg) / 100; i++) {
      PlayImmediateTone(2000, 300);  // play a warning tone
      PlaySoundFile("ExternalBatteryLow.rso");
    }
  }

  return;
}

task main()
{
	initializeRobot();

	waitForStart();

	motor[rightMotor] = 100;
	motor[leftMotor] = 100;
	wait1Msec(1800);
	motor[rightMotor] = 0;
	motor[leftMotor] = 0;

	motor[leftMotor] = 100;
	motor[rightMotor] = 0;
	wait1Msec(1200);
	motor[leftMotor] = 0;
	motor[rightMotor] = 0;

	motor[rightMotor] = 100;
	motor[leftMotor] = 100;
	wait1Msec(1500);
	motor[rightMotor] = 0;
	motor[leftMotor] = 0;

}
