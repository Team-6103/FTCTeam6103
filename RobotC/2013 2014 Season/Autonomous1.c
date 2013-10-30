#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motor2,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     motor3,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     motor1,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motor4,        tmotorTetrix, openLoop, reversed)

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

	motor[motor1] = 100;
	motor[motor3] = 100;
	wait1Msec(40);
	motor[motor1] = 0;
	motor[motor3] = 0;

}
