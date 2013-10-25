#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motor2,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     motor3,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     motor1,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motor4,        tmotorTetrix, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define JOYSTICK_Y1_MAX       127
#define JOYSTICK_Y1_MIN       0
#define MOTOR_POWER_UP_MAX    100
#define MOTOR_POWER_DOWN_MAX  30
#define JOYSTICK_DEAD_ZONE    10

#include "JoystickDriver.c"

void initializeRobot()
{
  if (nAvgBatteryLevel < 8000) {
	  PlayImmediateTone(1000, 100);
    PlaySoundFile("NxtBatteryLow.rso");
  }

  if (externalBatteryAvg < 12000) {
    int i;
    for (i = 0; i < (14000 - externalBatteryAvg) / 100; i++) {
      PlayImmediateTone(2000, 300);  // play a warning tone
      PlaySoundFile("ExternalBatteryLow.rso");
    }
  }

  return;
}
task drive()
{

  int lastMessage = 0;
  int motor1Speed = 0;
  int motor2Speed = 0;
  int motor3Speed = 0;
  int motor4Speed = 0;
  int totalMessages = 0;
  int topSpeed = MOTOR_POWER_DOWN_MAX;


  while (true)
  {
	  getJoystickSettings(joystick);

	  if (lastMessage != ntotalMessageCount) {
	  if (true) {

	    ClearTimer(T2);

	    lastMessage = ntotalMessageCount;

	    motor1Speed = joystick.joy1_x2 / JOYSTICK_Y1_MAX * topSpeed;
	    if (abs(motor1Speed) < JOYSTICK_DEAD_ZONE) motor1Speed = 0;

	    motor2Speed = joystick.joy1_x1 / JOYSTICK_Y1_MAX * topSpeed;
	    if (abs(motor2Speed) < JOYSTICK_DEAD_ZONE) motor2Speed = 0;

	    motor3Speed = joystick.joy1_y2  / JOYSTICK_Y1_MAX * topSpeed;
	    if (abs(motor3Speed) < JOYSTICK_DEAD_ZONE) motor3Speed = 0;

	    motor4Speed = joystick.joy1_y1  / JOYSTICK_Y1_MAX * topSpeed;
	    if (abs(motor4Speed) < JOYSTICK_DEAD_ZONE) motor4Speed = 0;

	    motor[motor1] = -motor3Speed;
	    motor[motor2] = -motor4Speed;
	    motor[motor3] = motor3Speed;
	    motor[motor4] = motor4Speed;

	    totalMessages = ntotalMessageCount;

	  } else if (time1[T2] > 200) {

		  PlayImmediateTone(3000, 1);
		  motor[motor1] = 0;
		  motor[motor2] = 0;
		  motor[motor3] = 0;
		  motor[motor4] = 0;

	  }
  }
}
}

task main()
{
  initializeRobot();

  waitForStart();

  StartTask(drive);

  while (true)
  {
      getJoystickSettings(joystick);
    }
}
