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
#define ARM_MOTOR_POWER_UP    75
#define ARM_MOTOR_POWER_DOWN  30
#define JOYSTICK_DEAD_ZONE    10
#define NUDGE_DURATION        100
#define NUDGE_POWER           30
#define NUDGE_DELAY           250

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

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
task drive()
{

  // Initialize variables
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

	    // New joystick messages have been received!
	    // Set your drive motors based on user input
	    // here.

	    motor1Speed = joystick.joy1_x2 / JOYSTICK_Y1_MAX * topSpeed; // Map the leftMotorSpeed variable to joystick 1_y1
	    if (abs(motor1Speed) < JOYSTICK_DEAD_ZONE) motor1Speed = 0; // Make sure that the joystick isn't inside dead zone

	    motor2Speed = joystick.joy1_x1 / JOYSTICK_Y1_MAX * topSpeed; // Map the rightMotorSpeed variable to joystick 1_y2
	    if (abs(motor2Speed) < JOYSTICK_DEAD_ZONE) motor2Speed = 0; // Make sure that the joystick isn't inside dead zone

	    motor3Speed = joystick.joy1_y2  / JOYSTICK_Y1_MAX * topSpeed; // Map the armMotorSpeed variable to joystick 2_y2
	    if (abs(motor3Speed) < JOYSTICK_DEAD_ZONE) motor3Speed = 0; // Make sure that the joystick isn't inside dead zone

	    motor4Speed = joystick.joy1_y1  / JOYSTICK_Y1_MAX * topSpeed; // Map the teleMotorSpeed variable to joystick 2_y1
	    if (abs(motor4Speed) < JOYSTICK_DEAD_ZONE) motor4Speed = 0; // Make sure that the joystick isn't inside dead zone

	    motor[motor1] = -motor3Speed; // Set the motor armMotor speed as armMotorSpeed
	    motor[motor2] = -motor4Speed; // Set the motor leftMotor speed as leftMotorSpeed
	    motor[motor3] = motor3Speed; // Set the motor rightMotor speed as rightMotorSpeed
	    motor[motor4] = motor4Speed;
/*
  	  if (joy1Btn(5) == 1) {
  	    // Power up
	      topSpeed = MOTOR_POWER_UP_MAX;
	    }

	    if (joy1Btn(7) == 1) {
	      // Power down
	      topSpeed = MOTOR_POWER_DOWN_MAX;
	    }
  	  if (joy2Btn(2) == 1) {
  	    // Power up
	      armTopSpeed = ARM_MOTOR_POWER_DOWN;
	    }
  	  if (joy1Btn(4) == 1) {
  	    // Power up
	      armTopSpeed = ARM_MOTOR_POWER_UP;
	    }

	    if (joy1Btn(4) == 1) {
	      // Nudge forward
	      motor[leftMotor] = NUDGE_POWER;
	      motor[rightMotor] = NUDGE_POWER;
	      wait1Msec(NUDGE_DURATION);
	      motor[leftMotor] = 0;
	      motor[rightMotor] = 0;
	      wait1Msec(NUDGE_DELAY);
	    }

	    if (joy1Btn(2) == 1) {
	      // Nudge backward
	      motor[leftMotor] = NUDGE_POWER * -1;
	      motor[rightMotor] = NUDGE_POWER * -1;
	      wait1Msec(NUDGE_DURATION);
	      motor[leftMotor] = 0;
	      motor[rightMotor] = 0;
	      wait1Msec(NUDGE_DELAY);
	    }

	    if (joy1Btn(1) == 1) {
	      // Nudge left
	      motor[leftMotor] = 0;
	      motor[rightMotor] = NUDGE_POWER;
	      wait1Msec(NUDGE_DURATION);
	      motor[leftMotor] = 0;
	      motor[rightMotor] = 0;
	      wait1Msec(NUDGE_DELAY);
	    }

	    if (joy1Btn(3) == 1) {
	      // Nudge right
	      motor[leftMotor] = NUDGE_POWER;
	      motor[rightMotor] = 0;
	      wait1Msec(NUDGE_DURATION);
	      motor[leftMotor] = 0;
	      motor[rightMotor] = 0;
	      wait1Msec(NUDGE_DELAY);
	    }
	    if (joy2Btn(1) == 1) {
	      // Move Ramp
	      motor[rampMotor] = RAMP_POWER;
	      wait1Msec(RAMP_DURATION);
	      motor[rampMotor] = 0;
	      wait1Msec(NUDGE_DELAY);
	    }
	    if (joy2Btn(3) == 1) {
	      // Nudge left
	      motor[rampMotor] = RAMP_POWER * -1;
	      wait1Msec(RAMP_DURATION);
	      motor[rampMotor] = 0;
	      wait1Msec(NUDGE_DELAY);
	    }
*/

	    totalMessages = ntotalMessageCount;

	  } else if (time1[T2] > 200) {

		  // We have not received a packet in over two-tenths of a
		  // second, which probably means communications have been
		  // lost.
		  // Put code to stop all drive motors to avoid
		  // damaging the robot here.

		  PlayImmediateTone(3000, 1);  // play a warning tone

		  // Stop motors
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
  initializeRobot();   // Initialize The Robot's Servos and Motor's

  waitForStart();   // wait for start of tele-op phase

  StartTask(drive);  // give driver control of the wheels

  while (true)
  {
      getJoystickSettings(joystick);
    }
}
