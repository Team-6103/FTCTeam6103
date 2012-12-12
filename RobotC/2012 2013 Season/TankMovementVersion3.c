#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     rightMotor,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotor,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     armMotor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    Claw,                 tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define JOYSTICK_Y1_MAX       127
#define JOYSTICK_Y1_MIN       0
#define MOTOR_POWER_UP_MAX    100
#define MOTOR_POWER_DOWN_MAX  20
#define JOYSTICK_DEAD_ZONE    10
#define NUDGE_DURATION        100
#define NUDGE_POWER           30
#define NUDGE_DELAY           250
#define SERVO_INITIAL 127   // Replace with the servo's real initial position
#define SERVO_RATE    2     // Larger = faster.  Be careful to give the servo time to move.
#define FPS           60.0  // Rate at which loop repeats.  Also will affect speed of servo.
#define TOPHAT_UP     0
#define TOPHAT_DOWN   4

// Variable stores desired position for servo.  This is the value sent to the servo[] array.
short servoDestination = SERVO_INITIAL;

// Time in milliseconds of one frame.
const float OneFrameMS = 1000.0 / FPS;

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

// Put the main driver control loop in its own task so
// the driver never loses control of the robot!

task drive()
{

  // Initialize variables
  int lastMessage = 0;
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  int armMotorSpeed = 0;
  int totalMessages = 0;
  int topSpeed = MOTOR_POWER_DOWN_MAX;
  int armTopSpeed = 20;

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

	    leftMotorSpeed = joystick.joy1_y1 / JOYSTICK_Y1_MAX * topSpeed;
	    if (abs(leftMotorSpeed) < JOYSTICK_DEAD_ZONE) leftMotorSpeed = 0;

	    rightMotorSpeed = joystick.joy1_y2 / JOYSTICK_Y1_MAX * topSpeed;
	    if (abs(rightMotorSpeed) < JOYSTICK_DEAD_ZONE) rightMotorSpeed = 0;

	    armMotorSpeed = joystick.joy2_y2 / JOYSTICK_Y1_MAX * armTopSpeed;
	    if (abs(armMotorSpeed) < JOYSTICK_DEAD_ZONE) armMotorSpeed = 0;

	    motor[armMotor] = armMotorSpeed;
	    motor[leftMotor] = leftMotorSpeed;
	    motor[rightMotor] = rightMotorSpeed;

  	  if (joy1Btn(5) == 1) {
  	    // Power up
	      topSpeed = MOTOR_POWER_UP_MAX;
	    }

	    if (joy1Btn(7) == 1) {
	      // Power down
	      topSpeed = MOTOR_POWER_DOWN_MAX;
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

	    totalMessages = ntotalMessageCount;

	  } else if (time1[T2] > 200) {

		  // We have not received a packet in over two-tenths of a
		  // second, which probably means communications have been
		  // lost.
		  // Put code to stop all drive motors to avoid
		  // damaging the robot here.

		  PlayImmediateTone(3000, 1);  // play a warning tone

		  // Stop motors
		  motor[leftMotor] = 0;
		  motor[rightMotor] = 0;

	  }
  }
}
}

task main()
{
  initializeRobot();

  waitForStart();   // wait for start of tele-op phase

  StartTask(drive);  // give driver control of the wheels

  while (true)
  {
      getJoystickSettings(joystick);

      // D-pad direction is up?
      if (joystick.joy1_TopHat == TOPHAT_UP)
      { servoDestination += SERVO_RATE; }

      // D-pad direction is down?
      if (joystick.joy1_TopHat == TOPHAT_DOWN)
      { servoDestination -= SERVO_RATE; }

      // Keep servo position values within the range [0, 255].
      servoDestination = (servoDestination > 255) ? 255 : (((servoDestination < 0) ? 0 : servoDestination));

      // Send destination to servo.
      servo[Claw] = servoDestination;

      // Don't increment variables too quickly.
      wait1Msec(OneFrameMS);
    }
}
