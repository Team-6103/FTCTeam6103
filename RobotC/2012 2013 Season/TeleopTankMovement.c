#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Motor,  mtr_S1_C1_1,     leftMotor,     tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rightMotor,    tmotorNormal, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define MOTOR_SPEED_MAX     127
#define MOTOR_SPEED_MIN     0
#define JOYSTICK_DEAD_ZONE  10

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Tele-Operation Mode Code Template
//
// This file contains a template for simplified creation of an tele-op program for an FTC
// competition.
//
// You need to customize two functions with code unique to your specific robot.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of tele-op mode, you may want to perform some initialization on your robot
// and the variables within your program.
//
// In most cases, you may not have to add any code to this function and it will remain "empty".
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeRobot()
{
  // Place code here to sinitialize servos to starting positions.
  // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.

  if (nAvgBatteryLevel < 8000) {  // brick under 8.0 volts
	  PlayImmediateTone(1000, 100);  // play a warning tone
//    PlaySoundFile("NxtBatteryLow.rso");
  }

  if (externalBatteryAvg < 14000) {  // main battery under 14.0
    int i;
    // play a warning siren once for
    // every tenth of a volt too low
    for (i = 0; i < (14000 - externalBatteryAvg) / 100; i++) {
      PlayImmediateTone(2000, 300);  // play a warning tone
//      PlaySoundFile("ExternalBatteryLow.rso");
    }
  }

  return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                         Main Task
//
// The following is the main code for the tele-op robot operation. Customize as appropriate for
// your specific robot.
//
// Game controller / joystick information is sent periodically (about every 50 milliseconds) from
// the FMS (Field Management System) to the robot. Most tele-op programs will follow the following
// logic:
//   1. Loop forever repeating the following actions:
//   2. Get the latest game controller / joystick settings that have been received from the PC.
//   3. Perform appropriate actions based on the joystick + buttons settings. This is usually a
//      simple action:
//      *  Joystick values are usually directly translated into power levels for a motor or
//         position of a servo.
//      *  Buttons are usually used to start/stop a motor or cause a servo to move to a specific
//         position.
//   4. Repeat the loop.
//
// Your program needs to continuously loop because you need to continuously respond to changes in
// the game controller settings.
//
// At the end of the tele-op period, the FMS will autonmatically abort (stop) execution of the program.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Put the main driver control loop in its own task so
// the driver never loses control of the robot!

task drive()
{
  int threshold = 15; //to avoid unnecessary movement

  while(true) //infinite loop
  {
    getJoystickSettings(joystick); //retrieves data from the joystick

    if(abs(joystick.joy1_y1) > threshold)
    {
      motor[leftMotor] = joystick.joy1_y1; //y1 controller moves motorD
    }
    else
    {
      motor[leftMotor] = 0;
    }

    if(abs(joystick.joy1_y2) > threshold)
    {
      motor[rightMotor] = joystick.joy1_y2; //y2 controller moves motorE
    }
    else
    {
      motor[rightMotor] = 0;
    }
	}
}

task main()
{
  // initializeRobot();

  waitForStart();   // wait for start of tele-op phase

  StartTask(drive);  // give driver control of the wheels

  while (true)
  {
	  // Put code to handle all your other tele-op controls
    // in here, such as grippers, scoring systems, harvesters
    // etc. They can use wait commands for timing without
    // affecting the driver's ability to control the robot's
    // drive train.

  }
}
