#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     IRSeeker,       sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     ,               sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     armMotor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     rightMotor,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     rampMotor,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     leftMotor,     tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    Claw,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define IR_BEACON_LEFT        4
#define IR_BEACON_CENTER      5
#define IR_BEACON_RIGHT       6
#define SERVO_INITIAL         0
#define SERVO_DELIVERY        127

short servoDestination = SERVO_INITIAL;
short servoDelivery = SERVO_DELIVERY;

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



task main()
{
	int IRValue = 0;

  initializeRobot();

  waitForStart(); // Wait for the beginning of autonomous phase.
  // Position arm and scoop at ir beacon level


  // Move forward ~3
  servo[Claw] = servoDestination;
  motor[leftMotor]=30;
  motor[rightMotor]=30;
  wait1Msec(2000);
  motor[leftMotor]=0;
  motor[rightMotor]=0;

  wait1Msec(100);

    // Determine column that ir beacon is in

  IRValue=SensorValue(IRSeeker);

  switch(IRValue){

  	// IR Beacon in left column
  	case IR_BEACON_LEFT:

  		// Turn robot left
	  	motor[rightMotor]=50;
	  	wait1Msec(1000);
	  	motor[rightMotor]=0;
	  	wait1Msec(100);

	  	// Move robot straight
	   	motor[leftMotor]=30;
	  	motor[rightMotor]=30;
	  	wait1Msec(1300);
	  	motor[leftMotor]=0;
	  	motor[rightMotor]=0;
	  	wait1Msec(100);

	  	// Turn robot right to align with IR beacon
	  	motor[leftMotor]=50;
	  	wait1Msec(1000);
	  	motor[leftMotor]=0;
	  	break;

  	// IR Beacon in center column
  	case IR_BEACON_CENTER:

  		// Do nothing because already aimed at IR beacon
  		break;

  	// IR Beacon in right column
  	case IR_BEACON_RIGHT:

  		// Turn robot right
	  	motor[leftMotor]=50;
	  	wait1Msec(1000);
	  	motor[leftMotor]=0;
	  	wait1Msec(100);

	  	// Move robot straight
	   	motor[leftMotor]=30;
	  	motor[rightMotor]=30;
	  	wait1Msec(1300);
	  	motor[leftMotor]=0;
	  	motor[rightMotor]=0;
	  	wait1Msec(100);

	  	// Turn robot left to align with IR beacon
	  	motor[rightMotor]=50;
	  	wait1Msec(1000);
	  	motor[rightMotor]=0;
	  	break;

	 	// IR Beacon did not match left, center, or right column
  	default:

	  	motor[leftMotor]=-30;
	  	motor[rightMotor]=-30;
	  	wait1Msec(2000);
	  	motor[leftMotor]=0;
	  	motor[rightMotor]=0;
  }

  //Move to IR beacon
    	motor[leftMotor]=23;
  		motor[rightMotor]=23;

  while (true){
  	IRValue = SensorValue(IRSeeker);
  	nMotorEncoder[armMotor] = 0;
  	nMotorEncoderTarget[armMotor]=20;

  	if (IRValue==IR_BEACON_CENTER){
  		motor[leftMotor]=23;
  		motor[rightMotor]=23;
  		wait1Msec(5000);
  		motor[leftMotor]=0;
  		motor[rightMotor]=0;
  	}
  	}
   	if (IRValue>IR_BEACON_CENTER){
  		motor[rightMotor]=0;
  		wait1Msec(5000);
  		motor[leftMotor]=0;
  		motor[rightMotor]=0;
  	}
   	if (IRValue<IR_BEACON_CENTER){
  		motor[leftMotor]=0;
  		wait1Msec(5000);
  		motor[leftMotor]=0;
  		motor[rightMotor]=0;
  	}

  }
  while(true){
  }
}
