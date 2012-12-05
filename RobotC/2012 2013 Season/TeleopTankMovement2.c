
/*--------------------------------------------------------------------------------------------------------*\
|*                                                                                                        *|
|*                                         - The MAZE Challenge -                                         *|
|*                                            ROBOTC on NXT                                               *|
|*                                                                                                        *|
|*  This program shows the skeleton of a program that could be used to complete a simple maze challenge.  *|
|*                                                                                                        *|
|*                                        ROBOT CONFIGURATION                                             *|
|*    NOTES:                                                                                              *|
|*    1)  To change the forward movement's speed, replace the two "100"'s with the desired speed number.  *|
|*    2)  To change the duration of the movement, replace the "4000" with the desired number of           *|
|*        milliseconds.                                                                                   *|
|*    3)  To program the robot to turn change the polarity of the motor (example line 33)                 *|
|*                                                                                                        *|
|*    MOTORS & SENSORS:                                                                                   *|
|*    [I/O Port]              [Name]              [Type]              [Description]                       *|
|*    Port C                  motorC              NXT                 Right motor                         *|
|*    Port B                  motorB              NXT                 Left motor                          *|
\*--------------------------------------------------------------------------------------------------------*/

task main()
{
   motor[motorC] = 50;   // Motor C is set to run at 50, but could be any number between 10-100.
   motor[motorB] = 50;   // Motor B is set to run at 50, but could be any number between 10-100.
   wait1Msec(2000);      // Replace the value of 2000 for your value

   motor[motorC] = 0;   // Lines 27 through 29 programs the robot to pause for one second
   motor[motorB] = 0;   // Programming wait states throughout the program is a very
   wait1Msec(1000);     // Powerful debugging tool.

   motor[motorC] = 50;   // Lines 31 through 33 programs the robot to turn.  The values 50
   motor[motorB] = -50;  // and 1000 are for illustrative purposes only
   wait1Msec(1000);

   /* To complete the maze you would cut and paste the above code and modify the wait state and the
      power levels so that you can complete the maze problem                                     */
}
