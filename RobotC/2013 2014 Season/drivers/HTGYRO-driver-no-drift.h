/*!@addtogroup HiTechnic
 * @{
 * @defgroup htgyro Gyroscopic Sensor
 * HiTechnic Gyroscopic Sensor
 * @{
 */

/*
 * $Id: HTGYRO-driver.h 98 2012-08-04 09:02:53Z xander $
 */

#ifndef __HTGYRO_H__
#define __HTGYRO_H__
/** \file HTGYRO-driver.h
 * \brief HiTechnic Gyroscopic Sensor driver
 *
 * HTGYRO-driver.h provides an API for the HiTechnic Gyroscopic Sensor.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: Renamed HTGYROgetCalibration to HTGYROreadCal<br>
 *        Renamed HTGYROsetCalibration to HTGYROsetCal<br>
 *        Renamed HTGYROcalibrate to HTGYROstartCal<br>
 *        Added SMUX functions
 * - 0.3: Removed some of the functions requiring SPORT/MPORT macros
 * - 0.4: Removed "NW - No Wait" functions\n
 *        Replaced array structs with typedefs\n
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where its due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.08 AND HIGHER.
 * \author Xander Soldaat (mightor_at_gmail.com)
 * \date 20 February 2011
 * \version 0.4
 * \example HTGYRO-test1.c
 * \example HTGYRO-test2.c
 * \example HTGYRO-SMUX-test1.c
 */

#pragma systemFile

#ifndef __COMMON_H__
#include "common.h"
#endif

float HTGYROreadRot(tSensors link);
float HTGYROstartCal(tSensors link);
float HTGYROreadCal(tSensors link);
// void HTGYROsetCal(tSensors link, int offset);

#ifdef __HTSMUX_SUPPORT__
float HTGYROreadRot(tMUXSensor muxsensor);
float HTGYROstartCal(tMUXSensor muxsensor);
float HTGYROreadCal(tMUXSensor muxsensor);
void HTGYROsetCal(tMUXSensor muxsensor, int offset);
#endif // __HTSMUX_SUPPORT__

float HTGYRO_offsets[][] = {{620.0, 620.0, 620.0, 620.0}, /*!< Array for offset values.  Default is 620 */
                          {620.0, 620.0, 620.0, 620.0},
                          {620.0, 620.0, 620.0, 620.0},
                          {620.0, 620.0, 620.0, 620.0}};
float gyroIdleRange = 1.2;
float gyroOffset = 620.0;

/**
 * Read the value of the gyro
 * @param link the HTGYRO port number
 * @return the value of the gyro
 */
float HTGYROreadRot(tSensors link) {

  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorAnalogInactive) {
    SetSensorType(link, sensorAnalogInactive);
    wait1Msec(100);
  }

  float sensorValue = SensorValue[link];

  if (abs(sensorValue - gyroOffset) < gyroIdleRange) {
    return 0.0;
  } else {
    return (sensorValue - gyroOffset);
  }
}


/**
 * Read the value of the gyro
 * @param muxsensor the SMUX sensor port number
 * @return the value of the gyro
 */
#ifdef __HTSMUX_SUPPORT__
float HTGYROreadRot(tMUXSensor muxsensor) {
  return HTSMUXreadAnalogue(muxsensor) - HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)];
}
#endif // __HTSMUX_SUPPORT__


/**
 * Calibrate the gyro by calculating the average offset of 5 raw readings.
 * @param link the HTGYRO port number
 * @return the new offset value for the gyro
 */
float HTGYROstartCal(tSensors link) {

  float sensorValue = 0.0;
  float gyroIdleReadCount = 0.0;
  float gyroIdleReadSum = 0.0;


  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorAnalogInactive) {
    SetSensorType(link, sensorAnalogInactive);
    wait1Msec(100);
  }

  // Take 500 readings and average them out per http://nxttime.wordpress.com/2010/11/03/gyro-offset-and-drift/
  for (int i = 0; i < 100; i++) {
    sensorValue = SensorValue[link];
    gyroIdleReadCount += 1.0;
    gyroIdleReadSum += sensorValue;
    wait1Msec(2);
  }

  // Store new offset
  gyroOffset = gyroIdleReadSum / gyroIdleReadCount;

  // Return new offset value
  return gyroOffset;
}


/**
 * Calibrate the gyro by calculating the average offset of 50 raw readings.
 * @param muxsensor the SMUX sensor port number
 * @return the new offset value for the gyro
 */
#ifdef __HTSMUX_SUPPORT__
float HTGYROstartCal(tMUXSensor muxsensor) {
  long _avgdata = 0;

  // Take 5 readings and average them out
  for (int i = 0; i < 50; i++) {
    _avgdata += HTSMUXreadAnalogue(muxsensor);
    wait1Msec(50);
  }

  // Store new offset
  HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)] = (_avgdata / 50.0);

  // Return new offset value
  return HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)];
}
#endif // __HTSMUX_SUPPORT__


/**
 * Override the current offset for the gyro manually
 * @param link the HTGYRO port number
 * @param offset the new offset to be used
 */
//#define HTGYROsetCal(link, offset) HTGYRO_offsets[link][0] = offset
void HTGYROsetCal(tSensors link, int offset) {
  HTGYRO_offsets[link][0] = offset;
}


/**
 * Override the current offset for the gyro manually
 * @param muxsensor the SMUX sensor port number
 * @param offset the new offset to be used
 */
#ifdef __HTSMUX_SUPPORT__
//#define HTGYROsetCal(muxsensor, offset) HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)] = offset
void HTGYROsetCal(tMUXSensor muxsensor, int offset) {
  HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)] = offset;
}
#endif // __HTSMUX_SUPPORT__


/**
 * Retrieve the current offset for the gyro
 * @param link the HTGYRO port number
 * @return the offset value for the gyro
 */
float HTGYROreadCal(tSensors link) {
  return HTGYRO_offsets[link][0];
}


/**
 * Retrieve the current offset for the gyro
 * @param muxsensor the SMUX sensor port number
 * @return the offset value for the gyro
 */
#ifdef __HTSMUX_SUPPORT__
float HTGYROreadCal(tMUXSensor muxsensor) {
  return HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)];
}
#endif // __HTSMUX_SUPPORT__

#endif // __HTGYRO_H__

/*
 * $Id: HTGYRO-driver.h 98 2012-08-04 09:02:53Z xander $
 */
/* @} */
/* @} */
