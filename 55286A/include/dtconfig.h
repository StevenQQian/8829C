#ifndef DTCONFIG_H
#define DTCONFIG_H
#include "vex.h"


// Default Six Motor Drive
motor lB(-1, gearSetting::ratio6_1, 0);
motor lM(-1, gearSetting::ratio6_1, 1);
motor lF(-1, gearSetting::ratio6_1, 0);

motor rB(-1, gearSetting::ratio6_1, 1);
motor rM(-1, gearSetting::ratio6_1, 0);
motor rF(-1, gearSetting::ratio6_1, 1);

double dt_rpm = 450;
double driveWheelDiameter = 3.25;
double trackingWidth = 0;


// Sensors
rotation horizontalTrackingWheel(-1, 0);
rotation verticalTrackingWheel(-1, 0);
double horizontalOffset = 0;
double verticalOffset = 0;
double verticalDiameter = 2;
double horizontalDiameter = 2;


inertial imu(-1);
optical optic(-1);

// Intake
motor intake(-1, gearSetting::ratio18_1, 0);
motor conveyor(-1, gearSetting::ratio18_1, 1);

// Arm
motor arm(-1, gearSetting::ratio18_1);


// Controller
controller master(primary);

// Brain
brain Brain;
#endif // !DTCONFIG_H