#ifndef DTCONFIG_H
#define DTCONFIG_H
#include "vex.h"


// Default Six Motor Drive
motor lB(PORT17, ratio6_1, 1);
motor lM(PORT20, ratio6_1, 1);
motor lF(PORT19, ratio6_1, 1);

motor rB(PORT3, ratio6_1, 0);
motor rM(PORT12, ratio6_1, 0);
motor rF(PORT11, ratio6_1, 0);

double dt_rpm = 450;
double driveWheelDiameter = 2.75;
double trackingWidth = 14.5;


// Sensors
rotation horizontalTrackingWheel(PORT2, 1);
rotation verticalTrackingWheel(PORT18, 1);
double horizontalOffset = 2;
double verticalOffset = 0;
double verticalDiameter = 2.75;
double horizontalDiameter = 2;


inertial imu(PORT10);
optical optic_1(-1);
optical optic_2(-1);


// Intake
motor intake(PORT14, gearSetting::ratio18_1, 1);
motor conveyor(PORT13, gearSetting::ratio6_1, 1);

// Arm
motor arm(-1, gearSetting::ratio18_1);


// Controller
controller master(primary);

// Brain
brain Brain;
#endif // !DTCONFIG_H