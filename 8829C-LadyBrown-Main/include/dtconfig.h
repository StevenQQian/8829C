#ifndef DTCONFIG_H
#define DTCONFIG_H
#include "vex.h"


// Default Six Motor Drive
motor lB(PORT12, ratio6_1, 1);
motor lM(PORT11, ratio6_1, 1);
motor lF(PORT16, ratio6_1, 1);

motor rB(PORT8, ratio6_1, 0);
motor rM(PORT2, ratio6_1, 0);
motor rF(PORT1, ratio6_1, 0);

double dt_rpm = 450;
double driveWheelDiameter = 2.75;
double trackingWidth = 14.5;


// Sensors
rotation horizontalTrackingWheel(PORT17, 1);
rotation verticalTrackingWheel(PORT9, 0);
double horizontalOffset = 1;
double verticalOffset = 0.25;
double verticalDiameter = 2.75;
double horizontalDiameter = 2;


inertial imu(PORT7);
optical optic_1(PORT3);
optical optic_2(PORT13);


// Intake
// motor intake(PORT14, gearSetting::ratio18_1, 1);
motor conveyor(PORT14, gearSetting::ratio6_1, 1);

// Arm
motor arm(-1, gearSetting::ratio18_1);


// Controller
controller master(primary);

// Brain
brain Brain;
#endif // !DTCONFIG_H