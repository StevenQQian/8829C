#ifndef DTCONFIG_H
#define DTCONFIG_H
#include "vex.h"


// Default Six Motor Drive
motor lB(PORT13, ratio6_1, 1);
motor lM(PORT12, ratio6_1, 1);
motor lF(PORT11, ratio6_1, 1);

motor rB(PORT21, ratio6_1, 0);
motor rM(PORT9, ratio6_1, 0);
motor rF(PORT10, ratio6_1, 0);

double dt_rpm = 450;
double driveWheelDiameter = 3.25;
double trackingWidth = 13.5;


// Sensors
rotation horizontalTrackingWheel(PORT15, 0);
rotation verticalTrackingWheel(PORT17, 0);
double horizontalOffset = -2.25;
double verticalOffset = -0.25; // 0.25
double verticalDiameter = 2;
double horizontalDiameter = 2;
vex::distance dis(PORT16);

inertial imu(PORT14);
optical optic_1(PORT4);
optical optic_2(PORT8);
vex::distance disFront(PORT5);
vex::distance disRight(PORT2);
// Intake
// motor intake(PORT14, gearSetting::ratio18_1, 1);
motor conveyor(PORT6, gearSetting::ratio6_1, 1);

// Arm
motor arm(-1, gearSetting::ratio18_1);


// Controller
controller master(primary);

// Brain
brain Brain;
#endif // !DTCONFIG_H