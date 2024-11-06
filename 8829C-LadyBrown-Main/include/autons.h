#ifndef AUTONS_H
#define AUTONS_H

#include "tools.h"
#include "motion.h"
#include "curve.h"
#include "pursuit.h"
#include "intake.h"

void test() {
    // Drive straight forward for 24 inches
    driveStraight(24, 1);
    // Turn towards 90 degrees on the cartesian plane
    // turnToHeading(90, 1);
    // turnToHeading(90);
    // follow(testDrive2, 3000, 1, 12, 0.8);
}

void blueRingSide() {
    imu.setRotation(180, rotationUnits::deg);
    driveStraight(-18);
    driveStraight(-5);
    mogo.set(true);
    vexDelay(100);
    turnToHeading(90);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(28);
    turnToHeading(-5);
    driveStraight(14);
    vexDelay(100);
    driveStraight(-8);
    turnToHeading(-21);
    driveStraight(12);
    vexDelay(100);
    turnToHeading(20);
    driveStraight(-35);
    mogo.set(false);
    spinning = false;
    turnToHeading(-90);
    blueStoring = true;

    driveStraight(37, 1, 0.5);
    vexDelay(100);
    turnToHeading(-65);
    driveStraight(-19);
    driveStraight(2);
    turnToHeading(0);
    blueStoring = false;
    driveStraight(-10.5);
    driveStraight(1.2);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(700);
    // conveyor.spin(fwd, 0, voltageUnits::mV);
    // driveStraight(30);

    vexDelay(100000);
}

#endif // !AUTONS_H