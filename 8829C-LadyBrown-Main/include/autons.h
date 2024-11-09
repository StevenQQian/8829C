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
    //  Drive towards the mobile goal in front and clamp 
    driveStraight(-18);
    driveStraight(-5);
    mogo.set(true);
    vexDelay(100);

    // Turn towards the first 2 stack and intake the bottom ring
    turnToHeading(90);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(28);

    // Turn towards the two middle stacks and intake them separately
    turnToHeading(-5);
    driveStraight(14);
    vexDelay(100);
    driveStraight(-8);
    turnToHeading(-21);
    driveStraight(12);
    vexDelay(100);

    // Back out from the middle and drop the mobile goal
    turnToHeading(20);
    driveStraight(-35);
    mogo.set(false);
    spinning = false;
    // Turn towards the last two stack in front of the alliance stake and store the top ring
    turnToHeading(-90);
    blueStoring = true;

    driveStraight(37, 1, 0.5);
    vexDelay(100);

    // Score the ring onto the alliance stake
    turnToHeading(-65);
    driveStraight(-19);
    driveStraight(2);
    turnToHeading(0);
    blueStoring = false;
    driveStraight(-10.5);
    driveStraight(1.2);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(700);

}
int mogoRushFight1() {
    vexDelay(700);
    mogo.set(true);
    return 0;
}

int mogoRushFight2() {
    vexDelay(720);
    blueStoring = true;
    return 0;
}

void redSideRush() {
    imu.setRotation(-99, rotationUnits::deg);
    x = -60;
    y = -60;
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);
    driveStraight(-20, 1, 1, 2800, 250, 56000);
    task t1(mogoRushFight1);
    driveToPoint(-26.75, -52.75, 9000);
    vexDelay(70);
    turnToHeading(-75);
    task t2(mogoRushFight2);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(40, 1, 0.8);

    turnToHeading(0);
    mogo.set(false);
    vexDelay(100);
    driveStraight(14.75);
    turnToHeading(-92);
    driveStraight(-17, 1, 0.7);
    mogo.set(true);
    vexDelay(100);
    blueStoring = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(20);
    turnToHeading(0);
    mogo.set(false);
    blueStoring = true;
    driveStraight(33.25, 1, 0.5);
    turnToHeading(17);
    driveStraight(-19);
    driveStraight(3);
    turnToHeading(90);
    driveStraight(-11);
    driveStraight(1.2);
    blueStoring = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(1000000);
}

#endif // !AUTONS_H