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

void redRingSide() {
    imu.setRotation(180, rotationUnits::deg);
    //  Drive towards the mobile goal in front and clamp 
    driveStraight(-18);
    driveStraight(-5);
    mogo.set(true);
    vexDelay(100);

    // Turn towards the first 2 stack and intake the bottom ring
    turnToHeading(-90);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(28);

    // Turn towards the two middle stacks and intake them separately
    turnToHeading(5);
    driveStraight(14);
    vexDelay(100);
    driveStraight(-8);
    turnToHeading(21);
    driveStraight(12);
    vexDelay(100);

    // Back out from the middle and drop the mobile goal
    turnToHeading(-20);
    driveStraight(-35);
    mogo.set(false);
    spinning = false;
    // Turn towards the last two stack in front of the alliance stake and store the top ring
    turnToHeading(90);
    blueStoring = true;

    driveStraight(37, 1, 0.5);
    vexDelay(100);

    // Score the ring onto the alliance stake
    turnToHeading(65);
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

int skillsStore1() {
    vexDelay(400);
    redStoring = true;
    return 0;
}

int skillsStore2() {
    vexDelay(1100);
    redStoring = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    return 0;
}

int skillsStore3() {
    vexDelay(300);
    redStoring = true;
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

void blueSideRush() {
    imu.setRotation(-99, rotationUnits::deg);
    x = 60;
    y = -60;
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);
    driveStraight(-20, 1, 1, 2800, 250, 56000);
    task t1(mogoRushFight1);
    driveToPoint(26.75, -52.75, 9000);
    vexDelay(70);
    turnToHeading(75);
    task t2(mogoRushFight2);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(40, 1, 0.8);

    turnToHeading(0);
    mogo.set(false);
    vexDelay(100);
    driveStraight(14.75);
    turnToHeading(92);
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
    turnToHeading(-17);
    driveStraight(-19);
    driveStraight(3);
    turnToHeading(-90);
    driveStraight(-11);
    driveStraight(1.2);
    blueStoring = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(1000000);
}

void skills() {
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);
    imu.setRotation(-90, rotationUnits::deg);
    ringSorting = false;
    x = -64.6;
    y = -24;
    driveStraight(-10, 1, 0.8);
    mogo.set(true);
    vexDelay(60);
    turnToHeading(90, 1, 1, 0, 360, 33, 2600);
    Curve c1 = Curve(Vector2d(-32.859, -24.11), Vector2d(-22.721, -31.382), Vector2d(-16.788, -37.333), Vector2d(-8.413, -56.743));
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    follow(c1, 4000, 0.6, 12);
    vexDelay(1000);
    turnToPoint(-24.7, -56);
    Curve c2 = Curve(Vector2d(-29.553, -46.148), Vector2d(-42.115, -50.352), Vector2d(-49.804, -51.767), Vector2d(-62.236, -50.691));

    follow(c2, 4000, 0.5, 12);
    vexDelay(800);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    spinning = false;
    turnToHeading(225, 1, 1, 0, 360, 33, 2600);
    driveStraight(-5);
    turnToPoint(-54.5, -70);
    task tt(skillsStore1);
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    vexDelay(300);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(11);
    turnToPoint(0, -15);
    mogo.set(false);
    driveStraight(-9.5);
    vexDelay(50);
    driveToPoint(-57, -5, 7000, 0, 2300, 230, 60000);

    redStoring = false;
    turnToHeading(90, 1, 1, 0, 360, 33, 2600);
    driveStraight(-15.6, 1, 0.8);
    driveStraight(1.2);

    x = -60;
    y = 0;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(700);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    driveStraight(15.2);
    turnToHeading(180, 1, 1, 0, 360, 33, 2600);
    driveStraight(-19, 1, 0.6);
    mogo.set(true);
    vexDelay(100);
    turnToHeading(92);
    Curve c3 = Curve(Vector2d(-32.969, 16.052), Vector2d(-23.933, 24.443), Vector2d(-13.576, 35.551), Vector2d(-1.091, 48.261));
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    follow(c3, 4000, 0.6, 12);

    vexDelay(700);
    turnToHeading(-106, 1, 1, 0, 360, 33, 2600);
    Curve c4 = Curve(Vector2d(-28.451, 47.845), Vector2d(-37.487, 43.437), Vector2d(-45.641, 43.658), Vector2d(-53, 44.404));

    follow(c4, 3000, 0.5, 12);

    vexDelay(300);
    turnToHeading(-60, 1, 1, 0, 360, 33, 2600);
    driveStraight(-7);
    turnToHeading(0, 1, 1, 0, 360, 33, 2600);
    driveStraight(11);
    vexDelay(500);
    turnToHeading(135, 1, 1, 0, 360, 33, 2600);
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    mogo.set(false);
    spinning = false;
    driveStraight(-10);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    Curve c5 = Curve(Vector2d(-48.027, 60.11), Vector2d(-32.423, 54.99), Vector2d(-7.311, 31.093), Vector2d(40.831, 18.981));
    task ttt(skillsStore2);

    follow(c5, 5000, 0.6, 15);
    turnToHeading(0, 1, 1, 0, 360, 33, 2600);
    driveStraight(-26, 1, 0.7);
    driveToPoint(48, -4.5, 8000);
    turnToHeading(-90, 1, 1, 0, 360, 33, 2600);
    driveStraight(-17.9, 1, 0.8);
    x = 60;
    y = 0;
    vexDelay(200);
    redStoring = false;
    driveStraight(1);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(600);
    ringSorting = true;
    conveyor.spin(fwd, 0, voltageUnits::mV);
    driveStraight(4);
    turnToHeading(-175, 1, 1, 0, 360, 33, 2600);
    driveStraight(-53.2, 1, 1);
    
    vexDelay(200);
    driveToPoint(45.5, 0, 8000);
    turnToHeading(0, 1, 1, 0, 360, 33, 2600);
    driveStraight(-15, 1, 0.5);
    mogo.set(true);
    vexDelay(100);
    turnToPoint(24, -24);
    Curve c6 = Curve(Vector2d(16.095, -23.516), Vector2d(19.508, -33.756), Vector2d(21.215, -38.876), Vector2d(23.653, -47.166));
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    follow(c6, 5000, 0.6, 12);
    vexDelay(200);
    turnToHeading(-270, 1, 1, 0, 360, 33, 2600);
    driveStraight(9.75, 1, 0.5);
    vexDelay(200);
    driveStraight(-3);
    turnToHeading(-225, 1, 1, 0, 360, 33, 2600);
    spinning = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    task tttt(skillsStore3);
    driveStraight(10.5);
    vexDelay(100);
    turnToHeading(-432, 1, 1, 0, 360, 33, 2600);
    mogo.set(false);
    driveStraight(-18, 1, 0.7);
    targetLadybrownDeg = 24.5;
    redStoring = false;
    driveToPoint(-5.5, -50, 7000);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    turnToPoint(-4.5, -70);
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    targetLadybrownDeg = 132;
    vexDelay(500);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(18.5, 1, 0.6);
    driveStraight(-20);
    vexDelay(1000000);
}
#endif // !AUTONS_H