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
    driveStraight(-6);
    mogo.set(true);
    vexDelay(100);

    // Turn towards the first 2 stack and intake the bottom ring
    turnToHeading(90, 1, 1, 0, 360, 33, 2600);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(30);

    // Turn towards the two middle stacks and intake them separately
    turnToHeading(-3);
    driveStraight(17);
    vexDelay(100);
    driveStraight(-15);
    turnToHeading(-21);
    driveStraight(19);
    vexDelay(100);

    // Back out from the middle and drop the mobile goal
    turnToHeading(20);
    driveStraight(-36.7, 1, 0.7);
    spinning = false;
    // Turn towards the last two stack in front of the alliance stake and store the top ring
    turnToHeading(-90);
    mogo.set(false);
    blueStoring = true;

    driveStraight(37, 1, 0.6);
    vexDelay(100);

    // Score the ring onto the alliance stake
    turnToHeading(-65, 1, 1, 0, 360, 33, 2600);
    lB.setBrake(coast);
  lM.setBrake(coast);
  lF.setBrake(coast);

  rB.setBrake(coast);
  rM.setBrake(coast);
  rF.setBrake(coast);
    driveStraight(-15);
    driveStraight(2.5);
    turnToHeading(0);
    blueStoring = false;
    driveStraight(-9.8);
    driveStraight(1.9);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(1300);
    // setDrive(12000, 12000);
    // vexDelay(500);
    // setDrive(0, 0);
}

void redRingSide() {
    imu.setRotation(-180, rotationUnits::deg);
    //  Drive towards the mobile goal in front and clamp 
    driveStraight(-18);
    driveStraight(-6);
    mogo.set(true);
    vexDelay(100);

    // Turn towards the first 2 stack and intake the bottom ring
    turnToHeading(-90, 1, 1, 0, 360, 33, 2600);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(26);

    // Turn towards the two middle stacks and intake them separately
    turnToHeading(5);
    driveStraight(18);
    vexDelay(100);
    driveStraight(-10);
    turnToHeading(23);
    driveStraight(16);
    vexDelay(100);

    // Back out from the middle and drop the mobile goal
    turnToHeading(-20);
    driveStraight(-37.5);
    mogo.set(false);
    spinning = false;
    // Turn towards the last two stack in front of the alliance stake and store the top ring
    turnToHeading(90);
    redStoring = true;

    driveStraight(37, 1, 0.5);
    vexDelay(100);

    // Score the ring onto the alliance stake
    turnToHeading(65);
    lB.setBrake(coast);
    lM.setBrake(coast);
    lF.setBrake(coast);

    rB.setBrake(coast);
    rM.setBrake(coast);
    rF.setBrake(coast);
    driveStraight(-17);
    driveStraight(2);
    turnToHeading(0);
    redStoring = false;
    driveStraight(-11);
    driveStraight(1.5);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(700);
    setDrive(12000, 12000);
    vexDelay(500);
    setDrive(0, 0);
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
    vexDelay(700);
    // setDrive(12000, 12000);
    // vexDelay(500);
    // setDrive(0, 0);
}

void blueSideRush() {
    imu.setRotation(99, rotationUnits::deg);
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
    vexDelay(700);
    setDrive(12000, 12000);
    vexDelay(100);
    setDrive(0, 0);
}

void skills() {
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);
    blueSorting = false;
    imu.setRotation(-90, rotationUnits::deg);
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
    Curve c2 = Curve(Vector2d(-29.553, -48.148), Vector2d(-42.115, -52.352), Vector2d(-49.804, -53.767), Vector2d(-62.236, -52.691));

    follow(c2, 4000, 0.5, 12);
    vexDelay(800);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    spinning = false;
    turnToHeading(225, 1, 1, 0, 360, 33, 2600);
    driveStraight(-5);
    turnToPoint(-54.5, -70, 360, 33, 2600);
    // task tt(skillsStore1);
    redStoring = true;
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    vexDelay(200);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(14);
    turnToPoint(0, -15);
    mogo.set(false);
    driveStraight(-9.5);
    vexDelay(50);
    driveToPoint(-57, -5.8, 7000, 0, 2300, 230, 60000);

    redStoring = false;
    turnToHeading(90, 1, 1, 0, 360, 33, 2600);
    driveStraight(-19.3, 1, 0.9);
    conveyor.spin(fwd, -4000, voltageUnits::mV);
    driveStraight(1.4);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    x = -60;
    y = 0;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(700);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    driveStraight(17.2);
    turnToHeading(180, 1, 1, 0, 360, 33, 2600);
    driveStraight(-19, 1, 0.6);
    mogo.set(true);
    vexDelay(100);
    turnToHeading(92);
    Curve c3 = Curve(Vector2d(-32.969, 16.052), Vector2d(-23.933, 24.443), Vector2d(-13.576, 35.551), Vector2d(-2.091, 52.261));
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    follow(c3, 4000, 0.6, 12);

    vexDelay(700);
    turnToHeading(-106, 1, 1, 0, 360, 33, 2600);
    Curve c4 = Curve(Vector2d(-28.451, 51.845), Vector2d(-37.487, 47.437), Vector2d(-45.641, 47.658), Vector2d(-54, 48.404));

    follow(c4, 3000, 0.5, 12);

    vexDelay(300);
    turnToHeading(-60, 1, 1, 0, 360, 33, 2600);
    driveStraight(-8);
    turnToHeading(0, 1, 1, 0, 360, 33, 2600);
    driveStraight(17);
    vexDelay(500);
    turnToHeading(130, 1, 1, 0, 360, 33, 2600);
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    mogo.set(false);
    spinning = false;
    driveStraight(-11);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    Curve c5 = Curve(Vector2d(-48.027, 60.11), Vector2d(-32.423, 60.99), Vector2d(-7.311, 39.893), Vector2d(36.531, 23.981));
    task ttt(skillsStore2);

    follow(c5, 5000, 0.6, 15);
    turnToHeading(0, 1, 1, 0, 360, 33, 2600);
    driveStraight(-26, 1, 0.7);
    driveToPoint(48, 1.4, 8000);
    turnToHeading(-90, 1, 1, 0, 360, 33, 2600);
    driveStraight(-18.7, 1, 0.8);
    x = 60;
    y = 0;
    vexDelay(200);
    redStoring = false;
    blueSorting = true;
    conveyor.spin(fwd, -4000, voltageUnits::mV);
    driveStraight(1.1);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(700);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    driveStraight(4);
    turnToHeading(-175, 1, 1, 0, 360, 33, 2600);
    driveStraight(-55.1, 1, 1);
    
    vexDelay(200);
    driveToPoint(45.5, 4, 8000);
    turnToHeading(0, 1, 1, 0, 360, 33, 2600);
    driveStraight(-16, 1, 0.5);
    mogo.set(true);
    vexDelay(100);
    turnToPoint(24, -24);
    Curve c6 = Curve(Vector2d(16.095, -23.516), Vector2d(19.508, -33.756), Vector2d(21.215, -38.876), Vector2d(22.653, -40.166));
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    follow(c6, 5000, 0.6, 12);
    vexDelay(200);
    turnToHeading(-264, 1, 1, 0, 360, 33, 2600);
    driveStraight(16.75, 1, 0.6);
    vexDelay(200);
    driveStraight(-9);
    turnToHeading(-235, 1, 1, 0, 360, 33, 2600);
    spinning = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    task tttt(skillsStore3);
    driveStraight(17.5);
    vexDelay(100);
    turnToHeading(-432, 1, 1, 0, 360, 33, 2600);
    mogo.set(false);
    driveStraight(-14.5, 1, 0.7);
    armkP = 500;
    armkD = 700;
    targetLadybrownDeg = 24.5;
    redStoring = false;
    driveToPoint(0.5, -50, 7000);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    turnToPoint(1.5, -70);
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    armkP = 500;
    armkD = 500;
    targetLadybrownDeg = 132;
    vexDelay(500);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveStraight(18.5, 1, 0.6);
    driveStraight(-20);
    vexDelay(1000000);
}

int tmogo() {
    vexDelay(820);
    mogo.set(true);
    return 0;
}

int ttmogo() {
    vexDelay(400);
    mogo.set(false);
    return 0;
}

int storettt() {
    vexDelay(200);
    redStoring = true;
    return 0;
}

void redRegionalAWP() {
    blueSorting = true;
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);
    imu.setRotation(-90, rotationUnits::deg);
    x = -52.396;
    y = 22.866;
    Curve mogoClamp(Vector2d(-43.795, 21.866), Vector2d(-32.776, 21.866), Vector2d(-39.14, 21.866), Vector2d(-32.291, 21.866));
    Curve firstStack(Vector2d(-25.493, 32.528), Vector2d(-25.493, 42.005), Vector2d(-28.993, 33.751), Vector2d(-28.993, 36.726));
    Curve middleFirstStack(Vector2d(-14.016, 49.702), Vector2d(-5.953, 49.079), Vector2d(-16.696, 48.5), Vector2d(-14.011, 48.5));
    Curve backUpFromFirstStack(Vector2d(-13.014, 38.87), Vector2d(-14.819, 38.87), Vector2d(-15.911, 37.87), Vector2d(-25.276, 37.87));
    Curve middleSecondStack(Vector2d(-18.025, 40.267), Vector2d(-19.038, 41.484), Vector2d(-18.103, 36.7), Vector2d(-14.206, 36.7));
    Curve backUpFromMiddle(Vector2d(-11.661, 34.038), Vector2d(-19.222, 23.586), Vector2d(-31, 20.358), Vector2d(-49.02, 20.358));
    Curve intakeAllianceStack(Vector2d(-50.02, 4.683), Vector2d(-50.465, -2.211), Vector2d(-50.465, 3.126), Vector2d(-50.465, -17.994));
    Curve kickAllianceRing(Vector2d(-66.808, -12), Vector2d(-66.808, -10), Vector2d(-66.808, -9), Vector2d(-66.808, -8.467));
    Curve scoreAllianceRings(Vector2d(-61.141, 11.599), Vector2d(-61.44, 8.015), Vector2d(-65.524, -4), Vector2d(-61.067, -4));

    task mmm(tmogo);
    follow(mogoClamp, 4000, 0.5, 12, 1);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(200);
    follow(firstStack, 4000, 0.7);
    follow(middleFirstStack, 4000, 0.6, 5);
    vexDelay(200);
    follow(backUpFromFirstStack, 3000, 0.6, 14, 1);
    follow(middleSecondStack, 3000, 0.6, 12);
    vexDelay(500);
    follow(backUpFromMiddle, 5000, 0.7, 25, 1);
    task mmmm(ttmogo);
    blueSorting = false;
    spinning = false;
    redStoring = true;
    follow(intakeAllianceStack, 5000, 0.5, 12);
    vexDelay(300);
    follow(kickAllianceRing, 4000, 0.7, 20, 1);
    driveStraight(3);
    turnToHeading(90);
    redStoring = false;
    setDrive(-7000, -7000);
    vexDelay(850);
    setDrive(0, 0);
    conveyor.spin(fwd, -4000, voltageUnits::mV);
    driveStraight(1.4);
    lB.setBrake(coast);
    lM.setBrake(coast);
    lF.setBrake(coast);

    rB.setBrake(coast);
    rM.setBrake(coast);
    rF.setBrake(coast);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(800);
    driveStraight(6);
    turnToHeading(-180);
    // setDrive(12000, 12000);
    // vexDelay(400);
    // setDrive(0, 0);
    // blueSorting = true;
    // vexDelay(100000);
}

void blueReginalAWP() {
    redSorting = true;
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);
    imu.setRotation(90, rotationUnits::deg);
    x = 52.396;
    y = 22.866;
    Curve mogoClamp(Vector2d(43.795, 21.866), Vector2d(32.776, 21.866), Vector2d(39.14, 21.866), Vector2d(32.291, 21.866));
    Curve firstStack(Vector2d(25.493, 32.528), Vector2d(25.493, 42.005), Vector2d(28.993, 33.751), Vector2d(28.993, 37.926));
    Curve middleFirstStack(Vector2d(14, 50.202), Vector2d(5.953, 49.579), Vector2d(16.696, 50), Vector2d(15.011, 50));
    Curve backUpFromFirstStack(Vector2d(13.014, 39.87), Vector2d(14.819, 39.87), Vector2d(15.911, 41.87), Vector2d(25.276, 41.87));
    Curve middleSecondStack(Vector2d(18.025, 41.267), Vector2d(19.038, 42.484), Vector2d(18.103, 38.7), Vector2d(13.706, 38.7));
    Curve backUpFromMiddle(Vector2d(11.661, 34.038), Vector2d(19.222, 23.586), Vector2d(31, 20.358), Vector2d(49.02, 20.358));
    Curve intakeAllianceStack(Vector2d(50.02, 4.683), Vector2d(50.465, 2.211), Vector2d(50.465, 3.126), Vector2d(50.465, -17.994));
    Curve kickAllianceRing(Vector2d(65.808, -12), Vector2d(65.808, -10), Vector2d(65.808, -10), Vector2d(65.808, -6.467));
    Curve scoreAllianceRings(Vector2d(61.141, 11.599), Vector2d(61.44, 8.015), Vector2d(65.524, -4), Vector2d(61.067, -4));

    task mmm(tmogo);
    follow(mogoClamp, 4000, 0.5, 12, 1);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(200);
    follow(firstStack, 4000, 0.7);
    follow(middleFirstStack, 4000, 0.6, 8);
    vexDelay(200);
    follow(backUpFromFirstStack, 3000, 0.6, 14, 1);
    follow(middleSecondStack, 3000, 0.6, 12);
    vexDelay(500);
    follow(backUpFromMiddle, 5000, 0.7, 25, 1);
    task mmmm(ttmogo);
    redSorting = false;
    spinning = false;
    blueStoring = true;
    follow(intakeAllianceStack, 5000, 0.5, 12);
    vexDelay(300);
    follow(kickAllianceRing, 4000, 0.7, 20, 1);
    driveStraight(1.7);
    turnToHeading(-90);
    blueStoring = false;
    setDrive(-7000, -7000);
    vexDelay(850);
    setDrive(0, 0);
    conveyor.spin(fwd, -4000, voltageUnits::mV);
    driveStraight(1.6);
    lB.setBrake(coast);
    lM.setBrake(coast);
    lF.setBrake(coast);

    rB.setBrake(coast);
    rM.setBrake(coast);
    rF.setBrake(coast);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(800);
    driveStraight(6);
    turnToHeading(180);
    // setDrive(11000, 11000);
    // vexDelay(400);
    // setDrive(0, 0);
    // redSorting = true;
    // vexDelay(100000);
}

int snowDaySkillsClamp1() {
    vexDelay(700);
    mogo.set(true);
    return 0;
}

int snowDaySkillsWall1() {
    vexDelay(600);
    armkP = 500;
    armkD = 700;
    targetLadybrownDeg = 25.5;
    return 0;
}

void snowDaySkills() {
    lF.setBrake(hold);
    lM.setBrake(hold);
    lB.setBrake(hold);

    rF.setBrake(hold);
    rM.setBrake(hold);
    rB.setBrake(hold);
    blueSorting = false;
    redSorting = false;
    imu.setRotation(90, rotationUnits::deg);
    x = -62;
    y = 0;
    driveStraight(1);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(800);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    driveStraightGyro(15.9);
    turnToHeading(180, 1, 1, 0, 360, 36, 2300);
    task t(snowDaySkillsClamp1);
    driveStraightGyro(-22, 0.5);
    vexDelay(100);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    Curve c1(Vector2d(-30.84, 17.891), Vector2d(0.309, 62.448), Vector2d(17.915, 40.076), Vector2d(25.869, 45.871));
    follow(c1, 5000, 0.8, 24, 0, 800);
    vexDelay(200);
    setDrive(-8000, 8000);
    vexDelay(600);
    task tt(snowDaySkillsWall1);
    Curve c2(Vector2d(11.328, 40.903), Vector2d(3.6, 40.242), Vector2d(4.5, 37.132), Vector2d(3, 60.573));
    spinning = false;
    follow(c2, 3000, 0.6, 10);
    vexDelay(800);
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    armkP = 500;
    armkD = 500;
    targetLadybrownDeg = 140;
    vexDelay(1000);
    targetLadybrownDeg = 0;
    driveStraightGyro(-15);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    Curve c3(Vector2d(-12.441, 48.142), Vector2d(-29.661, 48.142), Vector2d(-36.03, 48.142), Vector2d(-51.081, 48.142));
    spinning = true;
    follow(c3, 5000, 0.5, 25);
    vexDelay(200);
    driveStraightGyro(-15);
    vexDelay(10000000);
}
#endif // !AUTONS_H