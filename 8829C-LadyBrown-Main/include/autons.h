#ifndef AUTONS_H
#define AUTONS_H

#include "tools.h"
#include "motion.h"
#include "curve.h"
#include "pursuit.h"
#include "intake.h"

double loadingPos = 91;
int skillsFirstAlliance() {
    vexDelay(300);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    return 0;
}

int skillsFirstBrown() {
    vexDelay(700);
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    ladyBrownStat = 1;
    spinning = false;
    return 0;
}

int skillsFirstIntake() {
    vexDelay(1200);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(300);
    redStoring = true;
    return 0;
}

int skillsSecondIntake() {
    vexDelay(700);
    redStoring = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    return 0;
}




void provSkills() {
    lB.setBrake(brake);
    lM.setBrake(brake);
    lF.setBrake(brake);
    rB.setBrake(brake);
    rM.setBrake(brake);
    rF.setBrake(brake);
    x = -62.5;
    y = 0;
    imu.setRotation(-90, rotationUnits::deg);
    manual = false;
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    ladyBrownStat = 1;
    task t(skillsFirstAlliance);
    // score the first alliance stake
    driveStraightGyro(-5);
    vexDelay(600);
    targetLadybrownDeg = 600;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    vexDelay(500);
    ladyBrownStat = 0;
    // clamp the first mobile goal and score two rings on it
    autoClampActive = true;
    driveStraightGyro(-4);
    targetLadybrownDeg = 0;
    driveToPoint(-47, 26, 5000);
    Curve c1(Vector2d(-30.986, 27.17), Vector2d(-17.983, 27.068), Vector2d(-9.609, 43), Vector2d(21.228, 46));
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    newFollow(c1, 5000, 0.8, 19);
    vexDelay(200);
    task tt(skillsFirstBrown);
    // store a ring on the arm
    driveStraightGyro(17.5, 0.7, 0.7);
    x = 66 - disFront.objectDistance(distanceUnits::in) * cos(toRadian(90 - imu.rotation()));
    vexDelay(120);
    // score two rings on the first wall stake
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveToPoint(6, 39, 7000);
    conveyor.spin(fwd, 5000, voltageUnits::mV);
    vexDelay(50);
    turnToPoint(-3, 70);
    driveToPoint(0, 70, 8000, 0, 1300);
    setDrive(11000, 11000);
    vexDelay(100);
    setDrive(0, 0);
    vexDelay(200);
    lock = true;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    armkP = 50;
    armkD = 0;
    targetLadybrownDeg = 580;
    vexDelay(400);
    armkP = 80;
    targetLadybrownDeg = loadingPos;
    lock = false;
    ladyBrownStat = 1;
    vexDelay(600);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    setDrive(11000, 11000);
    vexDelay(100);
    setDrive(0, 0);
    vexDelay(700);
    conveyor.spin(fwd, 0, voltageUnits::mV);

    lock = true;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    armkP = 50;
    armkD = 0;
    targetLadybrownDeg = 580;
    vexDelay(400);
    targetLadybrownDeg = 0;
    lock = false;
    ladyBrownStat = 0;
    y = 60 - disFront.objectDistance(distanceUnits::in) * cos(0 - toRadian(imu.rotation()));

    vexDelay(200);

    // back out from the wall stake and go for the first corner
    driveStraightGyro(-15);
    Curve c2(Vector2d(-19.526, 42.523), Vector2d(-28, 42.523), Vector2d(-34, 42.523), Vector2d(-56, 42.523));
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    newFollow(c2, 4500, 0.55, 17);
    vexDelay(100);
    setDrive(0, -12000);
    vexDelay(200);
    setDrive(0, 0);
    driveToPoint(-47, 37);
    driveToPoint(-46, 52);
    vexDelay(300);
    turnToHeading(-250);
    // place the mobile goal in the first corner
    mogo.set(false);
    conveyor.spinFor(-1000, rotationUnits::deg, 600, velocityUnits::rpm, false);
    driveStraightGyro(-12);

    // go towards the blue alliance stake and push a blue ring mobile goal into a corner
    Curve c3(Vector2d(-59.084, 60.186), Vector2d(-51.151, 59.084), Vector2d(5.708, 34.776), Vector2d(22.898, 18.044));
    spinning = false; 
    redStoring = true;
    task tttttt(skillsFirstIntake);
    newFollow(c3, 7000, 0.7, 20);
    driveToPoint(56, 8, 7000);
    turnToHeading(-535 + 360);
    redStoring = false;
    driveStraight(-40, 0.7, 0.7, 1600);
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    ladyBrownStat = 1;
    conveyor.spin(fwd, 12000, voltageUnits::mV);


    // score the alliance stake
    driveToPoint(51, 20, 5000);
    turnToHeading(-360);
    autoClampActive = true;
    driveToPoint(50, -12, 5000);
    driveToPoint(50, -10);
    turnToPoint(70, -6);
    driveToPoint(67, -8.5, 7000, 0, 1300);
    setDrive(11000, 11000);
    vexDelay(50);
    setDrive(0, 0);
    double rX = x;
    double rY = y;
    driveStraightGyro(-6.3);
    targetLadybrownDeg = 600;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    vexDelay(500);
    ladyBrownStat = 0;
    driveStraightGyro(-4);
    targetLadybrownDeg = 0;
    x = 60 - disFront.objectDistance(distanceUnits::in) * cos(toRadian(90 - imu.rotation()));

    // go through the middle ladder towards the third corner
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    driveToPoint(35, 24, 7000);
    conveyor.spin(fwd, 0, voltageUnits::mV);
    spinning = false;
    task ttt(skillsFirstIntake);

    driveToPoint(-3, -7, 8000);

    // score all rings in the corner and place the mobile goal into the corner
    Curve c4(Vector2d(-20.752, -22.275), Vector2d(-24.252, -33.284), Vector2d(-32.762, -49.545), Vector2d(-59.278, -51.545));
    task tttt(skillsSecondIntake);
    newFollow(c4, 6000, 0.65, 17);
    x = -66 + disFront.objectDistance(distanceUnits::in) * cos(toRadian(-90 - imu.rotation()));
    vexDelay(300);
    setDrive(-12000, 0);
    vexDelay(200);
    setDrive(0, 0);
    driveToPoint(-47, -48);
    driveToPoint(-48, -62, 12000, 0, 1500);


    vexDelay(200);
    y = -66 + disFront.objectDistance(distanceUnits::in) * cos(toRadian(180 - imu.rotation()));
    turnToHeading(-290);
    mogo.set(false);
    conveyor.spinFor(-600, rotationUnits::deg, 600, velocityUnits::rpm, false);
    driveStraightGyro(-10.1);
    // intake two rings and go for the second wall stake
    driveToPoint(-50, -46, 8000);
    turnToPoint(-47, -50);
    autoClampActive = true;
    driveToPoint(-50, -25, 7000);
    spinning = false;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    ladyBrownStat = 1;
    driveToPoint(-22.75, -54, 8000);
    driveToPoint(-7.5, -46, 8000);
    turnToPoint(-5, -70);
    vexDelay(50);
    driveToPoint(-5, -70, 8000, 0, 1400);
    setDrive(11000, 11000);
    vexDelay(200);
    setDrive(0, 0);
    vexDelay(200);
    lock = true;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    armkP = 50;
    armkD = 0;
    targetLadybrownDeg = 580;
    vexDelay(400);
    armkP = 80;
    targetLadybrownDeg = loadingPos;
    lock = false;
    ladyBrownStat = 1;
    vexDelay(600);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    setDrive(11000, 11000);
    vexDelay(100);
    setDrive(0, 0);
    vexDelay(700);
    conveyor.spin(fwd, 0, voltageUnits::mV);

    lock = true;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    armkP = 50;
    armkD = 0;
    targetLadybrownDeg = 580;
    vexDelay(400);
    targetLadybrownDeg = 0;
    lock = false;
    ladyBrownStat = 0;
    vexDelay(200);
    y = -60 + disFront.objectDistance(distanceUnits::in) * cos(toRadian(180 - imu.rotation()));

    // last corner
    driveStraightGyro(-15);
    spinning = true;
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    turnToPoint(24, -24);
    driveToPoint(29, -14, 8000);
    vexDelay(200);
    turnToPoint(19.449, -47.131);
    driveToPoint(19.449, -47.131);
    vexDelay(200);
    turnToPoint(50, -47);
    driveToPoint(40, -47);
    driveToPoint(47, -47, 6000);
    vexDelay(200);
    turnToHeading(-39);
    mogo.set(false);
    spinning = false;
    conveyor.spin(fwd, -12000, voltageUnits::mV);
    driveStraight(-13, 0.7, 0.7, 1000);
    driveStraight(14);
    cout << rX << " " << rY << endl;
    vexDelay(10000000);
}

void provRedRingRush() {
    lB.setBrake(brake);
    lM.setBrake(brake);
    lF.setBrake(brake);
    rB.setBrake(brake);
    rM.setBrake(brake);
    rF.setBrake(brake);
    imu.setRotation(-90, rotationUnits::deg);
    x = -48;
    y = -24;
    autoClampActive = true;
    driveStraightGyro(-23, 0.7, 0.7);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(200); 
    turnToPoint(-6, -6.5);
    vexDelay(200);
    conveyor.spinTo(2130, rotationUnits::deg, 600, velocityUnits::rpm, false);
    driveToPoint(-6, -6, 4900);
    // conveyor.spin(fwd, -12000, voltageUnits::mV);
    vexDelay(100);
    turnToHeading(67.5);
    driveStraight(1.5, 0.7, 0.7);
    driveStraightGyro(-35, 0.75, 0.75, 3000);
    turnToHeading(110);

    vexDelay(200);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    driveToPoint(-35, -10);
    turnToPoint(-24, -50);
    driveToPoint(-24, -17);
    vexDelay(100000000);

}

void provRedNeg() {
    imu.setRotation(-90 - 27.9097, rotationUnits::deg);
    x = -52;
    y = 8;
    blueSorting = true;
    manual = false;
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    ladyBrownStat = 1;
    task t(skillsFirstAlliance);
    // score the first alliance stake
    driveStraightGyro(7.5);
    vexDelay(600);
    targetLadybrownDeg = 600;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    vexDelay(600);
    ladyBrownStat = 0;
    autoClampActive = true;
    driveStraightGyro(-38, 0.75, 0.75);
    targetLadybrownDeg = 0;
    vexDelay(100);
    turnToHeading(55);
    vexDelay(50);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    driveToPoint(-11.4, 39, 7000);
    swingToHeading(-2, false);
    driveStraightGyro(5);
    vexDelay(100);
    turnToPoint(-23.602, 47.084);
    driveStraightGyro(32, 0.7, 0.7);
    vexDelay(300);
    // turnToPoint(-62, 66);
    // spinning = false;
    // conveyor.spin(fwd, 0, voltageUnits::mV);
    // targetLadybrownDeg = 600;
    // driveToPoint(-62, 66, 7000, 0, 1800);
    // conveyor.spin(fwd, 12000, voltageUnits::mV);
    // vexDelay(200);
    // spinning = true;
    // driveStraightGyro(-21);
    // vexDelay(100);
    // driveStraightGyro(16);
    // vexDelay(100);
    // driveStraightGyro(-20);
    turnToPoint(0, 0);
    lB.setBrake(coast);
    lM.setBrake(coast);
    lF.setBrake(coast);
    rB.setBrake(coast);
    rM.setBrake(coast);
    rF.setBrake(coast);
    targetLadybrownDeg = 300;
    setDrive(12000, 12000);
    vexDelay(600);
    setDrive(0, 0);
}

void provBlueNeg() {
    imu.setRotation(-(-90 - 27.9097), rotationUnits::deg);
    x = 52;
    y = 8;
    redSorting = true;
    manual = false;
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    ladyBrownStat = 1;
    task t(skillsFirstAlliance);
    // score the first alliance stake
    driveStraightGyro(7.5);
    vexDelay(600);
    targetLadybrownDeg = 600;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    vexDelay(600);
    ladyBrownStat = 0;
    autoClampActive = true;
    driveStraightGyro(-38, 0.75, 0.75);
    targetLadybrownDeg = 0;
    vexDelay(100);
    turnToHeading(-55);
    vexDelay(50);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    driveToPoint(12, 39, 7000);
    swingToHeading(2, false);
    driveStraightGyro(13.5);
    vexDelay(100);
    turnToPoint(23.602, 47.084);
    driveStraightGyro(32, 0.7, 0.7); 
    turnToPoint(62, 66);
    spinning = false;
    conveyor.spin(fwd, 0, voltageUnits::mV);
    targetLadybrownDeg = 600;
    driveToPoint(62, 66, 7000, 0, 1800);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    vexDelay(200);
    spinning = true;
    driveStraightGyro(-21);
    vexDelay(100);
    driveStraightGyro(16);
    vexDelay(100);
    driveStraightGyro(-20);
    turnToPoint(0, 0);
    lB.setBrake(coast);
    lM.setBrake(coast);
    lF.setBrake(coast);
    rB.setBrake(coast);
    rM.setBrake(coast);
    rF.setBrake(coast);
    targetLadybrownDeg = 260;
    setDrive(12000, 12000);
    vexDelay(600);
    setDrive(0, 0);
}

void provBluePosElim() {
    lB.setBrake(brake);
    lM.setBrake(brake);
    lF.setBrake(brake);
    rB.setBrake(brake);
    rM.setBrake(brake);
    rF.setBrake(brake);
    redSorting = true;
    imu.setRotation(imu.rotation(), rotationUnits::deg);
    x = 52;
    y = 8;
    manual = false;
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    task tt(skillsFirstAlliance);
    driveStraightGyro(5.5);
    vexDelay(300);
    targetLadybrownDeg = 600;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    vexDelay(400);
    ladyBrownStat = 0;
    autoClampActive = true;
    driveStraightGyro(-37.5, 0.75, 0.75);
    targetLadybrownDeg = 0;
    vexDelay(150);
    driveStraightGyro(17);
    turnToPoint(-6, 17);
    vexDelay(100);
    driveToPoint(14, 10);
    leftDoinker.set(true);
    vexDelay(400);
    turnToHeading(-70);
    driveStraight(2.5, 1, 1, 3000, 3000, 70, 40000);
    rightDoinker.set(true);
    vexDelay(400);
    driveStraightGyro(-27, 0.7, 0.7);
    turnToHeading(-105);
    leftDoinker.set(false);
    rightDoinker.set(false);
    vexDelay(400);
    turnToHeading(-78);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    driveStraightGyro(13);
    vexDelay(200);
    turnToHeading(-183);
    driveStraightGyro(39, 0.65, 0.65);
    swingToHeading(-266, false);
    rightDoinker.set(true);
    driveStraightGyro(21, 0.85, 0.85);
    turnToHeading(-360);
    mogo.set(false);
    rightDoinker.set(false);
    vexDelay(100000000);
}

void provRedPosElim() {
    lB.setBrake(brake);
    lM.setBrake(brake);
    lF.setBrake(brake);
    rB.setBrake(brake);
    rM.setBrake(brake);
    rF.setBrake(brake);
    blueSorting = true;
    imu.setRotation(imu.rotation(), rotationUnits::deg);
    x = -52;
    y = 8;
    manual = false;
    armkP = 80;
    armkD = 0;
    targetLadybrownDeg = loadingPos;
    task tt(skillsFirstAlliance);
    driveStraightGyro(5.5);
    vexDelay(300);
    targetLadybrownDeg = 600;
    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
    vexDelay(400);
    ladyBrownStat = 0;
    autoClampActive = true;
    driveStraightGyro(-37.5, 0.75, 0.75);
    targetLadybrownDeg = 0;
    vexDelay(150);
    driveStraightGyro(19);
    turnToPoint(6, 17);
    vexDelay(100);
    driveToPoint(-13, 11);
    rightDoinker.set(true);
    vexDelay(400);
    turnToHeading(70);
    driveStraight(2.5, 1, 1, 3000, 3000, 70, 40000);
    leftDoinker.set(true);
    vexDelay(400);
    driveStraightGyro(-27, 0.7, 0.7);
    turnToHeading(115);
    leftDoinker.set(false);
    rightDoinker.set(false);
    vexDelay(400);
    turnToHeading(78);
    conveyor.spin(fwd, 12000, voltageUnits::mV);
    spinning = true;
    driveStraightGyro(13);
    vexDelay(200);
    turnToHeading(183);
    driveStraightGyro(39, 0.65, 0.65);
    swingToHeading(266);
    leftDoinker.set(true);
    driveStraightGyro(21, 0.85, 0.85);
    turnToHeading(360);
    mogo.set(false);
    leftDoinker.set(false);
    vexDelay(100000000);
}

#endif // !AUTONS_H