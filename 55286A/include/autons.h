#ifndef AUTONS_H
#define AUTONS_H

#include "tools.h"
#include "motion.h"
#include "curve.h"
#include "pursuit.h"
#include "intake.h"

void test() {
    // driveStraight(24, 1);
    // curveDrive(40, 24, 1);
    // turnToHeading(90, 1);

    // driveToPoint(32, 24);
    // turnToPoint(0, 24);
    // driveToPoint(0, 0);
    // turnToHeading(0);

    // turnToPoint(30, 0);
    lB.setBrake(hold);
    lM.setBrake(hold);
    lF.setBrake(hold);

    rB.setBrake(hold);
    rM.setBrake(hold);
    rF.setBrake(hold);

    x = -68;
    y = 0;
    imu.setRotation(90, rotationUnits::deg);
    Curve clampTheMogo = Curve(Vector2d(-48.837, 7.295), Vector2d(-54.31, 14.567), Vector2d(-54.31, 9.338), Vector2d(-54.31, 19.212));
    Curve intake_first = Curve(Vector2d(-23.052, 23.052), Vector2d(-23.713, 40.132), Vector2d(-22.832, 35.283), Vector2d(-23.933, 37.895));
    Curve intake_second = Curve(Vector2d(-40.793, 47.184), Vector2d(-49.938, 46.743), Vector2d(-45.972, 47.624), Vector2d(-56.787, 46.963));

    Curve backOutFromTheCorner = Curve(Vector2d(-57, 38.038), Vector2d(-57, 32.528), Vector2d(-57, 36.054), Vector2d(-57, 36.002));
    driveStraight(17);
    turnToPoint(-60, -24);
    follow(clampTheMogo, 10000, 0.5, 12, 1);
    vexDelay(100);
    mogo.set(true);
    vexDelay(100);
    turnToPoint(-23.603, 22.884);
    follow(intake_first, 5000, 0.65, 10);
    follow(intake_second, 5000, 0.65, 8);
    turnToHeading(-45);
    follow(backOutFromTheCorner, 5000, 0.7, 12, 1);
    // turnToHeading(90);
    // follow(testDrive2, 3000, 1, 12, 0.8);
}

#endif // !AUTONS_H