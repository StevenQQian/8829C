#ifndef AUTONS_H
#define AUTONS_H

#include "tools.h"
#include "motion.h"

void test() {
    // driveStraight(24, 1);
    // curveDrive(40, 24, 1);
    // turnToHeading(90, 1);
    driveToPoint(32, 24);
    turnToPoint(0, 24);
    driveToPoint(0, 0);
    turnToHeading(0);

    // turnToPoint(30, 0);

}

#endif // !AUTONS_H