#ifndef DRIVE_H
#include "pid.h"

void setDrive(double l, double r) {
    lB.spin(fwd, l, velocityUnits::pct);
    lM.spin(fwd, l, velocityUnits::pct);
    lF.spin(fwd, l, velocityUnits::pct);

    rB.spin(fwd, r, velocityUnits::pct);
    rM.spin(fwd, r, velocityUnits::pct);
    rF.spin(fwd, r, velocityUnits::pct);
}

#endif // !DRIVE_H