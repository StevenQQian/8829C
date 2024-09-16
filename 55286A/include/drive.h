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

void splitArcade() {
    double linearVel = master.Axis3.position(percentUnits::pct);
    double angularVel = master.Axis1.position(percentUnits::pct);
    if (fabs(linearVel) + fabs(angularVel) > 12000 && linearVel > 0) {
        linearVel = 100 - fabs(angularVel);
    }
    if (fabs(linearVel) + fabs(angularVel) > 12000 && linearVel < 0) {
        linearVel = -100 + fabs(angularVel);
    }
    setDrive(linearVel + angularVel, linearVel - angularVel);
}
#endif // !DRIVE_H