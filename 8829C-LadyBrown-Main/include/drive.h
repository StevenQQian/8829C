#ifndef DRIVE_H
#define DRIVE_H
#include "pid.h"
#include "tools.h"

void setDrive(double l, double r) {
    lB.spin(fwd, l, voltageUnits::mV);
    lM.spin(fwd, l, voltageUnits::mV);
    lF.spin(fwd, l, voltageUnits::mV);

    rB.spin(fwd, r, voltageUnits::mV);
    rM.spin(fwd, r, voltageUnits::mV);
    rF.spin(fwd, r, voltageUnits::mV);
}

void setDriveVelocity(double l, double r) {
    lB.spin(fwd, l, velocityUnits::rpm);
    lM.spin(fwd, l, velocityUnits::rpm);
    lF.spin(fwd, l, velocityUnits::rpm);

    rB.spin(fwd, r, velocityUnits::rpm);
    rM.spin(fwd, r, velocityUnits::rpm);
    rF.spin(fwd, r, velocityUnits::rpm);
}

void splitArcade() {
    double linearVel = deadband(master.Axis3.position(percentUnits::pct), 5) * 120;
    double angularVel = deadband(master.Axis1.position(percentUnits::pct), 5) * 120;

    // if (fabs(linearVel) + fabs(angularVel) > 12000 && linearVel > 0) {
    //     linearVel = 12000 - fabs(angularVel);
    // }
    // if (fabs(linearVel) + fabs(angularVel) > 12000 && linearVel < 0) {
    //     linearVel = -12000 + fabs(angularVel);
    // }
    // setDrive(linearVel + angularVel, linearVel - angularVel);
    setDrive(left_velocity_scaling(linearVel, angularVel), right_velocity_scaling(linearVel, angularVel));
}
#endif // !DRIVE_H