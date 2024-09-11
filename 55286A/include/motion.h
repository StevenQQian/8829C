#ifndef MOTION_H
#define MOTION_H

#include "drive.h"

void turnToHeading(double heading, double speedRatio = 1, double reversed = 0, double kP = 0, double kI = 0, double kD = 0) {
    resetPID();
    double turnError = heading - imu.rotation(rotationUnits::deg) - reversed * 180;
    while (fabs(turnError) > 0.3 && imu.gyroRate(zaxis, velocityUnits::dps) / 100 > 0.3) {
        turnError = heading - imu.rotation(rotationUnits::deg) - reversed * 180;
        double driveOutput = angularPID(turnError, kP, kI, kD);
        if (fabs(driveOutput) > 100 && driveOutput > 0) driveOutput = 100;
        else if (fabs(driveOutput) > 100 && driveOutput < 0) driveOutput = -100;
        driveOutput *= speedRatio;
        setDrive(driveOutput, -driveOutput);
        vexDelay(10); 
    }
    setDrive(0, 0);
}



#endif // !MOTION_H