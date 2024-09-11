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

void driveStraight(double distance, double speedRatio, double kP = 0, double kI = 0, double kD = 0) {
    resetPID();
    double linearError = distance;

    double leftSum = ((lB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (lM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (lF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));

    double leftCurrentReading = leftSum / 3;
    double leftPrevReading = leftCurrentReading;

    double distaceTraveled = 0;
    while (fabs(linearError) < 0.2 && lM.velocity(rpm) * driveWheelDiameter / 6000 < 0.02) {
        // Get drivetrain distance reading
        leftSum = ((lB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (lM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (lF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));

        leftCurrentReading = leftSum / 3;
        double deltaLeft = leftCurrentReading - leftPrevReading;
        distaceTraveled += deltaLeft;

        // Error calculation
        double error = distance - distaceTraveled;
        double driveOutput = lateralPID(error, kP, kI, kD);
        if (fabs(driveOutput) > 100 && driveOutput > 0) driveOutput = 100;
        else if (fabs(driveOutput) > 100 && driveOutput < 0) driveOutput = -100;
        setDrive(driveOutput, driveOutput);
        vexDelay(10);
    }
    
}



#endif // !MOTION_H