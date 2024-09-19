#ifndef MOTION_H
#define MOTION_H
#include "drive.h"
void turnToHeading(double heading, double speedRatio = 1, double reversed = 0, double kP = 0, double kI = 0, double kD = 0) {
    resetPID();
    double turnError = heading - imu.rotation(rotationUnits::deg) - reversed * 180;
    while (fabs(turnError) > 0.3 || imu.gyroRate(zaxis, velocityUnits::dps) / 100 > 0.3) {
        turnError = heading - imu.rotation(rotationUnits::deg) - reversed * 180;
        double driveOutput = angularPID(turnError, kP, kI, kD);
        if (fabs(driveOutput) > 100 && driveOutput > 0) driveOutput = 100;
        else if (fabs(driveOutput) > 100 && driveOutput < 0) driveOutput = -100;
        driveOutput *= speedRatio;
        cout << "error" << turnError << "drive output" << driveOutput << endl;
        setDrive(driveOutput, -driveOutput);
        vexDelay(10); 
    }
    setDrive(0, 0);
}
void driveStraight(double distance, double speedRatio = 1, double kP = 0, double kI = 0, double kD = 0) {
    resetPID();
    double error = distance;
    double leftSum = ((lB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (lM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (lF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));
    double leftCurrentReading = leftSum / 3;
    double leftPrevReading = leftCurrentReading;
    double distaceTraveled = 0;
    while (fabs(error) > 0.2 || lM.velocity(rpm) * driveWheelDiameter / 6000 > 0.02) {
        // Get drivetrain distance reading
        leftSum = ((lB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (lM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (lF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));
        leftCurrentReading = leftSum / 3;
        double deltaLeft = leftCurrentReading - leftPrevReading;
        distaceTraveled += deltaLeft;
        // Error calculation
        error = distance - distaceTraveled;
        double driveOutput = lateralPID(error, kP, kI, kD);
        if (fabs(driveOutput) > 100 && driveOutput > 0) driveOutput = 100;
        else if (fabs(driveOutput) > 100 && driveOutput < 0) driveOutput = -100;
        cout << "error" << error << "drive output" << driveOutput << endl;
        setDrive(driveOutput, driveOutput);
        vexDelay(10);
    }
    setDrive(0, 0);
}
void curveDrive(double leftDistance, double rightDistance, double speedRatio = 1, double kP = 0, double kI = 0, double kD = 0) {
    resetPID();
    double leftError = leftDistance;
    double rightError = rightDistance;
    double leftSum = ((lB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (lM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (lF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));
    double leftCurrentReading = leftSum / 3;
    double leftPrevReading = leftCurrentReading;
    double leftDistaceTraveled = 0;
    double rightSum = ((rB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (rM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
    (rF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));
    double rightCurrentReading = rightSum / 3;
    double rightPrevReading = rightCurrentReading;
    double rightDistaceTraveled = 0;
    while (fabs(leftError) > 0.2 || fabs(rightError) > 0.2 || lM.velocity(rpm) * driveWheelDiameter / 6000 > 0.02 || rM.velocity(rpm) * driveWheelDiameter / 6000 > 0.02) {
        leftSum = ((lB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (lM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (lF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));
        leftCurrentReading = leftSum / 3;
        rightSum = ((rB.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (rM.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)) + 
        (rF.position(rotationUnits::rev) * (driveWheelDiameter * M_PI) * (dt_rpm / 600)));
        rightCurrentReading = rightSum / 3;
        double deltaLeft = leftCurrentReading - leftPrevReading;
        double deltaRight = rightCurrentReading - rightPrevReading;
        leftDistaceTraveled += deltaLeft;
        rightDistaceTraveled += deltaRight;
        leftError = leftDistance - leftDistaceTraveled;
        rightError = rightDistance - rightDistaceTraveled;
        double leftOutput = lateralPID(leftError, kP, kI, kD);
        double rightOutput = lateralPID2(rightError, kP, kI, kD);
        if (fabs(leftOutput) > 100 && leftOutput > 0) leftOutput = 100;
        else if (fabs(leftOutput) > 100 && leftOutput < 0) leftOutput = -100;
        if (fabs(rightOutput) > 100 && rightOutput > 0) rightOutput = 100;
        else if (fabs(rightOutput) > 100 && rightOutput < 0) rightOutput = -100;
        setDrive(leftOutput, rightOutput);
        leftPrevReading = leftCurrentReading;
        rightPrevReading = rightCurrentReading;
        vexDelay(10);
    }
    setDrive(0, 0);
}

/** 
 * Functions below utilize odometry!!!
 * 
 * */

void driveToPoint(double targetX, double targetY, double maxVel, double minVel, double drivekP, double drivekI, double drivekD, double angularkP, double angularkI, double angularkD) {
    resetPID();
    Vector2d targetPose = Vector2d(targetX, targetY);
    double targetDeg = toDeg(atan2((targetPose - position)[0], (targetPose - position)[1]));
    double angularError = targetDeg - getAbsoluteHeading();
    double driveError = (targetPose - position).norm();
    bool lineSettled = is_line_settled(targetX, targetY, targetDeg, x, y);
    while (!lineSettled) {
        driveError = (targetPose - position).norm();
        angularError = toNegPos180(toDeg(atan2((targetPose - position)[0], (targetPose - position)[1])) - getAbsoluteHeading());
        double driveOutput = lateralPID(driveError, drivekP, drivekI, drivekD);
        double headingScaleFactor = cos(toRadian(angularError));
        driveOutput *= headingScaleFactor;
        angularError = toNegPos90(angularError);
        double turnOutput = angularPID(angularError, angularkP, angularkI, angularkD);
        driveOutput = clamp(driveOutput, -fabs(headingScaleFactor)*maxVel, fabs(headingScaleFactor)*maxVel);
        turnOutput = clamp(turnOutput, -maxVel, maxVel);
        driveOutput = clamp_min_voltage(driveOutput, minVel);
        setDrive(left_velocity_scaling(driveOutput, turnOutput), right_velocity_scaling(driveOutput, turnOutput));
        vexDelay(10);
    }
    setDrive(0, 0);
}
void turnToPoint(double targetX, double targetY, double kP, double kI, double kD) {
    resetPID();
    Vector2d targetPose = Vector2d(targetX, targetY);
    double targetDeg = toNegPos180(toDeg(atan2((targetPose - position)[0], (targetPose - position)[1])));
    double angularError = targetDeg - getAbsoluteHeading();
    while (fabs(angularError) > 0.3 || imu.gyroRate(zaxis, dps) / 100 > 0.3) {
        targetDeg = toNegPos180(toDeg(atan2((targetPose - position)[0], (targetPose - position)[1])));
        angularError = targetDeg - getAbsoluteHeading();
        double driveOutput = angularPID(angularError, kP, kI, kD);
        driveOutput = clamp(driveOutput, -100, 100);
        setDrive(driveOutput, -driveOutput);
        vexDelay(10);
    }
    setDrive(0, 0);
}

#endif // !MOTION_H