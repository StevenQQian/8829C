#ifndef MOTION_H
#define MOTION_H

#include "drive.h"
#include "odom.h"
#include "tools.h"
/**
 * @brief Turn towards an abosolute heading in degrees on a cartesian plane. 
 * @param heading The desired heading for the robot to turn to. 
 * @param speedRatio Voltage scaling for the output. Ranged 0-1, default 1. 
 * @param reversed Decides if the back of the robot points towards the heading or the front of the robot points towards the heading. Default front. 
 * @param kP A constant kP value fed into the turn PID loop. May be tuned. 
 * @param kI A constant kI value fed into the turn PID loop. May be tuned. 
 * @param kD A constant kD value fed into the turn PID loop. May be tuned. 
 */
void turnToHeading(double heading, double speedRatio = 1, double speedCap = 1, double reversed = 0, double kP = 360, double kI = 25, double kD = 2800) {
    resetPID();
    double time = 0;
    double turnError = heading - imu.rotation(rotationUnits::deg) - reversed * 180;
    while (fabs(turnError) > 0.3 || fabs(imu.gyroRate(zaxis, velocityUnits::dps)) / 100 > 0.3) {
        turnError = heading - imu.rotation(rotationUnits::deg) - reversed * 180;
        double driveOutput = angularPID(turnError, kP, kI, kD);
        if (fabs(driveOutput) > 12000 && driveOutput > 0) driveOutput = 12000;
        else if (fabs(driveOutput) > 12000 && driveOutput < 0) driveOutput = -12000;
        driveOutput *= speedRatio;
        clamp(driveOutput, -12000 * speedCap, 12000 * speedCap);
        cout << "error: " << turnError << " drive output: " << driveOutput << " derivative: " << imu.gyroRate(zaxis, velocityUnits::dps) / 100 << endl;
        setDrive(driveOutput, -driveOutput);
        time += 10;
        vexDelay(10); 
    }
    cout << "settled: " << time << endl;
    setDrive(0, 0);
}
/**
 * @brief Drive straight for a certain distance
 * @param distance The desired distance for the robot to move for in inches. 
 * @param speedRatio Voltage scaling for the output. Ranged 0-1, default 1.  
 * @param kP A constant kP value fed into the lateral PID loop. May be tuned. 
 * @param kI A constant kI value fed into the lateral PID loop. May be tuned. 
 * @param kD A constant kD value fed into the lateral PID loop. May be tuned. 
 */
void driveStraight(double distance, double speedRatio = 1, double speedCap = 1, double kP = 2000, double kI = 230, double kD = 57000) {
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
        if (fabs(driveOutput) > 12000 && driveOutput > 0) driveOutput = 12000;
        else if (fabs(driveOutput) > 12000 && driveOutput < 0) driveOutput = -12000;
        cout << "error: " << error << " drive output: " << driveOutput << endl;
        setDrive(speedCap * driveOutput, speedCap * driveOutput);
        leftPrevReading = leftCurrentReading;
        vexDelay(10);
    }
    setDrive(0, 0);
}
/**
 * @brief Give separate distance for every side of the drivetrain for curving purposes. 
 * @param leftDistance The distance commanded for the left side of the drivetrain. 
 * @param rightDistance The distance commanded for the right side of the drivetrain. 
 * @param speedRatio Voltage scaling for the output. Ranged 0-1, default 1. 
 * @param kP A constant kP value fed into the lateral PID loop. May be tuned. 
 * @param kI A constant kI value fed into the lateral PID loop. May be tuned. 
 * @param kD A constant kD value fed into the lateral PID loop. May be tuned. 
 */
void curveDrive(double leftDistance, double rightDistance, double speedRatio = 1, double kP = 2000, double kI = 230, double kD = 57000) {
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
        if (fabs(leftOutput) > 12000 && leftOutput > 0) leftOutput = 12000;
        else if (fabs(leftOutput) > 12000 && leftOutput < 0) leftOutput = -12000;
        if (fabs(rightOutput) > 12000 && rightOutput > 0) rightOutput = 12000;
        else if (fabs(rightOutput) > 12000 && rightOutput < 0) rightOutput = -12000;
        setDrive(leftOutput, rightOutput);
        leftPrevReading = leftCurrentReading;
        rightPrevReading = rightCurrentReading;
        cout << "leftError: " << leftError << " rightError: " << rightError << " leftOutput: " << leftOutput << " rightOutput: " << rightOutput << endl;
        vexDelay(10);
    }
    setDrive(0, 0);
}

/** 
 * Functions below utilize odometry!!!
 * */
void driveToPoint(double targetX, double targetY, double maxVel = 12000, double minVel = 0, double drivekP = 2300, double drivekI = 230, double drivekD = 90000, double angularkP = 360, double angularkI = 22, double angularkD = 3000) {
    resetPID();
    Vector2d targetPose = Vector2d(targetX, targetY);
    double targetDeg = toDeg(atan2(targetX - x, targetY - y));
    double angularError = targetDeg - getAbsoluteHeading();
    double driveError = (targetPose - bot_position).norm();
    bool lineSettled = is_line_settled(targetX, targetY, targetDeg, x, y);
    while (!lineSettled) {
        driveError = (targetPose - bot_position).norm();
        lineSettled = is_line_settled(targetX, targetY, targetDeg, x, y);
        angularError = toNegPos180(toDeg(atan2(targetX - x, targetY - y)) - getAbsoluteHeading());
        double driveOutput = lateralPID(driveError, drivekP, drivekI, drivekD);
        double headingScaleFactor = cos(toRadian(angularError));
        driveOutput *= headingScaleFactor;
        if (fabs(driveError) < 5) {
            angularError = 0;
        }
        angularError = toNegPos90(angularError);
        double turnOutput = angularPID(angularError, angularkP, angularkI, angularkD);
        driveOutput = clamp(driveOutput, -fabs(headingScaleFactor)*maxVel, fabs(headingScaleFactor)*maxVel);
        turnOutput = clamp(turnOutput, -maxVel, maxVel);
        driveOutput = clamp_min_voltage(driveOutput, minVel);
        cout << "driveError: " << driveError << " angularError: " << angularError << " driveOutput: " << driveOutput << " angularOutput: " << turnOutput << endl;
        cout << " " << endl;
        setDrive(left_velocity_scaling(driveOutput, turnOutput), right_velocity_scaling(driveOutput, turnOutput));
        vexDelay(10);
    }
    setDrive(0, 0);
}
void turnToPoint(double targetX, double targetY, double kP = 360, double kI = 22, double kD = 3000) {
    resetPID();
    Vector2d targetPose = Vector2d(targetX, targetY);
    double targetDeg = toNegPos180(toDeg(atan2((targetPose - bot_position)[0], (targetPose - bot_position)[1])));
    double angularError = toNegPos180(targetDeg - getAbsoluteHeading());
    while (fabs(angularError) > 0.3 || imu.gyroRate(zaxis, dps) / 100 > 0.3) {
        targetDeg = toNegPos180(toDeg(atan2((targetPose - bot_position)[0], (targetPose - bot_position)[1])));
        angularError = toNegPos180(targetDeg - getAbsoluteHeading());
        double driveOutput = angularPID(angularError, kP, kI, kD);
        driveOutput = clamp(driveOutput, -12000, 12000);
        setDrive(driveOutput, -driveOutput);
        cout << "error: " << angularError << endl;
        vexDelay(10);
    }
    setDrive(0, 0);
}

#endif // !MOTION_H