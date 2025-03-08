#ifndef PURSUIT_H
#define PURSUIT_H

#include "curve.h"

/**
 * @brief Calculates the curvature from one coordinate to another coordinate on a cartesian plane. 
 * @param currentPose Starting position of the curvature calculation. 
 * @param currentHeading Current absolute heading in degrees (0-360) of the robot. 
 * @param targetPose Ending position of the curvature calculation. 
 */
double curvatureToAPoint(Vector2d currentPose, double currenHeading, Vector2d targetPose) {
    double q = (sin(currenHeading) * (targetPose[0] - currentPose[0]) - cos(currenHeading) * (targetPose[1] - currentPose[1]));
    double side;
    if (q > 0) {side = 1;}
    else {side = -1;}
    double a = -tan(currenHeading);
    double c = tan(currenHeading) * currentPose[0] - currentPose[1];
    double x = fabs(a * targetPose[0] + targetPose[1] + c) / sqrt((a * a) + 1);
    double d = hypot(targetPose[0] - currentPose[0], targetPose[1] - currentPose[1]);
    return side * ((2 * x) / (d * d));
}
/**
 * @brief Follows a Bezier Curve created by four control coordinates on a cartesian plane. 
 * @param path A Bezier Curve created during runtime, constructed by four control coordinates on a cartesian plane. 
 * @param timeOut The MAX time allocated for the robot to run the path. 
 * @param speedRatio Default 1. A fixed coefficient for controlling the output voltage during the path. Ranged from 0-1. 
 * @param lookAhead The distance from the point the robot follows to the actual coordinate of the robot. 
 *                  Decrease for better follow on the path, increase for smoother path. Default 12. 
 * @param reversed Decides if the front is forward direction or the back is the forward direction. 
 */
void follow(Curve path, double timeOut, double speedRatio = 1, double lookAhead = 12, double reversed = 0, double kD = 0) {
    Vector2d currentPose = Vector2d(x, y);
    double currentHeading = theta;
    double curvature;
    // Draw a perpendicular line to the connection of control point 2 and target point as the ending line
    double targetDeg = toDeg(atan2(path.targetPose[0] - path.controlPoint2[0], path.targetPose[1] - path.controlPoint2[1]));
    bool lineSettled = is_line_settled(path.targetPose[0], path.targetPose[1], targetDeg, x, y);
    double time = 0;
    while ((time * 10 < timeOut && !lineSettled) || path.findCarrotPoint(path, lookAhead) != 1) {
        // Check if the robot went through the target line or not
        currentPose = Vector2d(x, y);
        double derivative = -verticalTrackingWheel.velocity(velocityUnits::dps) / 100;
        // Check if the robot is settled
        lineSettled = is_line_settled(path.targetPose[0], path.targetPose[1], targetDeg, x, y);
        currentHeading = theta;
        if (reversed) {
            currentHeading -= 180;
        }
        Vector2d carrotPoint = path.curveCalc(path.findCarrotPoint(path, lookAhead));

        if (carrotPoint == path.targetPose) {
            // Extend the Endline for the Robot to end at the Proper Angle
            Vector2d direction = path.targetPose - path.controlPoint2;
            direction.normalize();
            double d = fabs(lookAhead - (currentPose - path.targetPose).norm());
            carrotPoint = d * direction + path.targetPose;
        }
        double curvatureHeading = M_PI / 2 - toRadian(currentHeading);
        curvature = curvatureToAPoint(currentPose, curvatureHeading, carrotPoint);
        double angularError = toNegPos180(toDeg(atan2(carrotPoint[0] - x, carrotPoint[1] - y)) - getAbsoluteHeading());
        double curvScaleFactor = cos(toRadian(angularError));
        // Velocity modifiers
        double linearVoltage = 22000 * curvScaleFactor + derivative * kD;
        double feedAngularForward = 110000;
        double angularVoltage = feedAngularForward * curvature;
        setDrive(speedRatio * clamp_min_voltage(left_velocity_scaling(linearVoltage, angularVoltage), 0), speedRatio * clamp_min_voltage(right_velocity_scaling(linearVoltage, angularVoltage), 0));
        cout << "targetX: " << carrotPoint[0] << " targetY: " << carrotPoint[1] << " currentX: " << x << " currentY: " << y << " curvature: " << curvature << " linear: " << linearVoltage << " angular: " << angularVoltage << " angError: " << angularError << " scaleFactor: " << curvScaleFactor << " heading: " << currentHeading << endl;
        cout << " " << endl;
        time++;
        vexDelay(10);
    }
    setDrive(0, 0);
}

/**
 * @brief Follows a Bezier Curve created by four control coordinates on a cartesian plane. 
 * @param path A Bezier Curve created during runtime, constructed by four control coordinates on a cartesian plane. 
 * @param timeOut The MAX time allocated for the robot to run the path. 
 * @param speedRatio Default 1. A fixed coefficient for controlling the output voltage during the path. Ranged from 0-1. 
 * @param lookAhead The distance from the point the robot follows to the actual coordinate of the robot. 
 *                  Decrease for better follow on the path, increase for smoother path. Default 12. 
 * @param reversed Decides if the front is forward direction or the back is the forward direction. 
 */
void newFollow(Curve path, double timeOut, double speedRatio = 1, double lookAhead = 12, bool reversed = 0, double kD = 0) {
    Vector2d currentPose = Vector2d(x, y);
    double currentHeading = getAbsoluteHeading();
    double curvature;
    // Draw a perpendicular line to the connection of control point 2 and target point as the ending line
    double targetDeg = toDeg(atan2(path.targetPose[0] - path.controlPoint2[0], path.targetPose[1] - path.controlPoint2[1]));
    bool lineSettled = is_line_settled(path.targetPose[0], path.targetPose[1], targetDeg, x, y);
    double time = 0;
    while ((time * 10 < timeOut && !lineSettled) || path.findCarrotPoint(path, lookAhead) != 1) {
        // Check if the robot went through the target line or not
        currentPose = Vector2d(x, y);
        double derivative = -verticalTrackingWheel.velocity(velocityUnits::dps) / 100;
        // Check if the robot is settled
        lineSettled = is_line_settled(path.targetPose[0], path.targetPose[1], targetDeg, x, y);
        currentHeading = theta;
        Vector2d carrotPoint = path.curveCalc(path.findCarrotPoint(path, lookAhead));

        if (carrotPoint == path.targetPose) {
            // Extend the Endline for the Robot to end at the Proper Angle
            Vector2d direction = path.targetPose - path.controlPoint2;
            direction.normalize();
            double d = fabs(lookAhead - (currentPose - path.targetPose).norm());
            carrotPoint = d * direction + path.targetPose;
        }
        
        double desiredLinearVel = 12000;
        if (reversed) {
            desiredLinearVel = -12000;
            currentHeading = toAbsoluteDegree(currentHeading - 180);
        }
        // double absTargetAngle = toAbsoluteDegree(toDeg(atan2(carrotPoint[0] - currentPose[0], carrotPoint[1] - currentPose[1])));
        double angularError = toNegPos180(toDeg(atan2(carrotPoint[0] - x, carrotPoint[1] - y)) - currentHeading);
        double linearVel = desiredLinearVel * cos(toRadian(angularError)) + kD * derivative;
        linearVel = clamp_min_voltage(linearVel, 5000);
        double turnFactor;
        if (angularError > 90) {
            turnFactor = 2 - sin(toRadian(angularError));
        }
        else if (angularError < -90) {
            turnFactor =  -2 - sin(toRadian(angularError));
        }
        else {
            turnFactor = sin(toRadian(angularError));
        }
        
        double turnVel = trackingWidth * 1.45 * turnFactor / lookAhead * fabs(linearVel);
        setDrive(speedRatio * left_velocity_scaling(linearVel, turnVel), speedRatio * right_velocity_scaling(linearVel, turnVel));
        cout << "targetX: " << carrotPoint[0] << " targetY: " << carrotPoint[1] << " currentX: " << x << " currentY: " << y << " angError: " << angularError << " heading: " << currentHeading << " angular: " << turnVel << " linear: " << linearVel << endl;
        cout << " " << endl;
        time++;
        
        vexDelay(10);
    }
    setDrive(0, 0);
}

void ramsete(Curve path, double timeOut, double speedRatio, double reversed = 0) {
    Vector2d desiredPose = path.curveCalc(path.findCarrotPoint(path, 6));
    Vector2d deltaPose = desiredPose - bot_position;
    double currentTheta = toRadian(90 - toAbsoluteDegree(theta));
    double desiredTheta = M_PI / 2 - (atan2(deltaPose[0], deltaPose[1]));
    MatrixXd e(3, 1);
    MatrixXd calcFactor(3, 3);
    MatrixXd errors(3, 1);
    calcFactor(0, 0) = cos(currentTheta);
    calcFactor(0, 1) = sin(currentTheta);
    calcFactor(0, 2) = 0;
    calcFactor(1, 0) = -sin(currentTheta);
    calcFactor(1, 1) = cos(currentTheta);
    calcFactor(1, 2) = 0;
    calcFactor(2, 0) = 0;
    calcFactor(2, 1) = 0;
    calcFactor(2, 2) = 1;
    errors(0, 0) = deltaPose[0];
    errors(1, 0) = deltaPose[1];
    errors(2, 0) = desiredTheta - currentTheta;
    e = calcFactor * errors;
    double desiredLinearVel = e(0, 0) * 0.01;
    double desiredAngularVel = e(2, 0) * 0.01;
    double dampenConstant = 0.6;
    double b = 1;
    double k = 2 * dampenConstant * sqrt(pow(desiredAngularVel, 2) + b * pow(desiredLinearVel, 2));
    double linearVel = (desiredLinearVel * cos(e(2, 0))) + (k * e(0, 0));
    double angularVel = desiredAngularVel + k * e(2, 0) + ((b * desiredLinearVel * sin(e(2, 0)) * e(1, 0)) / e(2, 0));

    double targetDeg = toDeg(atan2(path.targetPose[0] - path.controlPoint2[0], path.targetPose[1] - path.controlPoint2[1]));
    bool lineSettled = is_line_settled(path.targetPose[0], path.targetPose[1], targetDeg, x, y);
    double time = 0;
    while (!lineSettled && time <= timeOut) {
        lineSettled = is_line_settled(path.targetPose[0], path.targetPose[1], targetDeg, x, y);
        desiredPose = path.curveCalc(path.findCarrotPoint(path, 6));
        if ((desiredPose - path.targetPose).norm() < 2) {
            // Extend the Endline for the Robot to end at the Proper Angle
            Vector2d direction = path.targetPose - path.controlPoint2;
            direction.normalize();
            double d = fabs(6 - (bot_position - path.targetPose).norm());
            desiredPose = d * direction + path.targetPose;
        }
        deltaPose = desiredPose - bot_position;
        double desiredTheta = M_PI / 2 - (atan2(deltaPose[0], deltaPose[1]));
        double currentTheta = toRadian(90 - toAbsoluteDegree(theta));
        
        MatrixXd e(3, 1);
        MatrixXd calcFactor(3, 3);
        MatrixXd errors(3, 1);
        calcFactor(0, 0) = cos(currentTheta);
        calcFactor(0, 1) = sin(currentTheta);
        calcFactor(0, 2) = 0;
        calcFactor(1, 0) = -sin(currentTheta);
        calcFactor(1, 1) = cos(currentTheta);
        calcFactor(1, 2) = 0;
        calcFactor(2, 0) = 0;
        calcFactor(2, 1) = 0;
        calcFactor(2, 2) = 1;
        errors(0, 0) = deltaPose[0];
        errors(1, 0) = deltaPose[1];
        errors(2, 0) = desiredTheta - currentTheta;
        e = calcFactor * errors;
        desiredLinearVel = e(0, 0) * 0.01;
        desiredAngularVel = e(2, 0) * 0.01;
        k = 2 * dampenConstant * sqrt(pow(desiredAngularVel, 2) + b * pow(desiredLinearVel, 2));
        double linearVel = (desiredLinearVel * cos(e(2, 0))) + (k * e(0, 0));
        double angularVel = desiredAngularVel + k * e(2, 0) + ((b * desiredLinearVel * sin(e(2, 0)) * e(1, 0)) / e(2, 0));

        double actualLinearVel = linearVel * 2000000;
        double actualAngularVel = angularVel * 400000;

        setDrive(speedRatio * (actualLinearVel, actualAngularVel), speedRatio * right_velocity_scaling(actualLinearVel, actualAngularVel));
        cout << "CarrotPoint X: " << desiredPose[0] << " CarrotPoint Y: " << desiredPose[1] << endl;
        cout << " " << endl;
        cout << "Left Output: " << left_velocity_scaling(actualLinearVel, actualAngularVel) << " Right Output: " << right_velocity_scaling(actualLinearVel, actualAngularVel) << endl;
        vexDelay(10);
    }
    setDrive(0, 0);
}
#endif // !PURSUIT_H