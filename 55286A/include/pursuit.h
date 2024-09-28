#ifndef PURSUIT_H
#define PURSUIT_H

#include "curve.h"

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
void follow(Curve path, double timeOut, double speedRatio = 1, double lookAhead = 12, double reversed = 0) {
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
        double linearVoltage = 11000 * curvScaleFactor;
        double feedAngularForward = 50000;
        
        double angularVoltage = feedAngularForward * curvature;
        // if (reversed) {
        //     setDrive(-right_velocity_scaling(linearVoltage, angularVoltage), -left_velocity_scaling(linearVoltage, angularVoltage));
        // }
        // else {
        //     setDrive(clamp_min_voltage(left_velocity_scaling(linearVoltage, angularVoltage), 0), clamp_min_voltage(right_velocity_scaling(linearVoltage, angularVoltage), 0));
        // }
        setDrive(speedRatio * clamp_min_voltage(left_velocity_scaling(linearVoltage, angularVoltage), 0), speedRatio * clamp_min_voltage(right_velocity_scaling(linearVoltage, angularVoltage), 0));
        cout << "targetX: " << carrotPoint[0] << " targetY: " << carrotPoint[1] << " currentX: " << x << " currentY: " << y << " curvature: " << curvature << " linear: " << linearVoltage << " angular: " << angularVoltage << " angError: " << angularError << " scaleFactor: " << curvScaleFactor << " heading: " << currentHeading << endl;
        cout << " " << endl;
        time++;
        vexDelay(10);   
    }
    setDrive(0, 0);
    
}
#endif // !PURSUIT_H