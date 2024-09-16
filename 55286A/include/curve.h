#ifndef CURVE_H
#define CURVE_H

#include "autons.h"
class Curve{
    public:
        Vector2d startPose;
        Vector2d controlPoint1;
        Vector2d controlPoint2;
        Vector2d targetPose;
        // Curve Constructor
        Curve(Vector2d startpose, Vector2d controlpoint1, Vector2d controlpoint2, Vector2d targetpose) {
            startPose = startpose;
            controlPoint1 = controlpoint1;
            controlPoint2 = controlpoint2;
            targetPose = targetpose;
        }
    // Find a point on the curve
    Vector2d curveCalc(double t) {
        Vector2d curve = pow(1-t, 3)*this->startPose + 3*pow(1-t, 2)*t*this->controlPoint1 + 3*(1-t)*pow(t, 2)*this->controlPoint2 + pow(t, 3)*this->targetPose;
        return curve;
    }
    // Finds a point for the robot to follow according to the lookahead distance
    double findCarrotPoint(Curve curve, double lookAhead) {
        Vector2d currentPose = Vector2d(x, y);
        double minDistance = 1000;
        double minimumT = 0;
        bool notFound = 1;
        double t = 1;
        Vector2d destination = curve.curveCalc(1);
        double distanceToDestination = (destination - currentPose).norm();
        if (distanceToDestination <= lookAhead) {
            return 1;
        }
        while (t >= 0) {
            Vector2d pose = curve.curveCalc(t);
            double distance = (pose - currentPose).norm();
            if (distance < minDistance) {
                minDistance = distance;
                minimumT = t;
            }
            if (distance <= lookAhead) {
                return t;
            }
            if (t >= 0) {
                t -= 0.01;
            }
        }
        return minimumT;
    }
};

#endif // !CURVE_H