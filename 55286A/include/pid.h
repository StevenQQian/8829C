#ifndef PID_H
#define PID_H

#include "intake.h"

double lateralIntegral = 0;
double lateralPrevError = 0;

double angularPrevError = 0;
double angularIntegral = 0;

void resetPID() {
    lateralPrevError = 0;
    lateralIntegral = 0;

    angularPrevError = 0;
    angularIntegral = 0;
}

double lateralPID(double error, double kP, double kI, double kD) {
    // Calculate the integral
    if (fabs(error) < 2) {
        lateralIntegral += error;
    }
    else {
        lateralIntegral = 0;
    }

    if ((error / fabs(error)) != (lateralPrevError / fabs(lateralPrevError))) {
        lateralIntegral = 0;
    }


    // Calculate Derivative
    const double derivative = -lM.velocity(dps) / 360 * driveWheelDiameter / 100;
    lateralPrevError = error;

    // Calculate Output
    return (kP * error) + (kI * lateralIntegral) + (kD * derivative);
}

double angularPID(double error, double kP, double kI, double kD) {
    // Calculate Integral

    if (fabs(error) < 5) {
        angularIntegral += error;
    }

    else {
        angularIntegral = 0;
    }

    
    if ((error / fabs(error)) != (angularPrevError / fabs(angularPrevError))) {
        angularIntegral = 0;
    }
    const double derivative = -imu.gyroRate(zaxis,dps) / 100;
    angularPrevError = error;

    return (error * kP) + (angularIntegral * kI) + (derivative * kD);
}


#endif // !PID_H