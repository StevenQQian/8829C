#ifndef PID_H
#define PID_H
#include "intake.h"
double lateralIntegral = 0;
double lateralPrevError = 0;
double lateralIntegral2 = 0;
double lateralPrevError2 = 0;
double angularPrevError = 0;
double angularIntegral = 0;

double armPrevError = 0;
double armIntegral = 0;

/**
 * @brief Reset all the used PID values, always used before every motion funtion. 
 */
void resetPID() {
    lateralPrevError = 0;
    lateralIntegral = 0;
    lateralIntegral2 = 0;
    lateralPrevError2 = 0;
    angularPrevError = 0;
    angularIntegral = 0;
}
/**
 * @brief Lateral PID loop, takes an error reading from a loop and returns a voltage value. 
 * @param error Calculated error in the motion loop. 
 * @param kP Proportional (kP) constant for the PID loop. 
 * @param kI Integral (kI) constant for the PID loop. 
 * @param kD Derivative (kD) constant for the PID loop. 
 */
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
/**
 * @brief Angular PID loop, takes an angular error reading from a loop and returns a voltage value. 
 * @param error Calculated error in the motion loop. 
 * @param kP Proportional (kP) constant for the PID loop. 
 * @param kI Integral (kI) constant for the PID loop. 
 * @param kD Derivative (kD) constant for the PID loop. 
 */
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

double lateralPID2(double error, double kP, double kI, double kD) {
    // Calculate the integral
    if (fabs(error) < 2) {
        lateralIntegral2 += error;
    }
    else {
        lateralIntegral2 = 0;
    }
    if ((error / fabs(error)) != (lateralPrevError2 / fabs(lateralPrevError2))) {
        lateralIntegral2 = 0;
    }
    // Calculate Derivative
    const double derivative = -rM.velocity(dps) / 360 * driveWheelDiameter / 100;
    lateralPrevError2 = error;
    // Calculate Output
    return (kP * error) + (kI * lateralIntegral2) + (kD * derivative);
}

#endif // !PID_H