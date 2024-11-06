#ifndef TOOLS_H
#define TOOLS_H

#include "vex.h"

/**
 * @brief Convert a degree measure to radians. 
 * @param degree Input degree in order to be converted to radians. 
 */
double toRadian(double degree) {
    double output = degree * (M_PI / 180.0);
    return output;
}
/**
 * @brief Convert a radian measure to radians. 
 * @param radian Input radian in order to be converted to degrees. 
 */
double toDeg(double radian) {
    double output = radian / (M_PI / 180.0);
    return output;
}
double toNegPos90(double heading) {
    double input = heading;
    while (input >= 90 || input < -90) {
        if (input < -90) input += 180;
        if (input >= 90) input -= 180;
    }
    return input;
}
double toNegPos180(double heading) {
    double input = heading;
    while (input >= 180 || input < -180) {
        if (input < -180) input += 360;
        if (input >= 180) input -= 360;
    }
    return input;
}
double toAbsoluteDegree(double heading) {
    double input = heading;
    while (input >= 360 || input < 0) {
        if (input < 0) input += 360;
        if (input >= 360) input -= 360;
    }
    return input;
}
/**
 * Voltage scaling to keep from applying more than 12 volts to either side of the drive.
 * Divides both drive and heading output proportionally to get a similar result to the
 * desired one.
 * 
 * @param drive_output The forward output of the drive.
 * @param heading_output The angular output of the drive.
 * @return The scaled voltage for the left side of the robot.
 */
double left_velocity_scaling(double drive_output, double heading_output){
    double ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/12000.0;
    if (ratio > 1) {
        return (drive_output+heading_output)/ratio;
    }
    return drive_output+heading_output;
}
/**
 * Voltage scaling to keep from applying more than 12 volts to either side of the drive.
 * Divides both drive and heading output proportionally to get a similar result to the
 * desired one.
 * 
 * @param drive_output The forward output of the drive.
 * @param heading_output The angular output of the drive.
 * @return The scaled voltage for the right side of the robot.
 */
double right_velocity_scaling(double drive_output, double heading_output){
    double ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/12000.0;
    if (ratio > 1) {
        return (drive_output-heading_output)/ratio;
    }
    return drive_output-heading_output;
}
/**
 * Settling control for odometry functions.
 * Draws a line perpendicular to the line from the robot to the desired 
 * endpoint, and checks if the robot has crossed that line. Allows for
 * very quick settling, and thereby chaining for fast motion control.
 * 
 * @param targetX The ending X position in inches.
 * @param targetY The ending Y position in inches.
 * @param desired_angle_deg The direction of the line to be drawn.
 * @param currentX The robot's X position in inches.
 * @param currentY The robot's Y position in inches.
 * @return Whether the robot can be considered settled.
 */
bool is_line_settled(float targetX, float targetY, float desired_angle_deg, float currentX, float currentY){
    return( (targetY-currentY) * cos(toRadian(desired_angle_deg)) <= -(targetX-currentX) * sin(toRadian(desired_angle_deg)));
}

double getAbsoluteHeading() {
    return (toAbsoluteDegree(imu.rotation(rotationUnits::deg)));
}

double clamp(double input, double min, double max) {
    if (input < min) return min;
    if (input > max) return max;
    return input;
}
double clamp_min_voltage(double drive_output, double drive_min_voltage){
    if(drive_output < 0 && drive_output > -drive_min_voltage){
        return -drive_min_voltage;
    }
    if(drive_output > 0 && drive_output < drive_min_voltage){
        return drive_min_voltage;
    }
  return drive_output;
}

double deadband(double input, double db) {
    if (fabs(input) < db) {
        return 0;
    }
    return input;
}
#endif // !UTIL_H