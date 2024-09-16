#ifndef ODOM_H
#define ODOM_H
#include "tools.h"
#include "motion.h"

double x = 0;
double y = 0;
double theta = 0;
Vector2d position = Vector2d(0, 0);

int track() {
    double currentVertial = verticalTrackingWheel.position(rotationUnits::rev) * verticalDiameter;
    double currentHorizontal = horizontalTrackingWheel.position(rotationUnits::rev) * horizontalDiameter;
    double prevVertical = currentVertial;
    double prevHorizontal = currentHorizontal;
    double currentHeading = imu.rotation(rotationUnits::deg);
    double prevHeading = currentHeading;
    while (1) {
        currentHeading = imu.rotation(rotationUnits::deg);
        double deltaHeading = currentHeading - prevHeading;
        double avgHeading = (currentHeading + prevHeading) / 2;
        double currentVertial = verticalTrackingWheel.position(rotationUnits::rev) * verticalDiameter;
        double currentHorizontal = horizontalTrackingWheel.position(rotationUnits::rev) * horizontalDiameter;
        double deltaY = currentVertial - prevVertical;
        double deltaX = currentHorizontal - prevHorizontal;
        double localX = 0;
        double localY = 0;
        if (deltaHeading == 0) {
            localX = deltaX;
            localY = deltaY;
        }
        else {
            localX = 2 * sin(deltaHeading * (M_PI / 180) / 2) * (deltaX / (deltaHeading * (M_PI / 180)) + horizontalOffset);
            localY = 2 * sin(deltaHeading * (M_PI / 180) / 2) * (deltaY / (deltaHeading * (M_PI / 180)) + verticalOffset);
        }
        x += localY * sin(avgHeading * (M_PI / 180));
        y += localY * cos(avgHeading * (M_PI / 180));
        x += localX * -cos(avgHeading * (M_PI / 180));
        y += localX * sin(avgHeading * (M_PI / 180));
        position = Vector2d(x, y);
        prevHeading = currentHeading;
        prevHorizontal = currentHorizontal;
        prevVertical = currentVertial;
        vexDelay(10);
    }
    return 0;
}

void calibrate() {
    horizontalTrackingWheel.setPosition(0, rotationUnits::deg);
    verticalTrackingWheel.setPosition(0, rotationUnits::deg);
    imu.calibrate();
    imu.setRotation(0, rotationUnits::deg);
}
#endif // !ODOM_H
