#ifndef ODOM_H
#define ODOM_H


#include "tools.h"


double x = 0;
double y = 0;
double theta = 0;
Vector2d bot_position = Vector2d(0, 0);

int track() {
    double currentVertial = verticalTrackingWheel.position(rotationUnits::deg) / 360 * verticalDiameter * M_PI;
    double currentHorizontal = horizontalTrackingWheel.position(rotationUnits::deg) / 360 * horizontalDiameter * M_PI;
    
    double prevVertical = currentVertial;
    double prevHorizontal = currentHorizontal;
    double currentHeading = imu.rotation(rotationUnits::deg);
    double prevHeading = currentHeading;
    while (1) {
        currentHeading = imu.rotation(rotationUnits::deg);
        double deltaHeading = currentHeading - prevHeading;
        double avgHeading = (currentHeading + prevHeading) / 2;
        double currentVertial = verticalTrackingWheel.position(rotationUnits::deg) / 360 * verticalDiameter * M_PI;
        double currentHorizontal = horizontalTrackingWheel.position(rotationUnits::deg) / 360 * horizontalDiameter * M_PI;
        double deltaY = currentVertial - prevVertical;
        double deltaX = currentHorizontal - prevHorizontal;
        double localX;
        double localY;
        if (deltaHeading == 0) {
            localX = deltaX;
            localY = deltaY;
        }
        else {
            localX = 2 * sin(toRadian(deltaHeading) / 2) * (deltaX / (toRadian(deltaHeading)) + horizontalOffset);
            localY = 2 * sin(toRadian(deltaHeading) / 2) * (deltaY / (toRadian(deltaHeading)) + verticalOffset);
        }
        double localPolarAngle;
        double localPolarLength;

        if (localX == 0 && localY == 0) {
            localPolarAngle = 0;
            localPolarLength = 0;
        }
        else {
            localPolarAngle = atan2(localY, localX);
            localPolarLength = sqrt(pow(localX, 2) + pow(localY, 2));
        }

        double globalPolarAngle = localPolarAngle - toRadian(prevHeading) - (toRadian(deltaHeading) / 2);

        x += localPolarLength * cos(globalPolarAngle);
        y += localPolarLength * sin(globalPolarAngle);
        
        bot_position = Vector2d(x, y);
        prevHeading = currentHeading;
        prevHorizontal = currentHorizontal;
        prevVertical = currentVertial;
        theta = currentHeading;
        vexDelay(2);
    }
    return 0;
}

void calibrate() {
    horizontalTrackingWheel.setPosition(0, rotationUnits::deg);
    verticalTrackingWheel.setPosition(0, rotationUnits::deg);
    imu.calibrate();
    imu.setRotation(0, rotationUnits::deg);
    x = 0;
    y = 0;
    theta = 0;
    task tracking(track);
}

int screen() {
    while (1) {
        Brain.Screen.printAt(10, 10, "x: %f", x);
        Brain.Screen.printAt(10, 35, "y: %f", y);
        Brain.Screen.printAt(10, 60, "theta: %f", theta);
        vexDelay(100);
    }
    return 0;
}
#endif // !ODOM_H
