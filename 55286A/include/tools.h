#ifndef UTIL_H
#define UTIL_H

#include "odom.h"

double toRadian(double degree) {
    double output = -(degree * (M_PI / 180)) + M_PI_2;
    return output;
}
double toDeg(double radian) {
    double output = -((radian - M_PI_2) / (M_PI / 180));
    return output;
}
#endif // !UTIL_H