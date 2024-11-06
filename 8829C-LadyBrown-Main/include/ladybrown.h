#ifndef LADYBROWN_H
#define LADYBROWN_H
#include "motion.h"


motor ladyBrownL(PORT17, gearSetting::ratio18_1, true);
motor ladyBrownR(PORT10, gearSetting::ratio18_1);
rotation ladyBrownRotation(PORT20, true);
int ladyBrownStat = 0;
bool armPIDActivation = true;
double targetLadybrownDeg = 0;
double error = 0;
double armkP = 500;
double armkD = 0;

int ladyBrownTask() {
    while (1) {
        error = targetLadybrownDeg - ladyBrownRotation.angle(rotationUnits::deg);
        error = toNegPos180(error);
        double armVel = -ladyBrownL.velocity(velocityUnits::dps) / 100;
        if (fabs(error) > 0.75) {
            ladyBrownL.spin(fwd, error * armkP + armVel * armkD, voltageUnits::mV);
            ladyBrownR.spin(fwd, error * armkP + armVel * armkD, voltageUnits::mV);
        }
        else {
            ladyBrownL.spin(fwd, 0, voltageUnits::mV);
            ladyBrownR.spin(fwd, 0, voltageUnits::mV);
        }
        vexDelay(10);
    }
    return 0;
}


#endif // !LADYBROWN_H