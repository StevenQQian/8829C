#ifndef LADYBROWN_H
#define LADYBROWN_H
#include "motion.h"


motor ladyBrownL(PORT17, gearSetting::ratio18_1, true);
motor ladyBrownR(PORT10, gearSetting::ratio18_1);

motor_group ladyBrown(ladyBrownL, ladyBrownR);
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
            double power = error * armkP + armVel * armkD;
            ladyBrown.spin(fwd, power, voltageUnits::mV);
        }
        else {
            ladyBrown.spin(fwd, 0, voltageUnits::mV);
        }
        vexDelay(10);
    }
    return 0;
}


#endif // !LADYBROWN_H