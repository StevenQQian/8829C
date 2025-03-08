#ifndef LADYBROWN_H
#define LADYBROWN_H
#include "motion.h"


motor ladyBrownL(PORT3, gearSetting::ratio18_1, true);
motor ladyBrownR(PORT7, gearSetting::ratio18_1);

motor_group ladyBrown(ladyBrownL, ladyBrownR);
rotation ladyBrownRotation(PORT20, true);
int ladyBrownStat = 0;
bool armPIDActivation = true;
double targetLadybrownDeg = 0;
double error = 0;
double armkP = 300;
double armkD = 0;
bool manual = false;

int ladyBrownTask() {
    while (1) {
        if (!manual) { // Only activates when the arm is not on manual mode. 
            double ladyBrownReading = (ladyBrownL.position(rotationUnits::deg) + ladyBrownR.position(rotationUnits::deg)) / 2;
            error = targetLadybrownDeg - ladyBrownReading;
            // error = toNegPos180(error);
            double armVel = -ladyBrownL.velocity(velocityUnits::dps) / 100;
            if (fabs(error) > 0.15) {
                double power = error * armkP + armVel * armkD;
                ladyBrown.spin(fwd, power, voltageUnits::mV);
            }
            else {
                ladyBrown.spin(fwd, 0, voltageUnits::mV);
            }
        }
        Brain.Screen.printAt(10, 110, "ladyBrownPos: %f", (ladyBrownL.position(rotationUnits::deg) + ladyBrownR.position(rotationUnits::deg)) / 2);
        vexDelay(10);
    }
    return 0;
}


#endif // !LADYBROWN_H