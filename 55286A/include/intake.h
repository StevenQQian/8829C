#ifndef INTAKE_H
#define INTAKE_H

#include "dtconfig.h"

bool lock = false;
bool ringSorting = true;
bool mogoStat = false;
digital_out mogo(Brain.ThreeWirePort.H);
void setIntakeMotors() {
    intake.spin(fwd, 12000 * (master.ButtonR1.pressing() - master.ButtonR2.pressing()), voltageUnits::mV);
    
    if (master.ButtonX.PRESSED) {
        if (!mogoStat) {
            mogo.set(true);
            mogoStat = true;
        }
        else {
            mogo.set(false);
            mogoStat = false;
        }
    }
}




#endif // !INTAKE_H