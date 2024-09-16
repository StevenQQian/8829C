#ifndef INTAKE_H
#define INTAKE_H

#include "dtconfig.h"
bool lock = false;
bool ringSorting = true;


void setIntakeMotors() {
    intake.spin(fwd, 12000 * (master.ButtonR1.pressing() - master.ButtonR2.pressing()), voltageUnits::mV);
}



#endif // !INTAKE_H