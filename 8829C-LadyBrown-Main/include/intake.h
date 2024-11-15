#ifndef INTAKE_H
#define INTAKE_H

#include "dtconfig.h"
#include "ladybrown.h"

bool lock = false;
bool ringSorting = true;
bool mogoStat = false;
bool sorting = true;
bool spinning = false;
bool redStoring = false;
bool blueStoring = false;

const double hook_1_detection = 0;
const double hook_1_sortPose = 640;
const double hook_2_detection = 750;
const double hook_2_sortPose = 1425;
const double hook_3_detection = 1600;
const double hook_3_sortPose = 2325;

const double full_rotation = 2457;

double ringQueue[] = {0, 0, 0};

bool blueSorting = false;
bool redSorting = false;
digital_out mogo(Brain.ThreeWirePort.E);
digital_out doinker(Brain.ThreeWirePort.H);

vex::color opponentColor = red;

int setLadybrown() {
    while (1) {
        if (master.ButtonL1.PRESSED) {
            if (ladyBrownStat == 0) {
                armkP = 500;
                armkD = 700;
                targetLadybrownDeg = 24.5;
                ladyBrownStat = 1;
            }
            else if (ladyBrownStat == 1) {
                lock = true;
                conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
                armkP = 500;
                armkD = 500;
                targetLadybrownDeg = 132;
                ladyBrownStat = 2;
                vexDelay(100);
                lock = false;
            }
            else if (ladyBrownStat == 2) {
                armkP = 500;
                armkD = 700;
                targetLadybrownDeg = 0;
                ladyBrownStat = 0;
            }
        }
        if (master.ButtonL2.PRESSED) {
            armkP = 500;
            armkD = 500;
            targetLadybrownDeg = 0;
            ladyBrownStat = 0;
        }
    }
}
void setIntakeMotors() {
    if (!lock) {
        conveyor.spin(fwd, 12000 * (master.ButtonR1.pressing() - master.ButtonR2.pressing()), voltageUnits::mV);
    }
    if (master.ButtonX.PRESSED) {
        mogo.set(!mogo.value());
    }
    if (master.ButtonB.PRESSED) {
        doinker.set(!doinker.value());
    }
    
}

int wheelchairTask() {
    while (1) {
        // Brain.Screen.printAt(10, 85, "intakeDeg: %f", conveyor.position(rotationUnits::deg));
        if (blueSorting) {
            // Detect the color when every hook passes the optical sensor and reset the arm if wrong color detected
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_detection) < 200 && (optic_1.color() == vex::color::blue || optic_2.color() == vex::color::blue)) {
                ringQueue[0] = 1;
                targetLadybrownDeg = 0;
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_detection) < 200 && (optic_1.color() == vex::color::blue || optic_2.color() == vex::color::blue)) {
                ringQueue[1] = 1;
                targetLadybrownDeg = 0;    
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_detection) < 200 && (optic_1.color() == vex::color::blue || optic_2.color() == vex::color::blue)) {
                ringQueue[2] = 1;
                targetLadybrownDeg = 0;
            }
        }
        if (redSorting) {
            // Detect the color when every hook passes the optical sensor and reset the arm if wrong color detected
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_detection) < 200 && (optic_1.color() == vex::color::red || optic_2.color() == vex::color::red)) {
                ringQueue[0] = 1;
                targetLadybrownDeg = 0;
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_detection) < 200 && (optic_1.color() == vex::color::red || optic_2.color() == vex::color::red)) {
                ringQueue[1] = 1;
                targetLadybrownDeg = 0;    
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_detection) < 200 && (optic_1.color() == vex::color::red || optic_2.color() == vex::color::red)) {
                ringQueue[2] = 1;
                targetLadybrownDeg = 0;
            }
        }

            // When every hook gets to the top of the conveyor, sort the ring by back spinning if there is a ring to be sorted
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_sortPose) < 50 && ringQueue[0] == 1) {
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                ringQueue[0] = 0;
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_sortPose) < 50 && ringQueue[1] == 1) {
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                ringQueue[1] = 0;
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_sortPose) < 50 && ringQueue[2] == 1) {
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                ringQueue[2] = 0;
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
            }
        
    vexDelay(10);
    }
    return 0;
}

int antiJam() {
    while (1) {
        if (spinning || master.ButtonR1.pressing() && ladyBrownStat != 1) {
            if (conveyor.efficiency() < 0.5) {
                vexDelay(100);
                if (conveyor.efficiency() < 0.5) {
                    lock = true;
                    conveyor.spin(fwd, -12000, voltageUnits::mV);
                    vexDelay(50);
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                    lock = false;
                }
            }
            
        }
        vexDelay(10);
    }
    return 0;
}

int ringStoring() {
    while (1) {
        if (redStoring) {
            if ((optic_1.isNearObject() || optic_2.isNearObject()) && (optic_1.color() == vex::color::red || optic_2.color() == vex::color::red)) {
                conveyor.spin(fwd, 0, voltageUnits::mV);
            }
        }
        else if (blueStoring) {
            if ((optic_1.isNearObject() || optic_2.isNearObject()) && (optic_1.color() == vex::color::blue || optic_2.color() == vex::color::blue)) {
                conveyor.spin(fwd, 0, voltageUnits::mV);
            }
        }
        vexDelay(10);
    }
    return 0;
}


#endif // !INTAKE_H