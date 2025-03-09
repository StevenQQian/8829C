#ifndef INTAKE_H
#define INTAKE_H

#include "dtconfig.h"
#include "ladybrown.h"

bool lock = false;
bool mogoStat = false;

bool redSorting = false;
bool blueSorting = false;
// bool sorting = true;
bool spinning = false;
bool redStoring = false;
bool blueStoring = false;
bool autoClampActive = false;

const double hook_1_detection = 0;
const double hook_1_sortPose = 460;
const double hook_2_detection = 660;
const double hook_2_sortPose = 1120;
const double hook_3_detection = 1320;
const double hook_3_sortPose = 1780;

const double full_rotation = 1980;

double ringQueue[] = {0, 0, 0};
bool armSort = false;
double redMin = 10;
double redMax = 17;

double blueMin = 210;
double blueMax = 225;

digital_out mogo(Brain.ThreeWirePort.H);
digital_out leftDoinker(Brain.ThreeWirePort.G);
digital_out rightDoinker(Brain.ThreeWirePort.F);
// digital_out doinker(Brain.ThreeWirePort.H);

vex::color opponentColor = red;

int autoClamp() {
    while (1) {
        if (autoClampActive && !mogo.value()) {
            if (dis.objectDistance(distanceUnits::mm) < 45) {
                mogo.set(true);
                autoClampActive = false;
            }
        }
        vexDelay(5);
    }
    return 0;
}



int setLadybrown() {
    while (1) {
        if (!manual) {
            if (master.ButtonL1.PRESSED) {
                if (ladyBrownStat == 0) {
                    armkP = 80;
                    armkD = 0;
                    targetLadybrownDeg = 89; // 31 for auto wall score
                    ladyBrownStat = 1;
                }
                else if (ladyBrownStat == 1) {
                    lock = true;
                    conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
                    armkP = 50;
                    armkD = 0;
                    targetLadybrownDeg = 580;
                    ladyBrownStat = 2;
                    vexDelay(100);
                }
            }
            if (master.ButtonL2.PRESSED) {
                armkP = 80;
                armkD = 0;
                targetLadybrownDeg = 0;
                ladyBrownStat = 0;
            }
        }
        else if (ladyBrownStat == 2) {
            if (!master.ButtonL1.pressing()) {
                targetLadybrownDeg = 0;
                ladyBrownStat = 0;
            }
            
        }

        if (master.ButtonX.PRESSED) {
            mogo.set(!mogo.value());
            vexDelay(500); // wait for the mobile goal to be fully on the robot
            if (mogo.value()) {
                if (dis.objectDistance(distanceUnits::mm) < 15) { // detects distance to the mobile goal
                    
                }
                else {
                    master.rumble("-");
                }
            }
            
        }
        vexDelay(10);
    }
    return 0;
}

void setIntakeMotors() {
    if (!lock) {
        conveyor.spin(fwd, 12000 * (master.ButtonR1.pressing() - master.ButtonR2.pressing()), voltageUnits::mV);
    }
    // if (master.ButtonB.PRESSED) {
    //     doinker.set(!doinker.value());
    // }
    
}

int wheelchairTask() {
    while (1) {
        Brain.Screen.printAt(10, 85, "intakeDeg: %f", conveyor.position(rotationUnits::deg));
        if (redSorting) {
            // Detect the color when every hook passes the optical sensor and reset the arm if wrong color detected
            double errorToHook1 = fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_detection);
            double errorToHook2 = fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_detection);
            double errorToHook3 = fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_detection);
            double arr[] = {errorToHook1, errorToHook2, errorToHook3};
            int n = sizeof(arr) / sizeof(arr[0]);
            double minimum = *min_element(arr, arr + n);
            if (minimum < 400 && minimum == errorToHook1 && ((optic_1.hue() >= redMin && optic_1.hue() <= redMax) || (optic_2.hue() >= redMin && optic_2.hue() <= redMax))) {
                ringQueue[0] = 1;
                if (ladyBrownStat == 1) {
                    armSort = true;
                    targetLadybrownDeg = 0;
                }
                minimum = 10000;
            }
            else if (minimum < 400 && minimum == errorToHook2 && ((optic_1.hue() >= redMin && optic_1.hue() <= redMax) || (optic_2.hue() >= redMin && optic_2.hue() <= redMax))) {
                ringQueue[1] = 1;
                if (ladyBrownStat == 1) {
                    armSort = true;
                    targetLadybrownDeg = 0;
                }
                minimum = 10000;
            }
            else if (minimum < 400 && minimum == errorToHook3 && ((optic_1.hue() >= redMin && optic_1.hue() <= redMax) || (optic_2.hue() >= redMin && optic_2.hue() <= redMax))) {
                ringQueue[2] = 1;
                if (ladyBrownStat == 1) {
                    armSort = true;
                    targetLadybrownDeg = 0;
                }
                minimum = 10000;
            }
            // When every hook gets to the top of the conveyor, sort the ring by back spinning if there is a ring to be sorted
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_sortPose) < 50 && ringQueue[0] == 1) {
                ringQueue[0] = 0;
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                lock = false;
                ringQueue[0] = 0;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                if (armSort) {
                    armkP = 200;
                    armkD = 0;
                    targetLadybrownDeg = 24.5; // 31 for auto wall score
                    ladyBrownStat = 1;
                    armSort = false;
                }
                
                
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_sortPose) < 50 && ringQueue[1] == 1) {
                ringQueue[1] = 0;
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                if (armSort) {
                    armkP = 200;
                    armkD = 0;
                    targetLadybrownDeg = 24.5; // 31 for auto wall score
                    ladyBrownStat = 1;
                    armSort = false;
                }
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_sortPose) < 50 && ringQueue[2] == 1) {
                ringQueue[2] = 0;
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                if (armSort) {
                    armkP = 200;
                    armkD = 0;
                    targetLadybrownDeg = 24.5; // 31 for auto wall score
                    ladyBrownStat = 1;
                    armSort = false;
                }
            }
        }
        else if (blueSorting) {
            // Detect the color when every hook passes the optical sensor and reset the arm if wrong color detected
            double errorToHook1 = fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_detection);
            double errorToHook2 = fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_detection);
            double errorToHook3 = fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_detection);

            double arr[] = {errorToHook1, errorToHook2, errorToHook3};
            int n = sizeof(arr) / sizeof(arr[0]);
            double minimum = *min_element(arr, arr + n);
            if (minimum < 400 && minimum == errorToHook1 && ((optic_1.hue() >= blueMin && optic_1.hue() <= blueMax) || (optic_2.hue() >= blueMin && optic_2.hue() <= blueMax))) {
                ringQueue[0] = 1;
                if (ladyBrownStat == 1) {
                    armSort = true;
                    targetLadybrownDeg = 0;
                }
                minimum = 10000;
            }
            else if (minimum < 400 && minimum == errorToHook2 && ((optic_1.hue() >= blueMin && optic_1.hue() <= blueMax) || (optic_2.hue() >= blueMin && optic_2.hue() <= blueMax))) {
                ringQueue[1] = 1;
                if (ladyBrownStat == 1) {
                    armSort = true;
                    targetLadybrownDeg = 0;
                }
                minimum = 10000;
            }
            else if (minimum < 400 && minimum == errorToHook3 && ((optic_1.hue() >= blueMin && optic_1.hue() <= blueMax) || (optic_2.hue() >= blueMin && optic_2.hue() <= blueMax))) {
                ringQueue[2] = 1;
                if (ladyBrownStat == 1) {
                    armSort = true;
                    targetLadybrownDeg = 0;
                }
                minimum = 10000;
            }
            // When every hook gets to the top of the conveyor, sort the ring by back spinning if there is a ring to be sorted
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_1_sortPose) < 50 && ringQueue[0] == 1) {
                ringQueue[0] = 0;
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                if (armSort) {
                    armkP = 200;
                    armkD = 0;
                    targetLadybrownDeg = 24.5; // 31 for auto wall score
                    ladyBrownStat = 1;
                    armSort = false;
                }
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_2_sortPose) < 50 && ringQueue[1] == 1) {
                ringQueue[1] = 0;
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                if (armSort) {
                    armkP = 200;
                    armkD = 0;
                    targetLadybrownDeg = 24.5; // 31 for auto wall score
                    ladyBrownStat = 1;
                    armSort = false;
                }
            }
            if (fabs(fmod(conveyor.position(rotationUnits::deg), full_rotation) - hook_3_sortPose) < 50 && ringQueue[2] == 1) {
                ringQueue[2] = 0;
                lock = true;
                conveyor.spin(fwd, 12000, voltageUnits::mV);
                vexDelay(100);
                conveyor.spin(fwd, -12000, voltageUnits::mV);
                vexDelay(150);
                conveyor.spin(fwd, 0, voltageUnits::mV);
                lock = false;
                if (spinning) {
                    conveyor.spin(fwd, 12000, voltageUnits::mV);
                }
                if (armSort) {
                    armkP = 200;
                    armkD = 0;
                    targetLadybrownDeg = 24.5; // 31 for auto wall score
                    ladyBrownStat = 1;
                    armSort = false;
                }
            }
        }
    vexDelay(5);
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
            if (((optic_1.hue() < redMax && optic_1.hue() > redMin) || (optic_2.hue() < redMax && optic_2.hue() > redMin))) {
                conveyor.spin(fwd, 0, voltageUnits::mV);
            }
        }
        else if (blueStoring) {
            if (((optic_1.hue() < blueMax && optic_1.hue() > blueMin) || (optic_2.hue() < blueMax && optic_2.hue() > blueMin))) {
                conveyor.spin(fwd, 0, voltageUnits::mV);
            }
        }
        vexDelay(10);
    }
    return 0;
}


#endif // !INTAKE_H

