#undef __ARM_NEON__
#undef __ARM_NEON

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
#include <eigen-3.4.0/Eigen/Dense>
#include <iostream>
using namespace vex;
using namespace std;
using namespace Eigen;

#define RED   vex::color::red;
#define BLUE  vex::color::blue;
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)


