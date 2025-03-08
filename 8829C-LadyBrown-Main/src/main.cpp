/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       qianjunye                                                 */
/*    Created:      9/10/2024, 10:52:17 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "api.h"
#include "ButtonClass.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int xplace = 70;
int ColorChosen = 1;
int aselection = -1;

lcdButton redButton(120, 22, 230, 35, "RED", "#252525","#FF2525",2);
lcdButton blueButton(480-120, 22, 230, 35, "BLUE", "#252525", "#2525FF", 2);

lcdButton confirm(300,150,100,60, "CONFIRM","#14c40e", "#0f990b", 4);
lcdButton skill(xplace,90,120,40, "skills", "#252525");

lcdButton redNeg(xplace,135,120,40, "Red Neg", "#FF2525", "#FFFFFF", 2);//, color(255, 33, 33));
lcdButton redRush(xplace,180,120,40, "Red Rush", "#FF2525", "#FFFFFF", 2);//, color(255, 33, 33));
lcdButton blueNeg(1000,135,120,40, "Blue Neg", "#2525FF", "#FFFFFF", 2); //figure out color scheme
lcdButton blueRush(1000,180,120,40, "Blue Rush", "#2525FF", "#FFFFFF", 2);

void find_tracking_center(float turnVoltage, int time) {
  x = 0;
  y = 0;
  imu.setRotation(0, rotationUnits::deg);
  unsigned long n = 0;
  float heading;

  std::cout << std::fixed << "\033[1mCopy this:\033[0m\n\\left[";
  setDrive(8000, -8000);

  std::ostringstream out;
  auto end_time = time;
  int t = 0;
  int i = 0;
  while (t < end_time && i++ < 10000) {
    std::cout << "\\left(" << x << "," << y << "\\right),";
    /*if (i % 250 == 0) {
      std::cout << "\\right]\n\\left[" ;
    } */
    if (i % 50 == 0) {
      std::cout.flush();
    }
    t+=20;
    vexDelay(20);
  }  
  setDrive(0, 0);
  std::cout << "\b\\right]" << std::endl;

  std::cout << "Go to https://www.desmos.com/calculator/rxdoxxil1j to solve for offsets." << std::endl;
}


void drawTonomous(){
  Brain.Screen.clearScreen("black");
  Brain.Screen.setPenColor(white);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawLine(0,60,500,60);
  redButton.draw();
  blueButton.draw();
  confirm.draw();
  skill.draw();
  redNeg.draw();
  redRush.draw();
  blueNeg.draw();
  blueRush.draw();
}



void pre_auton(void) {
  ladyBrownL.resetPosition();
  ladyBrownR.resetPosition();
  optic_1.integrationTime(50);
  optic_2.integrationTime(50);
  calibrate();
  ladyBrownStat = 0;
  task screenTask(screen);
  // task antiJ(antiJam);
  task store(ringStoring);
  task sortingTask(wheelchairTask);
  task ladyB(ladyBrownTask);
  task lad(setLadybrown);
  task mogoTask(autoClamp);
  optic_1.setLight(ledState::on);
  optic_2.setLight(ledState::on);
  optic_1.setLightPower(100);
  optic_2.setLightPower(100);
  targetLadybrownDeg = 0;
  ladyBrownL.setBrake(hold);
  ladyBrownR.setBrake(hold);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // switch (aselection) {
  //   case 0:
  //     redSorting = false;
  //     blueSorting = true;
  //     // skills();
  //     break;
  //   case 1:
  //     redSorting = false;
  //     blueSorting = true;
  //     // redTemporaryPos();
  //     break;
  //   case 2:
  //     redSorting = false;
  //     blueSorting = true;
  //     break;
  //   case 3:
  //     redSorting = true;
  //     blueSorting = false;
  //     // blueTemporaryPos();
  //     break;
  //   case 4:
  //     redSorting = true;
  //     blueSorting = false;
  //     break;
  // }
  // provSkills();
  // provRedNeg();
  // provBlueNeg();
  // provRedPosElim();
  provBluePosElim();
  // blueRingSide();
  // skills();
  // redRingSide();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  autoClampActive = false;
  lB.setBrake(coast);
  lM.setBrake(coast);
  lF.setBrake(coast);

  rB.setBrake(coast);
  rM.setBrake(coast);
  rF.setBrake(coast);
  // redSorting = false;
  // blueSorting = true;

  manual = false;
  targetLadybrownDeg = 0;
  ladyBrownStat = 0;
  blueSorting = false;
  redSorting = false;
  lock = false;
  // if (aselection == 0 || aselection == 1 || aselection == 2) {
  //       blueSorting = true;
  //       redSorting = false;
  //     }
  //     else if (aselection == 3 || aselection == 4) {
  //       redSorting = true;
  //       blueSorting = false;
  //     }
      // if (aselection == 1) {
      //   blueSorting = true;
      //   redSorting = false;
      // }
      // else if (aselection == 3) {
      //   redSorting = true;
      //   blueSorting = false;
      // }
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    splitArcade();
    setIntakeMotors();
    // setLadybrown();
    // if (master.ButtonRight.PRESSED) {
    //   lB.setBrake(brake);
    //   lM.setBrake(brake);
    //   lF.setBrake(brake);

    //   rB.setBrake(brake);
    //   rM.setBrake(brake);
    //   rF.setBrake(brake);
    //   // test();
    //   // blueRingSide();
    //   // redSideRush();
    //   // redRingSide();
    //   // driveStraight(0.999);
    //   // driveStraightGyro(24);
    //   // redRegionalAWP();
    //   // blueReginalAWP();
    //   // skills();
    //   // snowDaySkills();
    //   Curve tests(Vector2d(0, 0), Vector2d(10, 14), Vector2d(10, 14), Vector2d(0, 28));
    //   newFollow(tests, 3000, 1, 12);
    //   // redPositiveSide();

    //   // bluePositiveSide();


    //   // Curve test(Vector2d(23.889, 28), Vector2d(-4, 29), Vector2d(0, 12), Vector2d(0, 0));
    //   // Curve tests(Vector2d(0, 0), Vector2d(0, 12), Vector2d(0, 29), Vector2d(0, 48));
    //   // // newFollow(tests, 5000, 1, 12, 0, 200);
    //   // newFollow(tests, 5000, 1, 12, 1, 200);
    //   // ramsete(tests, 5000, 1);
    //   // find_tracking_center(12000, 3000);
    // }
    if (master.ButtonUp.PRESSED) {
      vexDelay(50);
      driveStraightGyro(-3.5);
      vexDelay(50);
      targetLadybrownDeg = 600;
      conveyor.spinFor(-200, rotationUnits::deg, 600, velocityUnits::rpm, false);
      vexDelay(300);
    }
    if (master.ButtonDown.PRESSED) {
      lB.setBrake(brake);
      lM.setBrake(brake);
      lF.setBrake(brake);

      rB.setBrake(brake);
      rM.setBrake(brake);
      rF.setBrake(brake);
      // turnToHeading(45);
      // driveStraightGyro(24);
      // driveStraight(24);
      provSkills();
      // provBlueNeg();
      // provBluePosElim();
      provBluePosElim();
      // provRedRingRush();
    }
    
    
    if (master.ButtonA.PRESSED) { // Click two buttons to switch mode to prevent accidents
      manual = !manual;
      if (manual) master.rumble("."); // Vibration to indicate mode
      else {
        master.rumble("-");
        ladyBrownStat = 0;
        targetLadybrownDeg = 0;
      } // Vibration to indicate mode
      
    }
    if (manual) {
      ladyBrown.spin(fwd, 12000 * (master.ButtonL1.pressing() - master.ButtonL2.pressing()), voltageUnits::mV);
    }
    
    if (master.ButtonRight.PRESSED) {
      rightDoinker.set(!rightDoinker.value());
    }
    if (master.ButtonLeft.PRESSED) {
      leftDoinker.set(!leftDoinker.value());
    }
    

    if (master.ButtonY.PRESSED) {
      blueSorting = !blueSorting;
      redSorting = !redSorting;
      master.rumble(".");
    }
    
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // bool unconfirmed = true;
  // Brain.Screen.clearScreen("black");
  // confirm.setPenColor("#0f990b");
  // drawTonomous();
  // // Set up callbacks for autonomous and driver control periods.
  // while(unconfirmed){
  //   if(Brain.Screen.pressing()){
  //     if(redButton.pressing()){
  //       ColorChosen = 1;
  //       redNeg.moveTo(xplace,redNeg.yPos);
  //       redRush.moveTo(xplace,redRush.yPos);
  //       blueNeg.moveTo(1000,blueNeg.yPos);
  //       blueRush.moveTo(1000,blueRush.yPos);
  //       drawTonomous();
  //     }
  //     else if (blueButton.pressing()){
  //       ColorChosen = 2;
  //       redNeg.moveTo(1000,redNeg.yPos);
  //       redRush.moveTo(1000,redRush.yPos);
  //       blueNeg.moveTo(xplace,blueNeg.yPos);
  //       blueRush.moveTo(xplace,blueRush.yPos);
  //       drawTonomous();
  //     }
  //     else if (ColorChosen == 1){
  //       if(redNeg.pressing()){
  //         aselection = 1;
  //         drawTonomous();
  //       }
  //       else if (redRush.pressing()){
  //         aselection = 2;
  //         drawTonomous();
  //       }
  //     }
  //     else if (ColorChosen == 2){
  //       if(blueNeg.pressing()){
  //         aselection = 3;
  //         drawTonomous();
  //       }
  //       else if (blueRush.pressing()){
  //         aselection = 4;
  //         drawTonomous();
  //       }
  //     }
  //     if (skill.pressing()){
  //       aselection = 0;
  //       drawTonomous();
  //     }
  //   }


  //   if(aselection >-1){  
  //     switch(aselection){
  //       case 0: Brain.Screen.printAt(200, 200, "Skills");
  //         break;
  //       case 1: Brain.Screen.printAt(200, 200, "Red Neg");
  //         break;
  //       case 2: Brain.Screen.printAt(200, 200, "Red Rush");
  //        break;
  //       case 3: Brain.Screen.printAt(200, 200, "Blue Neg");
  //        break;
  //       case 4: Brain.Screen.printAt(200, 200, "Blue Rush");
  //         break;
  //     }
  //     if(confirm.pressing()){
  //       unconfirmed = false;
  //     }
  //   }
  //   else{
  //     if(confirm.pressing()){
  //       Brain.Screen.printAt(150, 220, "Please select an option");
  //     }
  //   }
  //   vexDelay(50);
  // }
  Brain.Screen.clearScreen();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }


}
