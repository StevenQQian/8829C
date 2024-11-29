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
  optic_1.integrationTime(50);
  optic_2.integrationTime(50);
  calibrate();
  ladyBrownStat = 0;
  task screenTask(screen);
  task antiJ(antiJam);
  task store(ringStoring);
  task sortingTask(wheelchairTask);
  task ladyB(ladyBrownTask);
  task lad(setLadybrown);
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

  switch (aselection) {
    case 0:
      redSorting = false;
      blueSorting = true;
      skills();
      break;
    case 1:
      redSorting = false;
      blueSorting = true;
      redRegionalAWP();
      break;
    case 2:
      redSorting = false;
      blueSorting = true;
      redSideRush();
      break;
    case 3:
      redSorting = true;
      blueSorting = false;
      blueReginalAWP();
      break;
    case 4:
      redSorting = true;
      blueSorting = false;
      blueSideRush();
      break;
  }

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
  
  lB.setBrake(coast);
  lM.setBrake(coast);
  lF.setBrake(coast);

  rB.setBrake(coast);
  rM.setBrake(coast);
  rF.setBrake(coast);
  // redSorting = true;
  redSorting = true;
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
    if (master.ButtonUp.PRESSED) {
      // test();
      // blueRingSide();
      // redSideRush();
      // redRingSide();
      // driveStraight(0.9);
      // driveStraightGyro(24);
      // redRegionalAWP();
      // blueReginalAWP();
      // skills();
      snowDaySkills();
    }

    if (master.ButtonY.PRESSED) {
      redSorting = !redSorting;
      blueSorting = !blueSorting;
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
