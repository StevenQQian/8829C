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
lcdButton none(xplace,90,120,40, "None", "#252525");

lcdButton redrun(xplace,135,120,40, "Red Run", "#FF2525", "#FFFFFF", 2);//, color(255, 33, 33));
lcdButton red4(xplace,180,120,40, "Red Tower", "#FF2525", "#FFFFFF", 2);//, color(255, 33, 33));
lcdButton bluerun(1000,135,120,40, "Blue Run", "#2525FF", "#FFFFFF", 2); //figure out color scheme
lcdButton blue4(1000,180,120,40, "Blue Tower", "#2525FF", "#FFFFFF", 2);



void drawTonomous(){
  Brain.Screen.clearScreen("black");
  Brain.Screen.setPenColor(white);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawLine(0,60,500,60);
  redButton.draw();
  blueButton.draw();
  confirm.draw();
  none.draw();
  redrun.draw();
  red4.draw();
  bluerun.draw();
  blue4.draw();
}


void pre_auton(void) {
  calibrate();
  ladyBrownStat = 0;
  task screenTask(screen);
  task antiJ(antiJam);
  task store(ringStoring);
  task sortingTask(wheelchairTask);
  task ladyB(ladyBrownTask);
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
      blueSorting = true;
      skills();
      break;
    case 1:
      blueSorting = true;
      redRingSide();
      break;
    case 2:
      blueSorting = true;
      redSideRush();
      break;
    case 3:
      redSorting = true;
      blueRingSide();
      break;
    case 4:
      redSorting = true;
      blueSideRush();
      break;
  }
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
  task lad(setLadybrown);
  lB.setBrake(coast);
  lM.setBrake(coast);
  lF.setBrake(coast);

  rB.setBrake(coast);
  rM.setBrake(coast);
  rF.setBrake(coast);
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
      driveStraight(1.2);
      // skills();
    }
    
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  bool unconfirmed = true;

  Brain.Screen.clearScreen("black");
  confirm.setPenColor("#0f990b");
  drawTonomous();
  // Set up callbacks for autonomous and driver control periods.
  while(unconfirmed){
    if(Brain.Screen.pressing()){
      if(redButton.pressing()){
        ColorChosen = 1;
        redrun.moveTo(xplace,redrun.yPos);
        red4.moveTo(xplace,red4.yPos);
        bluerun.moveTo(1000,bluerun.yPos);
        blue4.moveTo(1000,blue4.yPos);
        drawTonomous();
      }
      else if (blueButton.pressing()){
        ColorChosen = 2;
        redrun.moveTo(1000,redrun.yPos);
        red4.moveTo(1000,red4.yPos);
        bluerun.moveTo(xplace,bluerun.yPos);
        blue4.moveTo(xplace,blue4.yPos);
        drawTonomous();
      }
      else if (ColorChosen == 1){
        if(redrun.pressing()){
          aselection = 1;
          drawTonomous();
        }
        else if (red4.pressing()){
          aselection = 3;
          drawTonomous();
        }
      }
      else if (ColorChosen == 2){
        if(bluerun.pressing()){
          aselection = 2;
          drawTonomous();
        }
        else if (blue4.pressing()){
          aselection = 4;
          drawTonomous();
        }
      }
      if (none.pressing()){
        aselection = 0;
        drawTonomous();
      }
    }


    if(aselection >-1){  
      switch(aselection){
        case 0: Brain.Screen.printAt(200, 200, "none");
          break;
        case 1: Brain.Screen.printAt(200, 200, "Red Run");
          break;
        case 2: Brain.Screen.printAt(200, 200, "Blue Run");
         break;
        case 3: Brain.Screen.printAt(200, 200, "Red Tower");
         break;
        case 4: Brain.Screen.printAt(200, 200, "Blue Tower");
          break;
      }
      if(confirm.pressing()){
        unconfirmed = false;
      }
    }
    else{
      if(confirm.pressing()){
        Brain.Screen.printAt(150, 220, "Please select an option");
      }
    }
    vexDelay(50);
  }
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
