/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Student                                                   */
/*    Created:      9/25/2025, 5:51:32 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

motor LF (PORT18, ratio18_1, true);
motor LB(PORT17, ratio18_1, true);
motor RF(PORT19, ratio18_1, false);
motor RB(PORT20, ratio18_1, false);
motor IntakeL(PORT11);
motor IntakeR(PORT12);

brain Brain;
controller Controller;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
  void Drive(int Lspeed, int Rspeed, int wt){
    
    LF.spin(fwd, Lspeed, pct);
    LB.spin(fwd, Lspeed, pct);
    RF.spin(fwd, Rspeed, pct);
    RB.spin(fwd, Rspeed, pct);

    wait (wt, msec);


  }


	
	


  double YOFFSET = 20; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
	
	if (curr < 1){
		Brain.Screen.setFillColor(green);
	} else if(curr >= 1 && curr  <= 2.5) {
		Brain.Screen.setFillColor(yellow);
	} else {
		Brain.Screen.setFillColor(red);
		Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	}

	
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
	
	if (temp < 45){
		Brain.Screen.setFillColor(green);
	} else if(temp <= 50 && temp  >= 45){
		// TRUE and TRUE --> True
		// TRUE and FALSE --> False
		// FALSE and FALSE --> False
		Brain.Screen.setFillColor(yellow);
	} else {
		Brain.Screen.setFillColor(red);
		Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
		Brain.Screen.setFillColor(transparent);
	}
}


//Displays information on the brain
void Display()
{
	double LFCurr = LF.current(amp);
	double LFTemp = LF.temperature(celsius);
	double LBCurr = LB.current(amp);
	double LBTemp = LB.temperature(celsius);
	double RFCurr = RF.current(amp);
	double RFTemp = RF.temperature(celsius);
	double RBCurr = RB.current(amp);
	double RBTemp = RB.temperature(celsius);


	if (LF.installed()){
		MotorDisplay(1, LFCurr, LFTemp);
		Brain.Screen.printAt(300, YOFFSET + 1, "LF");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 1, "LF Problem");
	}
	
	
	if (LB.installed()){
		MotorDisplay(31, LBCurr, LBTemp);
		Brain.Screen.printAt(300, YOFFSET + 31, "LB");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 31, "LB Problem");
	}


	if (RF.installed()) {
		MotorDisplay(61, RFCurr, RFTemp);
		Brain.Screen.printAt(300, YOFFSET + 61, "RF");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 61, "RF Problem");
	}
	
	
	if (RB.installed()) {
		MotorDisplay(91, RBCurr, RBTemp);
		Brain.Screen.printAt(300, YOFFSET + 91, "RB");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 91, "RB Problem");
	}

}
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

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
  // User control code here, inside the loop
  while (1) {
Display ();

    int Lspeed = Controller.Axis3.position(pct);
    int Rspeed = Controller.Axis2.position(pct);

    Drive(Lspeed, Rspeed, 10);

	if (Controller.ButtonL1.pressing()){
		IntakeR.spin(forward, 100, pct);
		IntakeL.spin(forward, 100, pct);
	}
	else if (Controller.ButtonL2.pressing()){
		IntakeR.spin(reverse, 100, pct);
		IntakeL.spin(reverse, 100, pct);
	}

	else {
		IntakeL.stop();
		IntakeR.stop();
	}


   wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
  

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
