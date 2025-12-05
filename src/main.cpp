

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

motor LF (PORT10, ratio6_1, true);
motor LB(PORT18, ratio6_1, true);
motor RF(PORT14, ratio6_1, false);
motor RB(PORT20, ratio6_1, false);

motor Intake(PORT13, ratio18_1, false);
motor Conveyor(PORT16, ratio6_1, true);
motor Outtake (PORT12, ratio6_1, true);

inertial Gyro (PORT6);

digital_out PneuSCRAPER = digital_out(Brain.ThreeWirePort.A);
digital_out PneuDESCORE = digital_out(Brain.ThreeWirePort.B);

controller Controller;

/*---------------------------------------------------------------------------*\
|*                          Pre-Autonomous Functions                         *|
\*---------------------------------------------------------------------------*/
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
	double intakeTemp = Intake.temperature(celsius);
	double intakeCurr = Intake.current(amp);
	double ConveyorTemp = Conveyor.temperature(celsius);
	double ConveyorCurr = Conveyor.current(amp);
	double OuttakeTemp= Outtake.temperature(celsius);
	double OuttakeCurr = Outtake.current(amp);


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
	if (Intake.installed()) {
		MotorDisplay(121, intakeCurr, intakeTemp);
		Brain.Screen.printAt(300, YOFFSET + 121, "Intake");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 121, "Intake Problem");
	}
	if (Conveyor.installed()) {
		MotorDisplay(151, ConveyorCurr, ConveyorTemp);
		Brain.Screen.printAt(300, YOFFSET + 151, "Conveyor");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 151, "Conveyor Problem");
	}
	if (Outtake.installed()) {
		MotorDisplay(181, OuttakeCurr, OuttakeTemp);
		Brain.Screen.printAt(300, YOFFSET + 181, "Outtake");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 181, "Outtake Problem");
	}

}


void gyroturn(float target)
{
		float heading=0.0; //initialize a variable for heading
		float accuracy=8.0; //how accurate to make the turn in degrees
		float error=target-heading;
		float kp= 0.3;
		float speed=kp*error;
		Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
		
		while(fabs(error)>=accuracy)
		{
			speed=kp*error;
			Drive(speed, -speed, 10); //turn right at Speed
			heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;  //calculate error
		}
			Brain.Screen.printAt(10, 20, "Gyro Reading= %.2f", heading); 
			Drive(0, 0, 0);  //stope the drive
}


void inchdrive (float inches){
	float x = 0;
	float error = inches - x;
	float kp = 2.0;
	float speed = error * kp;
	float accuracy = 0.5; 
	LF.resetPosition(); 
	x = LF.position(rev)*3.25*0.75*M_PI; 

	while ( fabs(error)>= accuracy) { 
		Drive(speed, speed, 10); 
		x = LF.position(rev)*3.25*0.75*M_PI; 
		error = inches - x;
		speed = error * kp;

	}
	Drive(0,0,0); 

}

void intake (){
	Conveyor.spin(fwd, 100, pct);
	Outtake.spin(reverse, 0, pct);
	Intake.spin(fwd, 100, pct);
}

void outtake (){
	Conveyor.spin(reverse, 100, pct);
	Outtake.spin(reverse, 100, pct);
	Intake.spin(reverse, 100, pct);
}

void score (){
	Conveyor.spin(fwd, 100, pct);
	Outtake.spin(fwd, 100, pct);
	Intake.spin(fwd, 100, pct);
}

void stopsub3 (){
	Conveyor.stop();
	Outtake.stop();
	Intake.stop();
}
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
	while (Gyro.isCalibrating()){ 
		wait(100, msec); 
	}
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
//left side
	// intake();
	// inchdrive(27);
	// wait(500, msec);
	// inchdrive(-15);
	// gyroturn(-70);
	// inchdrive (29);
	// gyroturn (-90);
	// inchdrive (-16);
	// score();

//right side
	// intake();
	// inchdrive(27);
	// wait(500, msec);
	// inchdrive(-15);
	// gyroturn(70);
	// inchdrive (29);
	// gyroturn (90);
	// inchdrive (-16);
	// score();

//skills
inchdrive(36);
gyroturn(-90);
inchdrive(-4);
PneuSCRAPER.set(true);
intake();
Drive(40, 40, 650); 

// Drive(-40, -40, 500);   was too odd:%
// Drive(40, 40, 600); 
// Drive(-40, -40, 500);
// Drive(40, 40, 600); 
// Drive(-40, -40, 500);
// Drive(40, 40, 600); 
// Drive(-40, -40, 500);
// Drive(40, 40, 600); 

Drive(0, 0, 0); // we should have gotten all the blocks
Drive(40, 40, 400 ); 
Drive(0,0,0); 
wait(4500, msec);
// inchdrive(-2);
// inchdrive(2);
// inchdrive(-2);
// inchdrive(2);
// inchdrive(-2);
// inchdrive(2);
// gyroturn(45);
// inchdrive(3);
// gyroturn(-45);
inchdrive(-24.5);
score();

}


/*---------------------------------------------------------------------------*\
|*                                                                           *|
|*                              User Control Task                            *|
|*                                                                           *|
|*  This task is used to control your robot during the user control phase of *|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|*  a VEX Competition.                                                       *|
|*                                                                           *|
|*  You must modify the code to add your own robot specific commands here.   *|
\*---------------------------------------------------------------------------*/

void usercontrol(void) {
  
PneuDESCORE.set(true);

  while (1) {
//Motor Monitor

Display ();


//drivecode


    int Lspeed = Controller.Axis3.position(pct);
    int Rspeed = Controller.Axis2.position(pct);

    Drive(Lspeed, Rspeed, 10);

	

//Scoring and intake

if (Controller.ButtonR1.pressing()){  //Scoring (all motors spinning fwd)
		Outtake.spin(fwd, 100, pct); 
		Conveyor.spin(fwd, 90, pct);
		Intake.spin(fwd, 100, pct);
		}


	else if (Controller.ButtonR2.pressing()){	//Outtake (everything rev)
		Outtake.spin(reverse, 100, pct);
		Conveyor.spin(reverse, 100, pct);
		Intake.spin(reverse, 100, pct);
	}

	else { // else stop all
		Outtake.stop();
		Conveyor.stop();
		Intake.stop();
	}

	 if (Controller.ButtonL1.pressing()){  //Intakeing (outake) 
		Conveyor.spin(fwd, 100, pct);
		Outtake.spin(reverse, 10, pct);
		Intake.spin(fwd, 100, pct);}



//Scraper
	if (Controller.ButtonY.pressing()) {
			PneuSCRAPER.set(true);
		} else if (Controller.ButtonX.pressing()) {
			PneuSCRAPER.set(false);
		}

//Descore
	if (Controller.ButtonA.pressing()) {
			PneuDESCORE.set(true);
		} else if (Controller.ButtonB.pressing()) {
			PneuDESCORE.set(false);
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
