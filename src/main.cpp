

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

motor LF (PORT10, ratio6_1, true);
motor LM(PORT18, ratio6_1, true);
motor LB(PORT1, ratio6_1, true);
motor RF(PORT14, ratio6_1, false);
motor RM(PORT20, ratio6_1, false);
motor RB(PORT2, ratio6_1, false);

motor Intake(PORT13, ratio18_1, false);
motor Conveyor(PORT16, ratio6_1, true);
motor Outtake (PORT12, ratio6_1, true);

inertial Gyro = inertial(PORT6);

digital_out PneuALIGNER = digital_out(Brain.ThreeWirePort.C); //Aligner
digital_out PneuGOAlFLAP = digital_out(Brain.ThreeWirePort.A); //Flap
digital_out PneuDESCORE = digital_out(Brain.ThreeWirePort.B); //Descorer
digital_out PneuSCRAPER = digital_out(Brain.ThreeWirePort.D); //Scraper

controller Controller;

/*---------------------------------------------------------------------------*\
|*                          Pre-Autonomous Functions                         *|
\*---------------------------------------------------------------------------*/
 
//set neccesary variables

float pi = 3.14;
float wheeld = 3.25;
float wheelr = wheeld / 2;
float wheelc = pi * wheeld;
float gearratio = 0.75;

int AutonSelected = 2;
int AutonMin = 0;
int AutonMax = 4;

// int autoselected = 0;
// int autonmin = 0;
// int autonmax = 2;
 
bool preAuton = true; 
 




//drive/turn commands


  void Drive(int Lspeed, int Rspeed, int wt){
    
    LF.spin(fwd, Lspeed, pct);
	LM.spin(fwd, Lspeed, pct);
    LB.spin(fwd, Lspeed, pct);
    RF.spin(fwd, Rspeed, pct);
	RM.spin(fwd, Rspeed, pct);
    RB.spin(fwd, Rspeed, pct);

    wait (wt, msec);
  }

	void drivestop(){
		LF.stop();
		LM.stop();
		LB.stop();
		RF.stop();
		RM.stop();
		RB.stop();
	}

  


void gyroturn(float target, double timeOut = 2)
{
		float heading=0.0; //initialize a variable for heading
		float accuracy=5.0; //how accurate to make the turn in degrees
		float error=target-heading;
		float kp= 0.3;
		float speed=kp*error;
		Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
		double startTime = Brain.timer(seconds);
		 		while(fabs(error)>=accuracy)
		{
			speed=kp*error;
			Drive(speed, -speed, 10); //turn right at Speed
			heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;  //calculate error
			if (Brain.timer(seconds)- startTime > timeOut) break; 
		}
			Brain.Screen.printAt(10, 20, "Gyro Reading= %.2f", heading); 
			Drive(0, 0, 0);  //stope the drive
}


void inchdrive (float inches, double timeOut ){
	float x = 0;
	float error = inches - x;
	float kp = 3;
	float speed = error * kp;
	float accuracy = 0.5; 
	LF.resetPosition(); 
	x = LF.position(rev)*3.25*0.75*M_PI; 
	double startTime = Brain.timer(seconds);

	while ( fabs(error)>= accuracy) { 
		Drive(speed, speed, 10); 
		x = LF.position(rev)*3.25*0.75*M_PI; 
		error = inches - x;
		speed = error * kp;
		if (Brain.timer(seconds)- startTime > timeOut) break;

	}
	Drive(0,0,0); 

}








//Display/motor monitor stuff past here





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


void gyroprint(){
	float rotation = Gyro.rotation (deg);
	Brain.Screen.printAt(1, 60, "Rotation = %.2f.degrees", rotation);
}








//subsystem 3 functions



//scoring, intake and conveyor

void intake (){ 
	Conveyor.spin(fwd, 50, pct);
	Intake.spin(fwd, 100, pct);
	Outtake.stop();
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

void stopscore (){
	Conveyor.stop();
	Outtake.stop();
	Intake.stop();
}

//individual pneumatic functions

void scraperup (){
	PneuSCRAPER.set(true);
}

void scraperdown(){
	PneuSCRAPER.set(false);
}

void descoreup(){
	PneuDESCORE.set(false);
}

void descoredown(){
	PneuDESCORE.set(true);
}

void goalflapup(){
	PneuGOAlFLAP.set(false);
}

void goalflapdown(){
	PneuGOAlFLAP.set(true);
}

void alignerup(){
	PneuALIGNER.set(false);
}

void alignerdown(){
	PneuALIGNER.set(true);
}

//Pneumatic logic commands (pt stands for pneumatic logic)

void plretractall(){
	scraperup();
	descoredown();
	goalflapup();
	alignerdown();
}

void plscraperdown(){
	scraperdown();
	goalflapup();
	alignerdown();
}


void plalignerup(){
	scraperup();
	alignerup();
}


void plalignerdown(){
	scraperup();
	alignerdown();
}

void plgoalflapup(){
	scraperup();
	goalflapup();
}

void plgoalflapdown(){
	scraperup();
	goalflapdown();
}


//Penumatic Toggle commands (pt for penumatic toggle)

void ptscraper(){
	plscraperdown();
}

void ptaligner(){

}



//preauton gui/selection etc.



void drawGUI() {
	// Draws 2 buttons to be used for selecting auto
	Brain.Screen.clearScreen();
	Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
	Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
	Brain.Screen.setFillColor(red);
	Brain.Screen.drawRectangle(20, 50, 100, 100);
	Brain.Screen.drawCircle(300, 75, 25);
	Brain.Screen.printAt(25, 75, "Select");
	Brain.Screen.setFillColor(green);
	Brain.Screen.drawRectangle(170, 50, 100, 100);
	Brain.Screen.printAt(175, 75, "GO");
	Brain.Screen.setFillColor(black);
}



void selectAuton() {
		bool selectingAuton = true;
		
		int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
		int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen
		
		// check to see if buttons were pressed
		if (x >= 20 && x <= 120 && y >= 50 && y <= 150){ // select button pressed
				AutonSelected++;
				if (AutonSelected > AutonMax){
						AutonSelected = AutonMin; // rollover
				}
				Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
		}
		
		
		if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
				selectingAuton = false; // GO button pressed
				Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", AutonSelected);
		}
		
		if (!selectingAuton) {
				Brain.Screen.setFillColor(green);
				Brain.Screen.drawCircle(300, 75, 25);
		} else {
				Brain.Screen.setFillColor(red);
				Brain.Screen.drawCircle(300, 75, 25);
		}
		
		wait(10, msec); // slow it down
		Brain.Screen.setFillColor(black);
}

 






void pre_auton(void) {

plretractall();

while(Gyro.isCalibrating()){
	wait(20, msec); 
}

// Initializing Robot Configuration. DO NOT REMOVE!
		Brain.Screen.printAt(1, 40, "pre auton is running");
		drawGUI(); 
		Brain.Screen.pressed(selectAuton);

		PneuDESCORE.set(true);

// Print which code for selection screen
		while (preAuton){ 
		if(AutonSelected == 0) { 
			Brain.Screen.clearLine(220);
			Brain.Screen.printAt(1, 220, "Left Side" ); 


		}
		else if(AutonSelected == 1) { 
			Brain.Screen.clearLine(220); 
			Brain.Screen.printAt(1, 220, "Right Side" ); 
		}
		else if(AutonSelected == 2) { 
			Brain.Screen.clearLine(220); 
			Brain.Screen.printAt(1, 220, "Auton Skills" ); 
		
		}
		else if(AutonSelected == 3) { 
			Brain.Screen.clearLine(220); 
			Brain.Screen.printAt(1, 220, "PID Turn Test" ); 
		}
		else if(AutonSelected == 4) { 
			Brain.Screen.clearLine(220); 
			Brain.Screen.printAt(1, 220, "PID Drive Test" ); 
		}

}

//descore down 
			



	Brain.Screen.clearScreen(); 


}

		
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...


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

// //left side Autonomous
// 	IntakeBalls(); 
// 	inchdrive(28);
// 	StopIntake(); 
// 	inchdrive(-15);
// 	gyroturn(110);  
// 	inchdrive(-26); 
// 	gyroturn(90); 
// 	inchdrive(-12.5); 
// 	ScoreBalls();


//----------------------------------------------\|

	// // Right Side Autononomous
	
	// IntakeBalls(); 
	// inchdrive(28);
	// StopIntake(); 
	// inchdrive(-15);
	// gyroturn(-120);  
	// inchdrive(-30); 
	// gyroturn(-88); 
	// inchdrive(-6); 
	// ScoreBalls();
	

	switch (AutonSelected) {
				case 0:
					//code 0
					//Left Side Autonomous
	scraperup();	
	intake();
	inchdrive(24,2);//pick up trio blocks
	wait(500, msec);
	inchdrive(-12, 1);
	gyroturn(-80);
	inchdrive (29, 2);//to long goal area
	gyroturn (-92);// turn so the back of robot faces the long goal
	wait(250, msec);
	inchdrive (-6, 1);//go to long goal
	score();//in long goal
	wait(1000, msec);
	intake();
	scraperdown();
	inchdrive(27, 3);//to loader
	
	wait(300,msec);
	//back to reg
	inchdrive(-27.5, 2);//score loader
	score();



					break;
				
				case 1:
					//code 1
					// Right Side Autononomous
	
	intake();
	inchdrive(24, 2);
	wait(100, msec);
	//inchdrive(-8, 1);
	gyroturn(-60);
	outtake();

	inchdrive(11, 1.5);
	
	
	wait(1000, msec);
	
	stopscore ();
	
	
	
	// gyroturn(80);
	// inchdrive (29);
	// gyroturn (90);
	// inchdrive (-5.5);
	// score();
					break;
				
				case 2:
					//code 2
					//skills left
	scraperup();	
	intake();
	inchdrive(24,2);//pick up trio blocks
	wait(500, msec);
	inchdrive(-12, 1);
	gyroturn(-73);
	inchdrive (27.5, 2);//to long goal area
	gyroturn (-100);// turn so the back of robot faces the long goal
	wait(250, msec);
	inchdrive (-7, 1);//go to long goal
	score();//in long goal
	wait(3000, msec);
	intake();
	scraperdown();
	Drive(50, 50, 10);//to loader
	wait(1500, msec);
	drivestop();
	wait(3000, msec);
	inchdrive(-2, 1);
	scraperup();
	inchdrive(-25.5, 2);//to long goal
	score();
	wait (2000, msec);
	stopscore();
	gyroturn (90);
	inchdrive (25, 1.5);//slam into wall
	wait (500, msec);
	inchdrive(-60, 5);	//front of park zone
	gyroturn (-100); //face park zone
	score();
	Drive(100, 100, 10);

	// inchdrive (-40, 2); //go next to park zone
	// gyroturn(-90);
	// inchdrive (30, 1);
	// inchdrive (-2, 1.5);
	// gyroturn (-86);
	// score();
	// inchdrive(43, 10);
	


	
	// gyroturn(-101.5);
	 //inchdrive(-3.5, 1);
	// PneuSCRAPER.set(true);
	 
	// Drive(40, 40, 650); 
	// Drive(0,0,0);

					break;
				
				case 3:
					//code 3
	gyroturn(90);
					break;

				case 4:
	inchdrive(10, 2);
					break;
		}


	}
//Test Code

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
// inchdrive(36);
// gyroturn(-90);
// inchdrive(-4);
// PneuSCRAPER.set(true);
// intake();
// Drive(40, 40, 650); 

// // Drive(-40, -40, 500);   was too odd:%
// // Drive(40, 40, 600); 
// // Drive(-40, -40, 500);
// // Drive(40, 40, 600); 
// // Drive(-40, -40, 500);
// // Drive(40, 40, 600); 
// // Drive(-40, -40, 500);
// // Drive(40, 40, 600); 

// Drive(0, 0, 0); // we should have gotten all the blocks
// Drive(40, 40, 400 ); 
// Drive(0,0,0); 
// wait(4500, msec);
// // inchdrive(-2);
// // inchdrive(2);
// // inchdrive(-2);
// // inchdrive(2);
// // inchdrive(-2);
// // inchdrive(2);
// // gyroturn(45);
// // inchdrive(3);
// // gyroturn(-45);
// inchdrive(-24.5);
// score();




/*---------------------------------------------------------------------------*\
|*                                                                           *|
|*                              User Control Task                            *|
|*                                                                           *|
|*  This task is used to control your robot during the user control phase of *|
|*                                                                           *|
|*  You must modify the code to add your own robot specific commands here.   *|
\*---------------------------------------------------------------------------*/

void usercontrol(void) {
Brain.Screen.clearScreen(); 


  while (1) {
//Motor Monitor

Display ();


//drivecode


    int Lspeed = Controller.Axis3.position(pct);
    int Rspeed = Controller.Axis2.position(pct);

    Drive(Lspeed, Rspeed, 10);

	//y=6x+5 + |!-5357|


//Scoring

if (Controller.ButtonR1.pressing()){  //Scoring (all motors spinning fwd)
		score();
		}

	else if (Controller.ButtonR2.pressing()){	//Outtake (everything rev)
		outtake();
	}

	else { // else stop all
		stopscore();
	}


if (Controller.ButtonL1.pressing()){  //Intakeing (outake stop) 
	intake();
}



			//turn around
			//gyroturn(-180);
			//wait(1000 ,msec);
		


//Flap (for middle/long goal switching)
	if (Controller.ButtonY.pressing()) {
			goalflapdown();
		} else if (Controller.ButtonX.pressing()) {
			goalflapup();
		}

//Descore
	if (Controller.ButtonB.pressing()) {
			descoredown();
		} else if (Controller.ButtonA.pressing()) {
			descoreup();
		}


	
	if (Controller.ButtonUp.pressing()) {
			scraperup();
		} else if (Controller.ButtonRight.pressing()) {
			scraperdown();
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
