/* PORT USAGE
leftMotorA          1
leftMotorB          2
rightMotorA         3
rightMotorB         4
*LeftDriveSmart     1,2
*RightDriveSmart    3,4
DrivetrainInertial  5
*Drivetrain         1,2, 3,4, 5
Collector           6
Catapult            8
DoubleReverseMotorA 9
DoubleReverseMotorB 10
*DoubleReverse      9,10
ThreeWirePort       22
SolenoidA           A
SolenoidB           B
*/

#include <math.h>
#include "vex.h"

using namespace vex;
brain Brain;


// Robot configuration code.
motor leftMotorA = motor(PORT1, ratio18_1, true);
motor leftMotorB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT3, ratio18_1, false);
motor rightMotorB = motor(PORT4, ratio18_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT5);
smartdrive Drivetrain = smartdrive(RightDriveSmart, LeftDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 1);

controller Controller1 = controller(primary);
motor Collector = motor(PORT6, ratio18_1, false);

triport ThreeWirePort = triport(PORT22);
digital_out SolenoidA = digital_out(ThreeWirePort.A);
digital_out SolenoidB = digital_out(ThreeWirePort.B);
motor Catapult = motor(PORT8, ratio18_1, false);

motor DoubleReverseMotorA = motor(PORT9, ratio36_1, false);
motor DoubleReverseMotorB = motor(PORT10, ratio36_1, false);
motor_group DoubleReverse = motor_group(DoubleReverseMotorA, DoubleReverseMotorB);

digital_in HomeJumper = digital_in(ThreeWirePort.G);
digital_in EnemyJumper = digital_in(ThreeWirePort.H);


void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}// define variable for remote controller enable/disable

bool RemoteControlCodeEnabled = true;
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1YAButtonsControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

bool DrivetrainToggle = false;
bool SolenoidToggle = false;
bool solToggleL = false;
bool solToggleR = false;


competition Competition;



/* FUNCTIONS */



void LockIt(){
  LeftDriveSmart.setStopping(hold);
  RightDriveSmart.setStopping(hold);
}
void UnlockIt(){
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
}

void MotorDrive(double x, double y){
  LeftDriveSmart.spinFor(x, degrees, false);
  RightDriveSmart.spinFor(y, degrees, true);
  
}

void TriggerHappy(int timems) 
{
  Catapult.spin(reverse);
  wait(timems, msec);
  Catapult.stop();
  return;
}

void DoubleSolenoid(bool isExtended){
  SolenoidA.set(isExtended);
  SolenoidB.set(isExtended);
  wait(50, msec);
}


void pre_auton(void) {
  calibrateDrivetrain();
  Catapult.setVelocity(75, percent); // catapult shoot speed
  Catapult.setMaxTorque(100, percent); // catapult torque
  Collector.setVelocity(100, percent);
  Catapult.setStopping(hold);
  DoubleSolenoid(false);
}

void autonomous(void) {
  Drivetrain.driveFor(-3, inches);
  Drivetrain.setHeading(0, degrees);
  Drivetrain.turnToHeading(15, degrees);
  Drivetrain.driveFor(-1, inches);
  TriggerHappy(30000);
  Drivetrain.turnToHeading(0, degrees);

  Drivetrain.driveFor(70, inches);
  Drivetrain.turnToHeading(90, degrees);
  Drivetrain.driveFor(10, inches);
  DoubleSolenoid(true);
  Drivetrain.driveFor(10, inches);

  // Smooth Turn Left
  RightDriveSmart.spin(fwd, 12, volt);
  LeftDriveSmart.spin(fwd, 6, volt);
  wait(1, seconds);
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
}

int main() {
  pre_auton();
  autonomous();

  while (true) {
    wait(100, msec);
  }
}
