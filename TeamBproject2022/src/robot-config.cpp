#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller1 = controller(primary);
motor RightFront (PORT11, true );
motor LeftFront  (PORT20, false);
motor RightBack  (PORT1, true  );
motor LeftBack   (PORT10, false );
motor intake (PORT16, true);
motor LiftMotorL (PORT2, true  );//L1 up, L2 down
motor LiftMotorR (PORT9, false );//L1 up, L2 down
motor ClawMotor  (PORT7, true  ); //R1 up, R2 down, R1 release hold, R2 release coast
motor BackLift   (PORT3, false ); // arrows up down
pneumatics piston (Brain.ThreeWirePort.G);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
*/

/* This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}