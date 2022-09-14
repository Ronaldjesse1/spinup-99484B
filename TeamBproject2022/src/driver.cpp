#include "vex.h"
#include "robot-config.h"
#include <cmath>
#include "driver.h"

int rotate (motor name) {
  return name.rotation(rotationUnits::raw);
}
void spin(motor name, double power){
  name.spin(directionType::fwd, power, velocityUnits::pct);
}
void acceleration();
double Distance_Left;
double Distance_Right;
double absd(int v) {
  if (v > 0) {
    return v;
  }
  else {
    return -v; 
  }
}
void Encoder_Value_Reset(){
  RightFront.resetRotation();
  LeftFront.resetRotation();
  RightBack.resetRotation();
  LeftBack.resetRotation();
  LiftMotorL.resetRotation();
  LiftMotorR.resetRotation();
  BackLift.resetRotation();
}

double Encoder_Value_Left ;
double Encoder_Value_Right;
double Encoder_Value_Lift ;
double Encoder_Value_Clamp;
double Encoder_Value_BackLift;
void Encoder_Value_Check(){
  Encoder_Value_Left = (LeftBack.rotation(rotationUnits::raw) + LeftFront.rotation(rotationUnits::raw))/2;
  Encoder_Value_Right = (RightBack.rotation(rotationUnits::raw) + RightFront.rotation(rotationUnits::raw))/2;
  Encoder_Value_Lift = (LiftMotorL.rotation(rotationUnits::raw) + LiftMotorR.rotation(rotationUnits::raw))/2;
  Encoder_Value_Clamp = ClawMotor.rotation(rotationUnits::raw);
  Encoder_Value_BackLift=(BackLift.rotation(rotationUnits::raw));
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Chassis_Left: %f", Encoder_Value_Left);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Chassis_Right: %f", Encoder_Value_Right);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Lift: %f", Encoder_Value_Lift);
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Clamp: %f", Encoder_Value_Clamp);
}



void run1(){
  // User control code here, inside the loop
 if(absd(Controller1.Axis2.value())> 10||absd(Controller1.Axis3.value())>10){
    spin(LeftFront, Controller1.Axis3.value());
    spin(LeftBack,Controller1.Axis3.value());
    spin(RightBack,Controller1.Axis2.value());
    spin(RightFront,Controller1.Axis2.value());
 }
 else{
    spin(LeftBack,0);
    spin(LeftFront,0);
    spin(RightBack,0);
    spin(RightFront,0);
 }
  
}

void splitarcade(){

  spin(LeftFront, (Controller1.Axis3.value() + Controller1.Axis1.value())/2);
  spin(LeftBack,(Controller1.Axis3.value() + Controller1.Axis1.value())/2);
  spin(RightBack,(Controller1.Axis3.value() - Controller1.Axis1.value())/2);
  spin(RightFront,(Controller1.Axis3.value() - Controller1.Axis1.value())/2);
  
}
// void instantBrake(){
//   if(Controller1.ButtonR2.pressing())
//   LeftBack.setBrake(brakeType::hold);
//   LeftFront.setBrake(brakeType::hold);
//   RightBack.setBrake(brakeType::hold);
//   RightFront.setBrake(brakeType::hold);
// }
void Intake(){

  spin(intake, 100);
}

bool i = false;
// void claw1(){
//   bool check1=Controller1.ButtonR1.pressing();
//   bool check2=Controller1.ButtonR2.pressing();
  
//   if (check1){
//     spin(ClawMotor,100);
//     i=1;
//   } else if (check2){
//     spin(ClawMotor,-100);
//     i=0;
//   } else if(check1==0&&i==1){
    
//     spin(ClawMotor,0);
//     ClawMotor.setBrake(brakeType::hold);
//   } else {
//     ClawMotor.setBrake(brakeType::coast);
//     ClawMotor.stop();
//   }
//   Brain.Screen.setCursor(1,7);
//   Brain.Screen.print(i);
//    Brain.Screen.setCursor(1,8);
//   Brain.Screen.print(check1);
// }
void clamp(){
  bool activate = Controller1.ButtonR1.pressing();
  if(activate){
    piston.close();
  } else {
    piston.open();
  }

}
int usercontrol_loops = 0 ;
int mode=2;
void usercontrol(void){
  
  Encoder_Value_Reset();
  while(1) {
    usercontrol_loops += 1;
    Encoder_Value_Check();
    
    
    Brain.Screen.print(usercontrol_loops);
    if(mode==1){
      run1();
    } else {
      splitarcade();
    }
    
    // lift1();
    
    clamp();
    // backlift1();
  }
}


