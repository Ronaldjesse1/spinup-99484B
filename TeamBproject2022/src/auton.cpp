#include "vex.h"
#include "robot-config.h"
#include "auton.h"
#include "driver.h"
#include <cmath>
#include <chrono>

void turn(int v){
  spin(LeftFront,v);
  spin(LeftBack,v);
  spin(RightBack,-v);
  spin(RightFront,-v);
}

void Chassis_Right_Run (int power) {
  RightBack.spin(directionType::fwd, power, percentUnits::pct);
  RightFront.spin(directionType::fwd, power, percentUnits::pct);
}
void Chassis_Left_Run (int power) {
  LeftBack.spin(directionType::fwd, power, percentUnits::pct);
  LeftFront.spin(directionType::fwd, power, percentUnits::pct);
}
void Chassis_Right_Brake () {
  RightBack.setBrake(brakeType::brake);
  RightFront.setBrake(brakeType::brake);
}
void Chassis_Left_Brake () {
  LeftBack.setBrake(brakeType::brake);
  LeftFront.setBrake(brakeType::brake);
}
void Lift (int power) {
  LiftMotorL.spin(directionType::fwd, power, percentUnits::pct);
  LiftMotorR.spin(directionType::fwd, power, percentUnits::pct);
}
void Lift_Brake () {
  LiftMotorL.setBrake(brakeType::hold);
  LiftMotorR.setBrake(brakeType::hold);
}





// float c;
// void Chassis_Move_fwd (int power, double distance) {
//   double init_distance = distance;
//   Encoder_Value_Reset();
//   Encoder_Value_Check();
  
//   while ((Encoder_Value_Right + Encoder_Value_Left)/2 < distance) {
//     Encoder_Value_Check();
//     Chassis_Left_Run(power *PID_tutorial(distance/init_distance));
//     Chassis_Right_Run(power * PID_tutorial(distance/init_distance));
//   }
  
//   Chassis_Left_Run(0);
//   Chassis_Right_Run(0);
//   Chassis_Left_Brake();
//   Chassis_Right_Brake();

// }



void Chassis_Move_fwd (int power, double distance) {
  Encoder_Value_Reset();
  Encoder_Value_Check();
  
  while ((Encoder_Value_Right + Encoder_Value_Left)/2 < distance) {
    Encoder_Value_Check();
    Chassis_Left_Run(power);
    Chassis_Right_Run(power);
  }
  
  Chassis_Left_Run(0);
  Chassis_Right_Run(0);
  Chassis_Left_Brake();
  Chassis_Right_Brake();

}
void Chassis_Move_rev (int power, double distance) {
  Encoder_Value_Reset();
  Encoder_Value_Check();
  while (absd(Encoder_Value_Right + Encoder_Value_Left)/2 < distance) {
    Encoder_Value_Check();
    Chassis_Left_Run(-power);
    Chassis_Right_Run(-power);
  }
  Chassis_Left_Run(0);
  Chassis_Right_Run(0);
  Chassis_Left_Brake();
  Chassis_Right_Brake();
}
void Chassis_rev_time (int power, double time) {
  Encoder_Value_Reset();
  Chassis_Left_Run(-power);
  Chassis_Right_Run(-power);
  wait(time, seconds);
  Chassis_Left_Run(0);
  Chassis_Right_Run(0);
  Chassis_Left_Brake();
  Chassis_Right_Brake();
}
void Chassis_Turn_Left (int power, double distance) {
  Encoder_Value_Reset();
  while ((Encoder_Value_Right - Encoder_Value_Left)/2 < distance) {
    Encoder_Value_Check();
    Chassis_Left_Run(-power);
    Chassis_Right_Run(power);   
  } 
  Chassis_Left_Run(0);
  Chassis_Right_Run(0); 
  Chassis_Left_Brake();
  Chassis_Right_Brake();
}
void Chassis_Turn_Right (int power, double distance) {
  Encoder_Value_Reset();
  while ((Encoder_Value_Left - Encoder_Value_Right)/2 < distance) {
    Encoder_Value_Check();
    Chassis_Left_Run(power);
    Chassis_Right_Run(-power);
  }
  Chassis_Left_Run(0);
  Chassis_Right_Run(0);
  Chassis_Left_Brake();
  Chassis_Right_Brake();
}
void Run_Lift (double distance) {
  Encoder_Value_Reset();
  if (distance > 0) {
    while (Encoder_Value_Lift < distance) {
      Encoder_Value_Check();
      Lift(100);
    }
  }
  else if (distance < 0) {
    while (absd(Encoder_Value_Lift) < absd(distance)) {
      Encoder_Value_Check();
      Lift(-100);
    }
  }
  Lift(0);
  Lift_Brake();
}

void Run_Clamp (int mode) {
  if (mode == 1) {
    
    ClawMotor.spin(directionType::fwd, 100, percentUnits::pct);
    wait(0.5, seconds);
      
    
  }
  else if (mode == 0) {
    
    ClawMotor.spin(directionType::fwd, -100, percentUnits::pct);
    wait(0.5, seconds);
    
    
  }
  ClawMotor.spin(directionType::fwd, 0, percentUnits::pct);
  ClawMotor.setBrake(hold);
}
void Backlift (double distance) {
  Encoder_Value_Reset();
  if (distance > 0) {
    while (Encoder_Value_BackLift < distance) {
      Encoder_Value_Check();
      BackLift.spin(directionType::fwd, 100, percentUnits::pct);
    }
  }
  else if (distance < 0) {
    
    
    
    if (Encoder_Value_BackLift < absd(distance)) {
      Encoder_Value_Check();
      BackLift.spin(directionType::fwd, -100, percentUnits::pct);
    }
  }
  BackLift.stop();
  BackLift.setBrake(brakeType::hold);
}
void Chassis_backup(){
  Chassis_Right_Run(-50);
  Chassis_Left_Run(-50);
  wait(0.2, seconds);
  Chassis_Left_Brake();
  Chassis_Right_Brake();
}

double error;
double deriavative;
double integral;
double speed;
double prev_error;
void PID(double target_distance, motor x,double kP,double kD, double kI){
  // int leftMotorPosition = (LeftBack.position(degrees)+LeftFront.position(degrees))/2;
  // int rightMotorPosition = (RightBack.position(degrees)+RightFront.position(degrees))/2;
  // int average_position = (leftMotorPosition+rightMotorPosition)/2;
  
  
  // preverror = error;
  // task::sleep(20);
  // return 1;
  while(true){
    error = target_distance - x.rotation(rotationUnits::raw);
    integral = integral + error;

    if (error == 0) {
    integral = 0; 
    }
    // if ( absd(error) > 20) {
    // integral = 0;
    // }
    deriavative = error - prev_error;
    prev_error = error;

    speed = kP*error + kD*deriavative +kI*integral;
    spin(x,speed);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Chassis_Left: %f", error);
    if(x.rotation(rotationUnits::raw)==target_distance){
      break;
    }
  } 
  
}
//300 = 17cm
int autonroute = 1;
/*
Autonroute 1 --> winpoint
Autonroute 2 --> tournament
Autonroute 3 --> Skills
Autonroute 4 --> winpoint + (test)
*/
void autonomous(void){
  if(autonroute == 1){

  } 
  

}





  





