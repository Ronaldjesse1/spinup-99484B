#include "vex.h"


extern double Distance_Left;
extern double Distance_Right;
extern double Encoder_Value_Left ;
extern double Encoder_Value_Right;
extern double Encoder_Value_Lift ;
extern double Encoder_Value_Clamp;
extern double Encoder_Value_BackLift;
extern double absd(int v);
extern void Encoder_Value_Reset();
extern void Encoder_Value_Check();
extern void spin(motor name, double power);
extern void usercontrol();