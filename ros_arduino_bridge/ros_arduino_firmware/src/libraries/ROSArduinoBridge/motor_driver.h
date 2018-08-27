/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 4
  #define LEFT_MOTOR_BACKWARD  5
  #define RIGHT_MOTOR_FORWARD  6
  #define LEFT_MOTOR_FORWARD   7
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
