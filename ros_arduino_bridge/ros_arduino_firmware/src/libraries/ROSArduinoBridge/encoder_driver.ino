/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

void initEncoder()
{
  pinMode(LEFT_ENC_PIN_A,INPUT);
  pinMode(LEFT_ENC_PIN_B,INPUT);
  attachInterrupt(0,encoderLEFT_ISR,CHANGE);
  attachInterrupt(1,encoderLEFT_ISR,CHANGE);

  pinMode(RIGHT_ENC_PIN_A,INPUT);
  pinMode(RIGHT_ENC_PIN_B,INPUT);
  attachInterrupt(0,encoderRIGHT_ISR,CHANGE);
  attachInterrupt(1,encoderRIGHT_ISR,CHANGE);

}

void encoderLEFT_ISR()
{
  if (directionWHeel(LEFT)==BACKWARD)
  {
    left_enc_pos--;
  }
  else
  {
    left_enc_pos++;
  }
}
void encoderRIGHT_ISR()
{
  if (directionWHeel(RIGHT)==BACKWARD)
  {
    right_enc_pos--;
  }
  else
  {
    right_enc_pos++;
  }
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return left_enc_pos;
  else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT){
    left_enc_pos=0L;
    return;
  } else { 
    right_enc_pos=0L;
    return;
  }
}

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}


