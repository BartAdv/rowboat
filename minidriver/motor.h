struct Motor
{
  int PwmPin;
  int DirPin;

  char Dir;
  int Pwm;

  volatile unsigned long EncoderTick;
  volatile unsigned long LastEncoderTick;
  volatile unsigned long RotationTime;
  volatile unsigned long LastRotationTime;
};
      
enum Direction
{
  Left,
  Right
};


