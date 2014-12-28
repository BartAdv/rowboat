struct Motor
{
  int PwmPin;
  int DirPin;

  char Dir;
  int Pwm;

  volatile unsigned long EncoderTick;
  volatile unsigned long EncoderTickTime;
  volatile unsigned long LastEncoderTickTime;
};
      
enum Direction
{
  Left,
  Right
};


