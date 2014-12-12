struct Motor
{
  int PwmPin;
  int DirPin;

  char Dir;
  float Speed;
  int Pwm;

  volatile unsigned long EncoderTick;  
  unsigned long LastEncoderTick;
};
      
enum Direction
{
  Left,
  Right
};


