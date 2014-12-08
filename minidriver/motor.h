struct Motor
{
  int SpeedPin;
  int DirPin;
  
  float Efficiency;
    
  int Dir;
  int AttemptedForce;
  int AppliedForce;
  int Encoder;
  
  int LastEncoderValue;
  int LastEncoder;
  int LastAppliedForce;
};
      
enum Direction
{
  Left,
  Right
};


