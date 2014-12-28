// http://blog.dawnrobotics.co.uk/2013/11/getting-started-with-the-dagu-arduino-mini-driver-board/
// http://letsmakerobots.com/node/38636

#include "motor.h"

const int LEFT_MOTOR_DIR_PIN = 7;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 8;
const int RIGHT_MOTOR_PWM_PIN = 10;

const int LEFT_ENCODER_PIN = 2;
const int RIGHT_ENCODER_PIN = 3;

const int POWER_PIN = 7;

// encoder gearwheel doesn't give the same length for 'up' and 'down' so we are tracking only one kind of change
const int ENCODER_TICKS_PER_ROTATION = 16; 
int MaxSpeed = 44;

//----------------------------------------------------------

void setPwm(Motor& m, int pwm)
{
  analogWrite(m.PwmPin, pwm);
  m.Pwm = pwm;
}

void setForward(Motor& m)
{
  digitalWrite(m.DirPin, HIGH);
  m.Dir = HIGH;
}

void setBackward(Motor& m)
{
  digitalWrite(m.DirPin, LOW);
  m.Dir = LOW;
}

// called from ISR
void encoderTick(Motor& m)
{
  unsigned long t = micros();
  m.EncoderTick++;
  m.EncoderTickTime = t - m.LastEncoderTickTime;
  m.LastEncoderTickTime = t;
}

int leftPwm(unsigned long ticktime)
{
  return 10110101.81274920 * pow(ticktime, -1.10251771);
}

int rightPwm(unsigned long ticktime)
{
  return 270789377.66169600 * pow(ticktime, -1.43222890);
}

Motor LeftMotor = { 
  LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN };
Motor RightMotor = { 
  RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN };

Motor *const Motors[2] = { &LeftMotor, &RightMotor };

volatile boolean leftTick = false;

void leftEncoderISR()
{
  encoderTick(LeftMotor);
  leftTick = true;
}

volatile boolean rightTick = false;

void rightEncoderISR()
{
  encoderTick(RightMotor);
  rightTick = true;
}

void setMove(unsigned long speed, boolean forward)
{
  if(forward)
  {
    setForward(LeftMotor);
    setForward(RightMotor);
  }
  else
  {
    setBackward(LeftMotor);
    setBackward(RightMotor);
  }
  setPwm(LeftMotor, leftPwm(speed));
  setPwm(RightMotor, rightPwm(speed));
}

void setTurnInPlace(Direction dir, unsigned long speed)
{ 
  if(dir == Right)
  {
    setForward(LeftMotor);
    setBackward(RightMotor);
  }
  else
  {
    setBackward(LeftMotor);
    setForward(RightMotor);
  }
  setPwm(LeftMotor, leftPwm(speed));
  setPwm(RightMotor, rightPwm(speed));
}

void stop()
{
  for(int i=0; i<2; i++)
    setPwm(*Motors[i], 0);
}

//----------------------------------------------------------

unsigned long MovementSpeed = 20000;
unsigned long RotationSpeed = 8000;

void processSerialInput()
{
  if(Serial.available() > 0)
  {
    int cmd = Serial.read();
    switch(cmd)
    {
    case 'a':
      setTurnInPlace(Left, RotationSpeed);
      break;
    case 'd':
      setTurnInPlace(Right, RotationSpeed);
      break;
    case 'w':
      setMove(MovementSpeed, true);
      break;
    case 's':
      setMove(MovementSpeed, false);
      break;
    case '[':
      MovementSpeed += 1000;
      break;
    case ']':
      MovementSpeed -= 1000;
      break;
    case ';':
      RotationSpeed += 1000;
      break;
    case '\'':
      RotationSpeed -= 1000;
      break;
    case 'r':
      LeftMotor.EncoderTick = RightMotor.EncoderTick = 0;       
      break;
    case 'i':
      Serial.print("PWM: ");
      Serial.print(LeftMotor.Pwm);
      Serial.print(", ");
      Serial.println(RightMotor.Pwm);    
      Serial.print("Wheel encoders: ");
      Serial.print(LeftMotor.EncoderTick);
      Serial.print(", ");
      Serial.println(RightMotor.EncoderTick);
      Serial.print("Rotation time: ");
      Serial.print("Movement/rotation speed: ");
      Serial.print(MovementSpeed);
      Serial.print(", ");
      Serial.println(RotationSpeed);
      Serial.print("Voltage: ");
      Serial.println(analogRead(POWER_PIN));
      break;
    default: // PANIC!
      stop();
    }
  }
}

void setup()
{
  // Setup the pins
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
 
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  
  attachInterrupt(0, leftEncoderISR, RISING);
  attachInterrupt(1, rightEncoderISR, RISING);
  
  pinMode(13, OUTPUT);
  
  Serial.begin(115200);
  
  //digitalWrite(13, HIGH);
  Serial.println("Thy bidding?");
  //delay(100);
  //digitalWrite(13, LOW);
}

void dumpTicks(Motor& m)
{
  Serial.print("pwm, ts, tt: ");
  Serial.print(m.Pwm);
  Serial.print(",");
  Serial.print(m.EncoderTick);
  Serial.print(",");
  Serial.println(m.EncoderTickTime);
}

void dumpTicks()
{
  static unsigned long lastl, lastr;
  
  if(LeftMotor.EncoderTick != lastl)
  {
    Serial.print("Left ");
    dumpTicks(LeftMotor);
    lastl = LeftMotor.EncoderTick;
  }
  if(RightMotor.EncoderTick != lastr)
  {
    Serial.print("Right ");
    dumpTicks(RightMotor);
    lastr = RightMotor.EncoderTick;
  }
}

void dumpReadings()
{

}

boolean Done = false;

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

void loop()
{ 
  Motor& m = LeftMotor;
  setForward(m);
  setPwm(m, 150);
  processSerialInput();
  dumpTicks();
  return;
  
  for(int pwm=150; pwm<=170; pwm+=5)
  {
    setForward(LeftMotor);
    setForward(RightMotor);
    setPwm(LeftMotor, pwm);
    setPwm(RightMotor, pwm);
    LeftMotor.EncoderTick = RightMotor.EncoderTick = 0;
    
    Motor& m = RightMotor;
    volatile boolean& ticked = rightTick;
    
    while(m.EncoderTick < 128)
    {
      while(!ticked) {}
      noInterrupts();
      ticked = false;
      unsigned long tc = m.EncoderTick;
      unsigned long tt = m.EncoderTickTime;
      interrupts();
        
      Serial.write((char*)&pwm, 2);
      Serial.write((char*)&tt, 4);
    }
  }
}

