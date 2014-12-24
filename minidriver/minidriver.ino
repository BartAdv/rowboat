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
  m.Pwm = pwm;
}

void setForward(Motor& m)
{
  m.Dir = HIGH;
}

void setBackward(Motor& m)
{
  m.Dir = LOW;
}

// called from ISR
void encoderTick(Motor& m)
{
  unsigned long t = micros();
  m.EncoderTick++;
  if(m.EncoderTick % ENCODER_TICKS_PER_ROTATION == 0)
  {
    m.RotationTime = t - m.LastRotationTime;
    m.LastRotationTime = t;
  }
}

int leftPwm(unsigned long ticktime)
{
  return 10110101.81274920 * pow(ticktime, -1.10251771);
}

int rightPwm(unsigned long ticktime)
{
  return 270789377.66169600 * pow(ticktime, -1.43222890);
}

void updateMotorPins(Motor& m)
{
  analogWrite(m.PwmPin, m.Pwm);
  digitalWrite(m.DirPin, m.Dir);
}

Motor LeftMotor = { 
  LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN };
Motor RightMotor = { 
  RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN };

const int TRANSIENT_LENGTH  = 10000;

bool detectTransient(byte signal, unsigned long& lastFall, unsigned long& lastRise)
{
  unsigned long t = micros();
  
  if(signal == LOW && t - lastRise > TRANSIENT_LENGTH)
  {
    lastFall = t;
    return false;
  }
  if(signal == HIGH && t - lastFall > TRANSIENT_LENGTH)
  {
    lastRise = t;
    return false;
  }
  return true;
}

void leftEncoderISR()
{
  /*static unsigned long lastRise, lastFall;
  int v = digitalRead(LEFT_ENCODER_PIN);
  if(detectTransient(v, lastRise, lastFall))
    return;
  
  if(v == HIGH)*/
    encoderTick(LeftMotor);
}

void rightEncoderISR()
{
  /*static unsigned long lastRise, lastFall;
  int v = digitalRead(RIGHT_ENCODER_PIN);
  if(detectTransient(v, lastRise, lastFall))
    return;
  
  if(v == HIGH)*/
    encoderTick(RightMotor);
}

void setMove(unsigned long speed, boolean forward)
{
  LeftMotor.LastRotationTime = RightMotor.LastRotationTime = micros();
  LeftMotor.RotationTime = RightMotor.RotationTime = 0;
  
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
  LeftMotor.Pwm = 0;
  RightMotor.Pwm = 0;
  updateMotorPins(LeftMotor);
  updateMotorPins(RightMotor);
}

void updateMotors()
{
  updateMotorPins(LeftMotor);
  updateMotorPins(RightMotor);
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
      Serial.print(LeftMotor.RotationTime);
      Serial.print(", ");
      Serial.println(RightMotor.RotationTime);
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

void dumpRotationTime(Motor& m)
{
  Serial.print("Pwm, Rotation time: ");
  Serial.print(m.Pwm);
  Serial.print(", ");
  Serial.println(m.RotationTime);
}

void dumpRotationInfo()
{
  static unsigned long lastlrot = 0;
  static unsigned long lastrrot = 0;
  
  if(LeftMotor.RotationTime != lastlrot)
  {
    Serial.print("Left ");
    dumpRotationTime(LeftMotor);
    lastlrot = LeftMotor.RotationTime;
  }
  if(RightMotor.RotationTime != lastrrot)
  {
    Serial.print("Right ");
    dumpRotationTime(RightMotor);
    lastrrot = RightMotor.RotationTime;
  }
}

void dumpTicks(Motor& m)
{
  Serial.print("ticks: ");
  Serial.println(m.EncoderTick);
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

boolean Started = false;

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

void loop()
{
  unsigned long t = micros();
  int v = analogRead(A0);
  Serial.write((char*)&t, 4);
  Serial.write((char*)&v, 2);
}

