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

void setSpeed(Motor& m, float speed)
{
  m.Speed = speed;
  m.Pwm = 200; // let's start from this value for the beginning
}

void setForward(Motor& m)
{
  m.Dir = HIGH;
}

void setBackward(Motor& m)
{
  m.Dir = LOW;
}

void encoderTick(Motor& m)
{
  m.EncoderTick++;
}

void updateMotorSpeed(Motor& m)
{
  /*unsigned long attemptedTickTime = m.Speed > 0 ? 1000000 / (MaxSpeed * m.Speed) : 1000000;
  if(m.Speed > 0 && attemptedTickTime < m.EncoderTickTime && m.Pwm < 255)
    m.Pwm++;
  if(attemptedTickTime > m.EncoderTickTime && m.Pwm > 0)
    m.Pwm--;
  m.LastEncoderTick = millis();*/
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

void leftEncoderISR()
{
  encoderTick(LeftMotor);
}

void rightEncoderISR()
{
  encoderTick(RightMotor);
}

void updateMotors()
{
  updateMotorSpeed(LeftMotor);
  updateMotorSpeed(RightMotor); 
  updateMotorPins(LeftMotor);
  updateMotorPins(RightMotor);
}

void setMove(float f, boolean forward)
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
  setSpeed(LeftMotor, f);
  setSpeed(RightMotor, f);
}

void setTurnInPlace(Direction dir, float speed)
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
  setSpeed(LeftMotor, speed);
  setSpeed(RightMotor, speed);
}

void setTurn(Direction dir)
{
  if(dir == Right)
  {
    setSpeed(RightMotor, RightMotor.Speed + 0.1);
  }
  else
  {
    setSpeed(LeftMotor, LeftMotor.Speed + 0.1);
  }
}

void setStraight()
{
  float speed = max(LeftMotor.Speed, RightMotor.Speed);
  setSpeed(LeftMotor, speed);
  setSpeed(RightMotor, speed);
}

void stop()
{
  LeftMotor.Speed = 0;
  RightMotor.Speed = 0;
  updateMotorPins(LeftMotor);
  updateMotorPins(RightMotor);
}

//----------------------------------------------------------

float MovementSpeed = 0.75;
float RotationSpeed = 0.75;

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
    case 'z':
      setTurn(Left);
      break;
    case 'x':
      setStraight();
      break;
    case 'c':
      setTurn(Right);
      break;
    case '[':
      MovementSpeed -= 0.05;
      break;
    case ']':
      MovementSpeed += 0.05;
      break;
    case ';':
      RotationSpeed -= 0.05;
      break;
    case '\'':
      RotationSpeed += 0.05;
      break;
    case 'i':
      Serial.print("Speed: ");
      Serial.print(LeftMotor.Speed);
      Serial.print(", ");
      Serial.println(RightMotor.Speed);
      Serial.print("PWM: ");
      Serial.print(LeftMotor.Pwm);
      Serial.print(", ");
      Serial.println(RightMotor.Pwm);    
      Serial.print("Wheel encoders: ");
      Serial.print(LeftMotor.EncoderTick);
      Serial.print(", ");
      Serial.print(RightMotor.EncoderTick);
      Serial.print("Movement/rotation speed: ");
      Serial.print(MovementSpeed);
      Serial.print(", ");
      Serial.println(RotationSpeed);
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
 
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachInterrupt(0, leftEncoderISR, RISING);
  attachInterrupt(1, rightEncoderISR, RISING);
  
  Serial.begin(9600);
  Serial.println("Thy bidding?");
}

int TestTicks = 16 * 4;
boolean Started = false;

void loop()
{
  if(!Started && !Serial.available())
    return;
  Started = true;
  
  LeftMotor.Dir = RightMotor.Dir = 0;
  
  for(int pwm = 50; pwm < 256; pwm += 5)
  {
    LeftMotor.EncoderTick = RightMotor.EncoderTick = 0;
    LeftMotor.Pwm = RightMotor.Pwm = pwm;
    updateMotorPins(LeftMotor);
    updateMotorPins(RightMotor);
    unsigned int v = analogRead(POWER_PIN);
    unsigned long t = micros();
    while(LeftMotor.EncoderTick < TestTicks || RightMotor.EncoderTick < TestTicks) {}
    t = micros() - t;
    Serial.print(v);
    Serial.print(", ");
    Serial.print(pwm);
    Serial.print(", ");
    Serial.print(t);
    Serial.print(", ");
    Serial.print(LeftMotor.EncoderTick);
    Serial.print(", ");
    Serial.println(RightMotor.EncoderTick);
  }
}

