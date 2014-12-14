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

void encoderTick(Motor& m)
{
  m.EncoderTick++;
}

int leftPwm(unsigned long ticktime)
{
  return 6892550.92794713 * pow(ticktime, -1.05923146);
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
  updateMotorPins(LeftMotor);
  updateMotorPins(RightMotor);
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
  LeftMotor.Pwm = 0;
  RightMotor.Pwm = 0;
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

void loop()
{
  processSerialInput();
  updateMotors();
}

