// http://blog.dawnrobotics.co.uk/2013/11/getting-started-with-the-dagu-arduino-mini-driver-board/

#include "motor.h"

const int LEFT_MOTOR_DIR_PIN = 7;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 8;
const int RIGHT_MOTOR_PWM_PIN = 10;

const int LEFT_ENCODER_PIN = 2;
const int RIGHT_ENCODER_PIN = 3;

const int DRIVE_FORWARD_TIME_MS = 1500;
const int TURN_TIME_MS = 2000;

const int MOTOR_CALIBRATION_TIME = 1000;
int MotorCalibrationStart;
CalibrationPhase MotorCalibrationProgress;

int getLeftEncoderValue()
{
  return digitalRead(LEFT_ENCODER_PIN);
}

int getRightEncoderValue()
{
  return digitalRead(RIGHT_ENCODER_PIN);
}

//----------------------------------------------------------
void setup()
{
  // Setup the pins
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  Serial.begin(9600);
  Serial.println("Thy bidding?");
}

//----------------------------------------------------------

void setForce(Motor& m, int force)
{
  m.AttemptedForce = force;
}

void setForward(Motor& m)
{
  m.Dir = HIGH;
}

void setBackward(Motor& m)
{
  m.Dir = LOW;
}

void startCalibration(Motor& m)
{
  m.LastEncoder = m.Encoder;
}

void updateMotorEfficiency(Motor& m)
{
  if(m.AppliedForce == 0)
    return;

  int adv = m.Encoder - m.LastEncoder;
  if(adv > 0)
  {
    m.Efficiency = (float)adv / (float)m.AppliedForce;
  }
}

void updateEncoder(Motor& m, int val)
{
  if(val != m.LastEncoderValue)
  {
    m.LastEncoderValue = val;
    m.Encoder++;
  }
}

void applyForce(Motor& m, float ratio)
{
  m.AppliedForce = ratio < 1 ? m.AttemptedForce * ratio : m.AttemptedForce;
}

void updateMotorPins(Motor& m)
{
  analogWrite(m.SpeedPin, m.AppliedForce);
  digitalWrite(m.DirPin, m.Dir);
}

Motor LeftMotor = { 
  LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, 1.0 };
Motor RightMotor = { 
  RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, 1.0 };

void updateMotorPins()
{
  float l2r = LeftMotor.Efficiency / RightMotor.Efficiency;
  float r2l = RightMotor.Efficiency / LeftMotor.Efficiency;

  applyForce(LeftMotor, r2l);
  applyForce(RightMotor, l2r);

  updateMotorPins(LeftMotor);
  updateMotorPins(RightMotor);
}

void updateEncoders()
{
  int leftEncoderValue = getLeftEncoderValue();
  int rightEncoderValue = getRightEncoderValue();
  updateEncoder(LeftMotor, leftEncoderValue);
  updateEncoder(RightMotor, rightEncoderValue);
}

void setMove(int f, boolean forward)
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
  setForce(LeftMotor, f);
  setForce(RightMotor, f);
}

void setTurn(Direction dir, int force)
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
  setForce(LeftMotor, force);
  setForce(RightMotor, force);
}

void stop()
{
  setForce(LeftMotor, 0);
  setForce(RightMotor, 0);
  applyForce(LeftMotor, 1);
  applyForce(RightMotor, 1);
}

//----------------------------------------------------------

int MovementSpeed = 150;
int RotationSpeed = 150;

void processSerialInput()
{
  if(Serial.available() > 0)
  {
    int cmd = Serial.read();
    switch(cmd)
    {
    case 'a':
      setTurn(Left, RotationSpeed);
      break;
    case 'd':
      setTurn(Right, RotationSpeed);
      break;
    case 'w':
      setMove(MovementSpeed, true);
      break;
    case 's':
      setMove(MovementSpeed, false);
      break;
    case '[':
      MovementSpeed -=10;
      break;
    case ']':
      MovementSpeed += 10;
      break;
    case ';':
      RotationSpeed -= 10;
      break;
    case '\'':
      RotationSpeed += 10;
      break;
    case 'i':
      Serial.print("Attempted force: ");
      Serial.print(LeftMotor.AttemptedForce);
      Serial.print(", ");
      Serial.println(RightMotor.AttemptedForce);
      Serial.print("Applied force: ");
      Serial.print(LeftMotor.AppliedForce);
      Serial.print(", ");
      Serial.println(RightMotor.AppliedForce);    
      Serial.print("Wheel encoders: ");
      Serial.print(LeftMotor.Encoder);
      Serial.print("/");
      Serial.print(LeftMotor.LastEncoder);
      Serial.print(", ");
      Serial.print(RightMotor.Encoder);
      Serial.print("/");
      Serial.println(RightMotor.LastEncoder);
      Serial.print("Motors efficiency: ");
      Serial.print(LeftMotor.Efficiency);
      Serial.print(", ");
      Serial.println(RightMotor.Efficiency);
      Serial.print("Movement/rotation speed: ");
      Serial.print(MovementSpeed);
      Serial.print(", ");
      Serial.println(RotationSpeed);
      break;
    case 'r':
      LeftMotor.Efficiency = 1.0;
      RightMotor.Efficiency = 1.0;
    case 'c':
      MotorCalibrationProgress = StartCalibration;
      break;
    default: // PANIC!
      stop();
    }
  }
}

void loop()
{
  int t = millis();

  updateEncoders();

  if(MotorCalibrationProgress == StartCalibration)
  {
    if(LeftMotor.AttemptedForce == RightMotor.AttemptedForce
      && LeftMotor.AttemptedForce != 0)
    {
      startCalibration(LeftMotor);
      startCalibration(RightMotor);
      MotorCalibrationProgress = CalibrationStarted;
      MotorCalibrationStart = t;
      Serial.println("Calibrating motors...");
    }
    else
    {
      Serial.println("Motors at different speed, can't calibrate");
      MotorCalibrationProgress = CalibrationFinished;
    }
  }
  if(MotorCalibrationProgress == CalibrationStarted && t - MotorCalibrationStart > MOTOR_CALIBRATION_TIME)
  {
    updateMotorEfficiency(LeftMotor);
    updateMotorEfficiency(RightMotor);
    MotorCalibrationProgress = CalibrationFinished;
    Serial.println("Motor calibration done.");
  }
  if(LeftMotor.AttemptedForce != RightMotor.AttemptedForce)
  {
    Serial.println("Calibration interrupted");
    MotorCalibrationProgress = CalibrationFinished;
  }
    
  processSerialInput();
  updateMotorPins();
}

