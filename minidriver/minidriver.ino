
// http://blog.dawnrobotics.co.uk/2013/11/getting-started-with-the-dagu-arduino-mini-driver-board/
// http://letsmakerobots.com/node/38636

#include "motor.h"
#include "IMU.h"

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
  return 200;// pow(ticktime / 41607161.2536758, -0.7039581);
}

int rightPwm(unsigned long ticktime)
{
  return 200;//pow(ticktime / 42061109.0080257, -0.7066563);
}

Motor LeftMotor = { 
  LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN };
Motor RightMotor = { 
  RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN };

Motor *const Motors[2] = { &LeftMotor, &RightMotor };

volatile boolean leftTick = false;

boolean debounce(unsigned long& t)
{
  if(micros() - t > 10000) // tick time at 255pwm was around 17millis so this should be safe
  {
    t = micros();
    return false;
  }
  return true;
}

void leftEncoderISR()
{
  static unsigned long t;
  if(debounce(t))
    return;
  encoderTick(LeftMotor);
  leftTick = true;
}

volatile boolean rightTick = false;

void rightEncoderISR()
{
  static unsigned long t;
  if(debounce(t))
    return;
  encoderTick(RightMotor);
  rightTick = true;
}

void setSpeed(unsigned long speed)
{
  setPwm(LeftMotor, leftPwm(speed));
  setPwm(RightMotor, rightPwm(speed));
}

void setPwm(int leftPwm, int rightPwm)
{
  setPwm(LeftMotor, leftPwm);
  setPwm(RightMotor, rightPwm);
}

void setMove(boolean forward)
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
int LeftPwm = 200;
int RightPwm = 200;

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
      setMove(true);
      setSpeed(MovementSpeed);
      break;
    case 's':
      setMove(false);
      setSpeed(MovementSpeed);
      break;
    case 'W':
      setMove(true);
      setPwm(LeftPwm, RightPwm);
      break;
    case 'S':
      setMove(false);
      setPwm(LeftPwm, RightPwm);
      break;
    case '-':
      LeftPwm++;
      break;
    case '_':
      LeftPwm--;
      break;
    case '=':
      RightPwm++;
      break;
    case '+':
      RightPwm--;
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

    default: // PANIC!
      stop();
    }
  }
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      ;//TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    ;//TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
void setup()
{
  // Setup the pins
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
 
  setPwmFrequency(9, 8);
 
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  
  attachInterrupt(0, leftEncoderISR, RISING);
  attachInterrupt(1, rightEncoderISR, RISING);
  
  IMU_init();
  
  Serial.begin(115200);
  Serial.write("Thy bidding?\n", 13);
}

void sendData()
{
  float x, y, z;
  IMU_readGyro(x, y, z);
  Serial.write((char*)&x, 4);
  Serial.write((char*)&y, 4);
  Serial.write((char*)&z, 4);
}

void loop()
{
  processSerialInput();
  sendData();
}
