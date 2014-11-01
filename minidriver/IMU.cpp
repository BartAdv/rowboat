// https://github.com/pololu/minimu-9-ahrs-arduino

#include "IMU.h"
#include "Wire.h"

L3G Gyro;
LSM303 Compass;

void IMU_init()
{
  Wire.begin();
  
  Gyro.init();
  Gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
  Gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void IMU_readGyro(float& x, float& y, float& z)
{
  Gyro.read();

  x = Gyro.g.x;
  y = Gyro.g.y;
  z = Gyro.g.z;
}
