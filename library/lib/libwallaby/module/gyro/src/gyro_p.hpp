#ifndef _KIPR_GYRO_GYRO_P_HPP_
#define _KIPR_GYRO_GYRO_P_HPP_

namespace kipr
{
  namespace gyro
  {
    float gyro_x();

    float gyro_y();

    float gyro_z();

    bool gyro_calibrate();

    bool gyro_calibrated();
  }
}

#endif
