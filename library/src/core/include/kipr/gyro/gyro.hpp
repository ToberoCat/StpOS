#ifndef _KIPR_GYRO_GYRO_HPP_
#define _KIPR_GYRO_GYRO_HPP_

#include "kipr/sensor/sensor.hpp"

namespace kipr
{
  namespace gyro
  {
    class Gyro
    {
    public:
      static float x();
      static float y();
      static float z();
      static bool calibrate();

    private:
    };

    class GyroX : public sensor::Sensor<float>
    {
    public:
      virtual float value() const;
    };

    class GyroY : public sensor::Sensor<float>
    {
    public:
      virtual float value() const;
    };

    class GyroZ : public sensor::Sensor<float>
    {
    public:
      virtual float value() const;
    };
  }
}

#endif
