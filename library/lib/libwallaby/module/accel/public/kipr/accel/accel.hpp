#ifndef _KIPR_ACCEL_ACCEL_HPP_
#define _KIPR_ACCEL_ACCEL_HPP_

#include "kipr/sensor/sensor.hpp"

namespace kipr
{
  namespace accel
  {
    class Acceleration
    {
    public:
      static float x();
      static float y();
      static float z();
      static bool calibrate();

    private:
    };

    class AccelX : public sensor::Sensor<float>
    {
    public:
      virtual float value() const;
    };

    class AccelY : public sensor::Sensor<float>
    {
    public:
      virtual float value() const;
    };

    class AccelZ : public sensor::Sensor<float>
    {
    public:
      virtual float value() const;
    };
  }
}

#endif
