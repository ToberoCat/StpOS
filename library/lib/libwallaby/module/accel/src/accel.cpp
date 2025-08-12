#include "kipr/accel/accel.hpp"
#include "accel_p.hpp"

using namespace kipr;
using namespace kipr::accel;

float Acceleration::x()
{
  return accel_x();
}

float Acceleration::y()
{
  return accel_y();
}

float Acceleration::z()
{
  return accel_z();
}

bool Acceleration::calibrate()
{
  return accel_calibrate();
}

float AccelX::value() const
{
  return accel_x();
}

float AccelY::value() const
{
  return accel_y();
}

float AccelZ::value() const
{
  return accel_z();
}
