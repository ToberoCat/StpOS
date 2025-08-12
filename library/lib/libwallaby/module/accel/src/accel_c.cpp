#include "kipr/accel/accel.h"
#include "accel_p.hpp"

using namespace kipr;
using namespace kipr::accel;

float accel_x()
{
  return accel::accel_x();
}

float accel_y()
{
  return accel::accel_y();
}

float accel_z()
{
  return accel::accel_z();
}

int accel_calibrate()
{
  return accel::accel_calibrate();
}
