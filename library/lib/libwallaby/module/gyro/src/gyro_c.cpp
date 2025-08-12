#include "kipr/gyro/gyro.h"
#include "gyro_p.hpp"

using namespace kipr;
using namespace kipr::gyro;

float gyro_x()
{
	return gyro::gyro_x();
}

float gyro_y()
{
	return gyro::gyro_y();
}

float gyro_z()
{
	return gyro::gyro_z();
}

int gyro_calibrate()
{
	return (gyro::gyro_calibrate() ? 0 : -1);
}
