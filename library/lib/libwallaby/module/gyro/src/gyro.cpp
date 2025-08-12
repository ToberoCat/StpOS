#include "kipr/gyro/gyro.hpp"
#include "gyro_p.hpp"

using namespace kipr;
using namespace kipr::gyro;

float Gyro::x()
{
	return gyro_x();
}

float Gyro::y()
{
	return gyro_y();
}

float Gyro::z()
{
	return gyro_z();
}

bool Gyro::calibrate()
{
	return gyro_calibrate();
}
float GyroX::value() const
{
	return gyro_x();
}

float GyroY::value() const
{
	return gyro_y();
}

float GyroZ::value() const
{
	return gyro_z();
}
