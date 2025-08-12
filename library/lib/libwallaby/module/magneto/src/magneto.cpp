#include "kipr/magneto/magneto.hpp"
#include "magneto_p.hpp"

using namespace kipr;
using namespace kipr::magneto;

float Magneto::x()
{
	return magneto_x();
}

float Magneto::y()
{
	return magneto_y();
}

float Magneto::z()
{
	return magneto_z();
}

float MagnetoX::value() const
{
	return magneto_x();
}

float MagnetoY::value() const
{
	return magneto_y();
}

float MagnetoZ::value() const
{
	return magneto_z();
}
