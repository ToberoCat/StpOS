#ifndef _KIPR_MAGNETO_MAGNETO_HPP_
#define _KIPR_MAGNETO_MAGNETO_HPP_

#include "kipr/sensor/sensor.hpp"

namespace kipr
{
	namespace magneto
	{
		class Magneto
		{
		public:
			static float x();
			static float y();
			static float z();
		};

		class MagnetoX : public sensor::Sensor<float>
		{
		public:
			virtual float value() const;
		};

		class MagnetoY : public sensor::Sensor<float>
		{
		public:
			virtual float value() const;
		};

		class MagnetoZ : public sensor::Sensor<float>
		{
		public:
			virtual float value() const;
		};
	}
}

#endif
