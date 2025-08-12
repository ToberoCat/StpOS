//
// Created by tobias on 12/27/24.
//

#include "libstp/datatype/speed.h"

libstp::datatype::Speed libstp::datatype::Speed::Slowest = Speed(0.1f, 0.1f, 0.1f);
libstp::datatype::Speed libstp::datatype::Speed::Slow = Speed(0.2f, 0.2f, 0.2f);
libstp::datatype::Speed libstp::datatype::Speed::Medium = Speed(0.5f, 0.5f, 0.45f);
libstp::datatype::Speed libstp::datatype::Speed::Fast = Speed(0.95f, 0.9f, 0.6f);
libstp::datatype::Speed libstp::datatype::Speed::Fastest = Speed(1.0f, 1.0f, 1.0f);
