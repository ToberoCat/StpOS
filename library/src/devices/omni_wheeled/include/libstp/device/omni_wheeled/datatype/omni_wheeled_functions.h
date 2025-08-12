//
// Created by tobias on 1/12/25.
//

#pragma once
#include "libstp/datatype/functions.h"

namespace libstp::device::omni_wheeled::datatype {
    libstp::datatype::ConditionalFunction forForwardDistance(const float& distanceCm);
    libstp::datatype::ConditionalFunction forSideDistance(const float& distanceCm);
}
