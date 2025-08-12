//
// Created by tobias on 1/12/25.
//

#include "libstp/device/omni_wheeled/datatype/omni_wheeled_functions.h"

#include "libstp/device/omni_wheeled/datatype/omni_wheeld_conditions.h"

libstp::datatype::ConditionalFunction libstp::device::omni_wheeled::datatype::forForwardDistance(
    const float& distanceCm)
{
    return [distanceCm](bool)
    {
        return std::make_shared<ForwardDistanceConditionalResult>(distanceCm);
    };
}

libstp::datatype::ConditionalFunction libstp::device::omni_wheeled::datatype::forSideDistance(const float& distanceCm)
{
    return [distanceCm](bool)
    {
        return std::make_shared<SideDistanceConditionalResult>(distanceCm);
    };
}
