//
// Created by tobias on 12/26/24.
//

#pragma once
#include <thread>

namespace libstp::utility
{
    // ToDo: Will this function work with python asyncio?
    inline void msleep(const int milliseconds = 10) {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

    inline long get_current_time_millis()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
}
