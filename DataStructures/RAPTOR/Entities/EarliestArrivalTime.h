/**********************************************************************************

 Copyright (c) 2020 Jonas Sauer

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Types.h"

namespace RAPTOR {

struct SeparatedEarliestArrivalTime {
    SeparatedEarliestArrivalTime() : arrivalTimeByRoute(never), arrivalTimeByTransfer(never) {}

    inline void setArrivalTimeByRoute(const int time) noexcept {
        arrivalTimeByRoute = time;
    }

    inline void setArrivalTimeByTransfer(const int time) noexcept {
        arrivalTimeByTransfer = time;
    }

    inline int getArrivalTimeByRoute() const noexcept {
        return arrivalTimeByRoute;
    }

    inline int getArrivalTimeByTransfer() const noexcept {
        return arrivalTimeByTransfer;
    }

    inline int getArrivalTime() const noexcept {
        return std::min(arrivalTimeByRoute, arrivalTimeByTransfer);
    }

    int arrivalTimeByRoute;
    int arrivalTimeByTransfer;
};

struct CombinedEarliestArrivalTime {
    CombinedEarliestArrivalTime() : arrivalTime(never) {}

    inline void setArrivalTimeByRoute(const int time) noexcept {
        arrivalTime = time;
    }

    inline void setArrivalTimeByTransfer(const int time) noexcept {
        arrivalTime = time;
    }

    inline int getArrivalTimeByRoute() const noexcept {
        return arrivalTime;
    }

    inline int getArrivalTimeByTransfer() const noexcept {
        return arrivalTime;
    }

    inline int getArrivalTime() const noexcept {
        return arrivalTime;
    }

    int arrivalTime;
};

template<bool USE_MIN_TRANSFER_TIMES>
using EarliestArrivalTime = Meta::IF<USE_MIN_TRANSFER_TIMES, SeparatedEarliestArrivalTime, CombinedEarliestArrivalTime>;
}
