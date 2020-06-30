/**********************************************************************************

 Copyright (c) 2020 Tobias Zündorf

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

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class ReachedIndexSmall {

public:
    ReachedIndexSmall(const Data& data) :
        data(data),
        labels(data.numberOfTrips(), -1),
        defaultLabels(data.numberOfTrips(), -1) {
        for (const TripId trip : data.trips()) {
            if (data.numberOfStopsInTrip(trip) > 255) warning("Trip ", trip, " has ", data.numberOfStopsInTrip(trip), " stops!");
            defaultLabels[trip] = data.numberOfStopsInTrip(trip);
        }
    }

public:
    inline void clear() noexcept {
        labels = defaultLabels;
    }

    inline StopIndex operator()(const TripId trip) const noexcept {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        return StopIndex(labels[trip]);
    }

    inline bool alreadyReached(const TripId trip, const u_int8_t index) const noexcept {
        return labels[trip] <= index;
    }

    inline void update(const TripId trip, const StopIndex index) noexcept {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId i = trip; i < routeEnd; i++) {
            if (labels[i] <= index) break;
            labels[i] = index;
        }
    }

private:
    const Data& data;

    std::vector<u_int8_t> labels;

    std::vector<u_int8_t> defaultLabels;

};

}
