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

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Trip {

public:
    Trip(const std::string& routeId = "", const std::string& serviceId = "", const std::string& tripId = "", const std::string& name = "") :
        routeId(routeId),
        serviceId(serviceId),
        tripId(tripId),
        name(name) {
    }
    Trip(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        return (!routeId.empty()) && (!tripId.empty()) && (!tripId.empty());
    }

    friend std::ostream& operator<<(std::ostream& out, const Trip& t) {
        return out << "Trip{" << t.routeId << ", " << t.serviceId << ", " << t.tripId << ", " << t.name << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(routeId, serviceId, tripId, name);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(routeId, serviceId, tripId, name);
    }

public:
    std::string routeId{""};
    std::string serviceId{""};
    std::string tripId{""};
    std::string name{""};

};

}
