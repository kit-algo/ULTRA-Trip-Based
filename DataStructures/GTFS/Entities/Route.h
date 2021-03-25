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

#include "Vehicle.h"

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Route {

public:
    Route(const std::string& routeId = "", const std::string& agencyId = "", const std::string& name = "", const int type = -1, const std::string& routeColor = "FFFFFF", const std::string& textColor = "000000") :
        routeId(routeId),
        agencyId(agencyId),
        name(name),
        type(type),
        routeColor(routeColor),
        textColor(textColor) {
    }
    Route(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        if (!String::isColor(routeColor)) routeColor = "FFFFFF";
        if (!String::isColor(textColor)) textColor = "000000";
        return !routeId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const Route& r) {
        return out << "Route{" << r.routeId << ", " << r.agencyId << ", " << r.name << ", " << r.type << ", " << r.routeColor << ", " << r.textColor << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(routeId, agencyId, name, type, routeColor, textColor);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(routeId, agencyId, name, type, routeColor, textColor);
    }

public:
    std::string routeId{""};
    std::string agencyId{""};
    std::string name{""};
    int type{-1};
    std::string routeColor{"FFFFFF"};
    std::string textColor{"000000"};

};

}
