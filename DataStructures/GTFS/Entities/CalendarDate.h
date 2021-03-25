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

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Calendar.h"
#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class CalendarDate {

public:
    CalendarDate(const std::string& serviceId = "", const int date = -1, const bool operates = true) :
        serviceId(serviceId),
        date(date),
        operates(operates) {
    }
    CalendarDate(const std::string& serviceId, const std::string& date, const bool operates = true) :
        serviceId(serviceId),
        date(stringToDay(date)),
        operates(operates) {
    }
    CalendarDate(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        return !serviceId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const CalendarDate& c) {
        return out << "CalendarDate{" << c.serviceId << ", " << c.date << ", " << c.operates << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(serviceId, date, operates);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(serviceId, date, operates);
    }

public:
    std::string serviceId{""};
    int date{-1};
    bool operates{true};

};

}
