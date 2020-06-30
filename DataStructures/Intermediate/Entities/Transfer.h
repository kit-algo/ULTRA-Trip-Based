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

static_assert(false, "Deprecated!");

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace Intermediate {

class Transfer {

public:
    Transfer(const StopId fromStopId = noStop, const StopId toStopId = noStop, const int minTransferTime = 0) :
        fromStopId(fromStopId),
        toStopId(toStopId),
        minTransferTime(minTransferTime) {
    }
    Transfer(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool operator<(const Transfer& t) const noexcept {
        return (fromStopId < t.fromStopId) || ((fromStopId == t.fromStopId) && (
               (toStopId < t.toStopId)));
    }

    friend std::ostream& operator<<(std::ostream& out, const Transfer& t) {
        return out << "Transfer{" << t.fromStopId << ", " << t.toStopId << ", " << t.minTransferTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(fromStopId, toStopId, minTransferTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(fromStopId, toStopId, minTransferTime);
    }

public:
    StopId fromStopId{noStop};
    StopId toStopId{noStop};
    int minTransferTime{0};

};

}
