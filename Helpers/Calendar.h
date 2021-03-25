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

#include "String/String.h"
#include "Ranges/Range.h"

const std::string dayName[7]{"sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday"};

const Range<int> week{0, 7};

inline int stringToDay(const std::string& time) {
    if (time.size() != 8) error("The string " + time + " is not in the format YYYYMMDD");
    int year = String::lexicalCast<int>(time.substr(0, 4)) - 1900;
    int month = String::lexicalCast<int>(time.substr(4, 2)) - 1;
    int day = String::lexicalCast<int>(time.substr(6, 2));
    std::tm t = {0,0,12,day,month,year, 0, 0, 0, 0, 0};
    time_t seconds = std::mktime(&t);
    return (seconds < 0) ? (seconds / (60 * 60 * 24)) - 1 : (seconds / (60 * 60 * 24));
}

inline std::string dayToString(const int day) {
    time_t seconds = (((time_t)day * 24) + 12) * 60 * 60;
    std::tm t = *std::localtime(&seconds);
    std::string result = std::to_string(t.tm_mday);
    while (result.length() < 2) result = "0" + result;
    result = std::to_string(t.tm_mon + 1) + result;
    while (result.length() < 4) result = "0" + result;
    result = std::to_string(t.tm_year + 1900) + result;
    while (result.length() < 8) result = "0" + result;
    return result;
}

inline int weekday(const int day) {
    time_t seconds = (((time_t)day * 24) + 12) * 60 * 60;
    std::tm t = *std::localtime(&seconds);
    return t.tm_wday;
}
