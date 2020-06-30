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

#include <map>

#include "../../Helpers/Types.h"
#include "../../Helpers/Ranges/SimultaneousRange.h"

template<typename KEY, typename VALUE>
class Map : public std::map<KEY, VALUE> {

public:
    using Key = KEY;
    using Value = VALUE;
    using Type = Map<Key, Value>;

private:
    using Super = std::map<Key, Value>;

public:
    inline bool contains(const Key& key) const noexcept {
        return Super::count(key) == 1;
    }

    inline void insert(const Key& key, const Value& value) noexcept {
        Super::operator[](key) = value;
    }

};

template<typename VALUE, bool RESIZEABLE = false, typename KEY_TYPE = size_t>
class IndexedMap {

public:
    using Value = VALUE;
    static constexpr bool Resizeable = RESIZEABLE;
    using KeyType = KEY_TYPE;
    using Type = IndexedMap<Value, Resizeable, KeyType>;

    inline static constexpr size_t NotContained = std::numeric_limits<size_t>::max();
    using Iterator = typename std::vector<Value>::const_iterator;

public:
    IndexedMap(const size_t initialCapacity) :
        indices(initialCapacity, NotContained) {
    }

    inline const std::vector<KeyType>& getKeys() const noexcept {
        return keys;
    }

    inline const std::vector<Value>& getValues() const noexcept {
        return values;
    }

    inline SimultaneousRange<std::vector<KeyType>, std::vector<Value>> map() const noexcept {
        return SimultaneousRange<std::vector<KeyType>, std::vector<Value>>(keys, values);
    }

    inline Iterator begin() const noexcept {
        return values.begin();
    }

    inline Iterator end() const noexcept {
        return values.end();
    }

    inline size_t size() const noexcept {
        return values.size();
    }

    inline bool empty() const noexcept {
        return values.empty();
    }

    inline size_t capacity() const noexcept {
        return indices.size();
    }

    inline bool contains(const KeyType key) noexcept {
        if (Resizeable) {
            if (key >= capacity()) indices.resize(key + 1, NotContained);
        } else {
            AssertMsg(key < capacity(), "Key " << key << " is out of range!");
        }
        return indices[key] != NotContained;
    }

    inline const Value& operator[](const KeyType key) const noexcept {
        AssertMsg(key < capacity(), "No value for key " << key << " contained!");
        return values[indices[key]];
    }

    inline Value& operator[](const KeyType key) noexcept {
        AssertMsg(contains(key), "No value for key " << key << " contained!");
        return values[indices[key]];
    }

    inline bool insert(const KeyType key, const Value& value = Value()) noexcept {
        if (contains(key)) {
            values[indices[key]] = value;
            return false;
        } else {
            indices[key] = keys.size();
            keys.emplace_back(key);
            values.emplace_back(value);
            return true;
        }
    }

    inline bool remove(const KeyType key) noexcept {
        if (!contains(key)) return false;
        keys[indices[key]] = keys.back();
        values[indices[key]] = values.back();
        indices[keys.back()] = indices[key];
        indices[key] = NotContained;
        keys.pop_back();
        values.pop_back();
        return true;
    }

    inline void clear() noexcept {
        for (const KeyType key : keys) {
            indices[key] = NotContained;
        }
        keys.clear();
        values.clear();
    }

private:
    std::vector<size_t> indices;
    std::vector<KeyType> keys;
    std::vector<Value> values;

};
