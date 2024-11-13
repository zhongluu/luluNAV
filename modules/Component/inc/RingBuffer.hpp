/*
LULUNAV
Copyright (C) {{ 2024 }}  {{ yulu_zhong }}

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _RINGBUFFER_
#define _RINGBUFFER_

#include <cstddef> 
#include <vector>

template <typename T>
class RingBuffer
{
private:
    std::vector<T> _buf;
    size_t _capacity;
    size_t _front;
    size_t _rear;

public:
    explicit RingBuffer(size_t capacity);
    ~RingBuffer();
    bool popOut();
    bool pushIn(const T& val);
    bool popNOut(size_t nums);
    bool pushNIn(const T*& pVal, size_t nums);
    bool readN(T* pBuf, size_t nums);
    size_t curLen();
    bool isEmpty();
    bool isFull();
    T front();
};

#include "RingBuffer.tpp"

#endif