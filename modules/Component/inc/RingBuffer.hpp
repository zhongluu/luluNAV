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