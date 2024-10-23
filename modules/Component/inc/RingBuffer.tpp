#ifndef _RINGBUFFER_TPP_
#define _RINGBUFFER_TPP_

#include "RingBuffer.hpp"
#include <iostream>

template <typename T>
RingBuffer<T>::RingBuffer(size_t capacity) : _capacity(capacity), _front(0), 
                                    _rear(0), _buf(capacity + 1)
{

}

template <typename T>
RingBuffer<T>::~RingBuffer()
{

}

template <typename T>
bool RingBuffer<T>::popOut()
{
    if (isEmpty())
    {
        return false;
    }

    _front = (_front + 1) % _capacity;
    return true;
}

template <typename T>
bool RingBuffer<T>::pushIn(const T& val)
{
    if (isFull())
    {
        return false;
    }
    
    _buf[_rear] = val;
    _rear = (_rear + 1) % _capacity;
    return true;
}

template <typename T>
bool RingBuffer<T>::popNOut(size_t nums)
{
    size_t len = curLen();
    if (len < nums)
    {
        return false;
    }
    
    _front =  (_front + nums) % _capacity;
    return true;
}

template <typename T>
bool RingBuffer<T>::pushNIn(const T*& pVal, size_t nums)
{

    if ((_front + _capacity - _rear) % _capacity < nums)
    {
        return false;
    }
    
    for (size_t i = 0; i < nums; i++)
    {
        _buf[((_rear + i) % _capacity)] = *(pVal + i);
    }
    
    _rear = (_rear + nums) % _capacity;

    return true;
}

template <typename T>
bool RingBuffer<T>::readN(T* pBuf, size_t nums)
{
    size_t len = curLen();
    if (len < nums)
    {
        return false;
    }

    for (size_t i = 0; i < nums; i++)
    {
        *(pBuf + i) = _buf[(_front + i) % _capacity];
    }
    return true;
}

template <typename T>
size_t RingBuffer<T>::curLen()
{
    return (_rear - _front + _capacity) % _capacity;
}

template <typename T>
bool RingBuffer<T>::isEmpty()
{
    if (_front == _rear)
    {
        return true;
    } else {
        return false;
    }
    
}

template <typename T>
bool RingBuffer<T>::isFull()
{
    if (((_rear + 1) % _capacity) == _front)
    {
        return true;
    } else {
        return false;
    }
}

template <typename T>
T RingBuffer<T>::front()
{
    if (isEmpty())
    {
        return T();
    }
    
    return _buf[_front];
}

#endif
