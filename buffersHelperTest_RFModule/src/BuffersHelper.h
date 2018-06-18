/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/Mutex.h>
#include <vector>

template <typename T>
class BuffersHelper;

template <typename T>
class Buffer
{
friend class BuffersHelper<T>;

private:
    uint32_t key;
    T* dataPtr;
    uint32_t numOfElements;
    //uint32_t numOfByte;//serve?
public:
    Buffer():key(0), dataPtr(nullptr), numOfElements(0){;}
    uint32_t getKey() {return key;}
    T* getData() {return dataPtr;}
    uint32_t getSize() {return numOfElements;}
};





template <typename T>
class BuffersHelper
{
private:
    yarp::os::Mutex m_mutex;
    std::vector<T*> m_buffers;
    std::vector<bool> m_usedBuffers;
    std::size_t m_numElem;
    uint32_t m_firstFreeBuff; //euristic

    inline bool searchFirstFreeBuffer(uint32_t &index)
    {
        for(uint32_t i=0; i< m_buffers.size(); i++)
            if(false == m_usedBuffers[i])
            {
                index = i;
                return true;
            }

        return false;
    }

public:
    explicit BuffersHelper(int numOfElements, std::size_t initialNumOfBuffers=3);
    ~BuffersHelper();
    T* getBuffer(Buffer<T> &b);
    std::size_t getBufferSize(void);
    void releaseBuffer(T* buff);
    void releaseBuffer(Buffer<T> &b);
    void printBuffers(void);
};

#include "BuffersHelper-inl.h"
