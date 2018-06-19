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
class FixedSizeBuffersManager;

template <typename T>
class Buffer
{
friend class FixedSizeBuffersManager<T>;

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
class FixedSizeBuffersManager
{
private:
    yarp::os::Mutex m_mutex;
    std::vector<T*> m_buffers;
    std::vector<bool> m_usedBuffers;
    std::size_t m_numElem;
    uint32_t m_firstFreeBuff; //euristic

    inline bool searchFirstFreeBuffer(uint32_t &index)
    {
        for(uint32_t p=0; p< m_buffers.size(); p++)
            if(false == m_usedBuffers[p])
            {
                index = p;
                return true;
            }

        return false;
    }

public:
    explicit FixedSizeBuffersManager(uint32_t numOfElements, std::size_t initialNumOfBuffers=3);
    ~FixedSizeBuffersManager();
    T* getBuffer(Buffer<T> &b);
    std::size_t getBufferSize(void);
    void releaseBuffer(T* buff);
    void releaseBuffer(Buffer<T> &b);
    void printBuffers(void);
};

#include "FixedSizeBuffersManager-inl.h"
