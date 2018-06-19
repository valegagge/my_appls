/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;

template <typename T>
BuffersHelper<T>::BuffersHelper(int numOfElements, std::size_t initialNumOfBuffers)
{
    m_numElem = numOfElements;
    m_buffers.resize(0);

    for (size_t i = 0; i < initialNumOfBuffers; i++)
    {
        T* buff = new T [numOfElements];
        m_buffers.push_back(buff);
    }
    m_usedBuffers.resize(initialNumOfBuffers, false);
    m_firstFreeBuff = 0;
}




template <typename T>
T* BuffersHelper<T>::getBuffer(Buffer<T> &b)
{
    m_mutex.lock();
    //get fisrt  free buffer
    uint32_t i;
    T* buff;
    bool needNewBuff = false;
    if(false == m_usedBuffers[m_firstFreeBuff])
    {
        //you are lucky
        i = m_firstFreeBuff;
    }
    else
    {
        //needNewBuff = searchFirstFreeBuffer(i);
        for(i=0; i< m_buffers.size(); i++)
        {
            if(false == m_usedBuffers[i])
            {
                break;
            }
        }
        if(i>=m_buffers.size())
        {
            needNewBuff = true;
        }

    }

    //if all buffers are used, I create new one and return it
    if(needNewBuff)
    {
        buff = new T[m_numElem];
        if(nullptr == buff)
        {
            yError() << "no more memory!!";
            //todo Lancia una ecezzione
        }
        m_buffers.push_back(buff);
        m_usedBuffers.push_back(true);
        yError() << "I need to create a new buffer. Now size is " << m_buffers.size() << "pointer is " << buff;
        i = m_buffers.size()-1;
    }
    else //use the first free buffer
    {
        buff = m_buffers[i];
        m_usedBuffers[i] = true;

    }
    b.key=i;
    b.dataPtr=buff;
    b.numOfElements = m_numElem;
    //yInfo() << "getBuffer: key=" << b.key << " ptr=" << b.dataPtr;
    m_mutex.unlock();
    return buff;
}


template <typename T>
void BuffersHelper<T>::releaseBuffer(T* buff)
{
    m_mutex.lock();
    int i;
    for(i=0; i< m_buffers.size(); i++)
    {
        if(m_buffers[i] == buff)
        {
            m_usedBuffers[i] = false;
            break;
        }
    }
    if(i>=m_buffers.size())
    {
        yError() << "error in deallocation!!";
    }
    m_mutex.unlock();
}


template <typename T>
void BuffersHelper<T>::releaseBuffer(Buffer<T> &b)
{
    m_mutex.lock();

    if(b.key>=m_buffers.size())
    {
        yError() << "error in deallocation!!";
    }

    m_usedBuffers[b.key] = false;
    m_firstFreeBuff = b.key;
    //yInfo() << "ReleaseBuffer: key=" << b.key << " ptr=" << b.dataPtr;
    m_mutex.unlock();
}





template <typename T>
void BuffersHelper<T>::printBuffers(void)
{
    m_mutex.lock();
    for(int i=0; i<m_buffers.size(); i++)
        yDebug() << "buff["<< i<< "]: addr = " << m_buffers[i] << "; it is used?" << m_usedBuffers[i] ;

    m_mutex.unlock();
}

template <typename T>
BuffersHelper<T>::~BuffersHelper()
{
    for (size_t i = 0; i < m_buffers.size(); i++)
    {
        delete[] m_buffers[i];
    }
}

template <typename T>
std::size_t BuffersHelper<T>::getBufferSize(void)
{
    return m_numElem;
}

