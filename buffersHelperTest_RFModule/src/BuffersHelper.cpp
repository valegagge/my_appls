/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "BuffersHelper.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;

template <typename T>
BuffersHelper<T>::BuffersHelper(int joints_num, size_t initial_size)
{
    nj = joints_num;
    for (size_t i = 0; i < initial_size; i++)
    {
        T* buff = new T [joints_num];
        m_buffers.push_back(buff);
    }
    m_usedBuffers.resize(initial_size, false);
}

template <typename T>
T* BuffersHelper<T>::get_buffer(void)
{
    m_mutex.lock();
    //get fisrt  free buffer
    int i;
    for(i=0; i< m_buffers.size(); i++)
        if(false == m_usedBuffers[i])
            break;

    T* buff;
    //if all buffers are used, I create new one and return it
    if(i>=m_buffers.size())
    {
        buff = new T[nj];
        if(nullptr == buff)
        {
            yError() << "no more memory!!";
            //todo Lancia una ecezzione
        }
        m_buffers.push_back(buff);
        m_usedBuffers.push_back(true);
        yDebug() << "I need to create a new buffer. Now size is " << m_buffers.size();
    }
    else //use the first free buffer
    {
        buff = m_buffers[i];
        m_usedBuffers[i] = true;
    }

    m_mutex.unlock();
    return buff;
}
template <typename T>
void BuffersHelper<T>::release_buffer(T* buff)
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
std::size_t BuffersHelper<T>::get_buffers_size(void)
{
    return nj;
}


template class BuffersHelper <int>;