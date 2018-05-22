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
class BuffersHelper
{
private:
    yarp::os::Mutex m_mutex;
    std::vector<T *> m_buffers;
    std::vector<bool> m_usedBuffers;
    std::size_t nj;

public:
    explicit BuffersHelper(int joints_num, std::size_t initial_size=3);
    ~BuffersHelper();
    T* get_buffer(void);
    std::size_t get_buffers_size(void);
    void release_buffer(T* buff);
    void printBuffers(void);
};
