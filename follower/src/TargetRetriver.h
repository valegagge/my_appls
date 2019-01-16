
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file TargetRetriver.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef TARGETRETRIVER_H
#define TARGETRETRIVER_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <vector>
#include <utility>

using Target_t = std::pair<std::vector<double>, bool>;

class TargetRetriver
{
public:
    virtual Target_t getTarget(void)=0;
    bool init(std::string inputPortName);
    bool deinit(void);
protected:
    std::vector<double> m_target;
    yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_outputPort;//???is it necessary???
};

class Ball3DPPointRetriver : public TargetRetriver
{
public:
    Target_t getTarget(void);
//     Ball3DPPointRetriver();
//     ~Ball3DPPointRetriver();
private:
};

#endif
