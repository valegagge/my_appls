
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Person3DRetriver.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "TargetRetriver.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


bool TargetRetriver::init(std::string inputPortName)
{
    if(! m_inputPort.open(inputPortName))
    {
        yError() << "Ball3DPPointRetriver:Error opening input port";
        return false;
    }
    return true;
}

bool TargetRetriver::deinit(void)
{
    m_inputPort.interrupt();
    m_inputPort.close();
    return true;
}


// Ball3DPPointRetriver::Ball3DPPointRetriver() {;}
// Ball3DPPointRetriver::~Ball3DPPointRetriver() {;}

Target_t Ball3DPPointRetriver::getTarget(void)
{
    std::vector<double> point3d = {0,0,0};

    Bottle *b = m_inputPort.read();

    //     yDebug() << "Vedo pallina " << b->get(6).asDouble();
    //     yDebug() << "pos pallina" << b->get(0).asDouble() << b->get(1).asDouble() << b->get(2).asDouble();
    bool ballIsTracked = (b->get(6).asDouble() == 1.0) ? true : false;

    if(!ballIsTracked)
    {
        yError() << "Ball3DPPointRetriver: I can't see the ball!!!";
        return std::make_pair(std::move (point3d), false);
    }

    point3d[0] = b->get(0).asDouble();
    point3d[1] = b->get(1).asDouble();
    point3d[2] = b->get(2).asDouble();

    return std::make_pair(std::move (point3d), true);
}
