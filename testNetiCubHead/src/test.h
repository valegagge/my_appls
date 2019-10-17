// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/*!
 *  \brief     In here there is a simple example how to use a RFModule
 *  \author    Valentina Gaggero
 *  \date      October 2019
 *  \copyright GPL-2+ license
 */

#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>

class TestNetiCubHead:public yarp::os::RFModule
{
public:
    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();
private:


    std::string m_portPrefix = "/testnet";
    double m_period = 0.5; //seconds
    bool m_started = true;
    yarp::dev::PolyDriver        m_controlBoard;
    yarp::dev::IPidControl       *m_ipid = nullptr;

    yarp::os::Port m_rpcPort;
};
