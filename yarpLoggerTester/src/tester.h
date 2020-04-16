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
 *  \date      July 2020
 *  \copyright GPL-2+ license
 */

#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>

class Tester:public yarp::os::RFModule
{
public:
    Tester();
    ~Tester();

    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();
private:


    double m_period=5; //seconds

 int count =0;
};
