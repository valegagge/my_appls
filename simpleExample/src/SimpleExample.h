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
 *  \date      July 2019
 *  \copyright GPL-2+ license
 */

#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>

class SimpleExample:public yarp::os::RFModule
{
public:
    SimpleExample();
    ~SimpleExample();

    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();
private:


    std::string m_portPrefix="/simpleEx";
    double m_period=0.5; //seconds

    yarp::dev::PolyDriver        m_controlBoard;
    yarp::dev::IPositionControl  *m_ipos;
    yarp::dev::IEncoders         *m_ienc;
    const int m_numOfJoints=8;

    const double m_positions_up[8]={54.0, 34.0, 54.0, 44.0, 0.0, 0.0, 0.0, 0.0};
    const double m_positions_down[8]={0.0, 6.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0};
    bool m_goingup=false;
    bool m_posNotReached=true;



    double m_linear_vel=0.0;
    yarp::os::Port m_rpcPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPort2baseCtr;
    bool sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity);

};
