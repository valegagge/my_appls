// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */


#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>

#include "SimpleExample.h"

using namespace std;
using namespace yarp::os;


double SimpleExample::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool SimpleExample::updateModule()
{

    //1) send command to the base control
    sendCommand2BaseControl(0.0, m_linear_vel, 0.0);


    //2) move up and down the arm

    if(m_posNotReached)
    {
        bool done;
        m_ipos->checkMotionDone(&done);
        if(done)
            m_posNotReached=false;

        return true;
    }

    if(m_goingup==false)
    {
        m_ipos->positionMove(m_positions_up);
        m_goingup=true;
        m_posNotReached=true;
    }
    else
    {
        m_ipos->positionMove(m_positions_down);
        m_goingup=false;
        m_posNotReached=true;
    }




    return true;
}
// Message handler. Just echo all received messages.
bool SimpleExample::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    if(command.get(0).asString()=="help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("move <linear_velocity>");
    }
    else if(command.get(0).asString()=="move")
    {
        m_linear_vel = command.get(1).asDouble();
        reply.addString("OK.move" + command.get(1).asString());
    }
    else
    {
        reply.addString("you");
        reply.addString("said: ");
        reply.append(command);
    }
    m_rpcPort.reply(reply);
    return true;
}

// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool SimpleExample::configure(yarp::os::ResourceFinder &rf)
{
    std::string robotName="SIM_CER_ROBOT";

    //1) read the configuration file
    Bottle config_group = rf.findGroup("GENERAL");
    if (config_group.isNull())
    {
        yWarning() << "Missing GENERAL group! the module uses default value!";
    }
    else
    {
        if (config_group.check("portPrefix")) { m_portPrefix = config_group.find("portPrefix").asString(); }
        if (config_group.check("period")) { m_period = config_group.find("period").asDouble(); }
        if (config_group.check("robotName")) { robotName = config_group.find("robotName").asString(); }
    }

    //2) create the remote control board
    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/" + robotName + "/left_arm");
    options.put("local",  m_portPrefix + "/mc");

    m_controlBoard.open(options);
    if(!m_controlBoard.isValid())
    {
        yError() <<"Unable to open device driver. Aborting";
        return false;
    }

    if(!m_controlBoard.view(m_ienc) || m_ienc==nullptr)
    {
        yError() <<"Unable to open encoders interface. Aborting";
        return false;
    }

    if(!m_controlBoard.view(m_ipos)  || m_ienc==nullptr)
    {
        yError() <<"Unable to open position interface. Aborting";
        return false;
    }
    //from now I can use the interfaces
    int nj=0;
    m_ipos->getAxes(&nj);
    if(nj != m_numOfJoints)
    {
        yError() <<"Different number of joints. Aborting";
        return false;
    }

    //3) open the rpc port
    if(!m_rpcPort.open(m_portPrefix + "/rpc"))
    {
        yError() << "Error opening rpc port";
        return false;
    }
    attach(m_rpcPort);

    //4) open the communication port toward baseController module
    if(!m_outputPort2baseCtr.open(m_portPrefix +"/cmd2BaseCtrl:o"))
    {
        yError() << "Error opening output port for base control";
        return false;
    }

    return true;
}

// Interrupt function.
bool SimpleExample::interruptModule()
{
    yInfo()<<"interrupt\n";
    m_rpcPort.interrupt();
    m_rpcPort.close();
    return true;
}
// Close function, to perform cleanup.
bool SimpleExample::close()
{
    // optional, close port explicitly
    yInfo() << "Calling close function\n";
    return true;
}


SimpleExample::SimpleExample():m_ienc(nullptr), m_ipos(nullptr)
{;}
SimpleExample::~SimpleExample(){;}


bool SimpleExample::sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity)
{
    static yarp::os::Stamp stamp;

    stamp.update();
    //send velocity commands to the base control
    if (m_outputPort2baseCtr.getOutputCount() > 0) //if I have connection
    {
        Bottle &b = m_outputPort2baseCtr.prepare();
        m_outputPort2baseCtr.setEnvelope(stamp);
        b.clear();
        b.addInt(2);                    // polar speed commands
        b.addDouble(linearDirection);    // angle in deg
        b.addDouble(linearVelocity);    // lin_vel in m/s
        b.addDouble(angularVelocity);    // ang_vel in deg/s
        b.addDouble(100);
        m_outputPort2baseCtr.write();
    }

    return true;

}
