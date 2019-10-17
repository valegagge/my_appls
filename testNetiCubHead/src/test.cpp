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
#include <yarp/dev/ControlBoardPid.h>

#include "test.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


double TestNetiCubHead::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool TestNetiCubHead::updateModule()
{
    if(!m_started)
        return true;
    Pid pid;
    bool ret = m_ipid->getPid(VOCAB_PIDTYPE_POSITION, 0, &pid);
    if(ret)
        yInfo() << "ok get pid";
    else
        yError() << "error getting pid";

    return true;
}
// Message handler. Just echo all received messages.
bool TestNetiCubHead::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    if(command.get(0).asString()=="help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("start");
        reply.addString("stop");
    }
    else if(command.get(0).asString()=="start")
    {
        m_started = true;
        reply.addString("OK.started");
    }
    else if(command.get(0).asString()=="stop")
    {
        m_started = false;
        reply.addString("OK.stopped");
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
bool TestNetiCubHead::configure(yarp::os::ResourceFinder &rf)
{
    std::string robotName="icub";

    // create the remote control board
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
    if(!m_controlBoard.view(m_ipid)  || m_ipid==nullptr)
    {
        yError() <<"Unable to open pid interface. Aborting";
        return false;
    }

    // open the rpc port
    if(!m_rpcPort.open(m_portPrefix + "/rpc"))
    {
        yError() << "Error opening rpc port";
        return false;
    }
    attach(m_rpcPort);

    return true;
}

// Interrupt function.
bool TestNetiCubHead::interruptModule()
{
    yInfo()<<"interrupt\n";
    m_rpcPort.interrupt();
    m_rpcPort.close();
    return true;
}
// Close function, to perform cleanup.
bool TestNetiCubHead::close()
{
    // optional, close port explicitly
    yInfo() << "Calling close function\n";
    return true;
}
