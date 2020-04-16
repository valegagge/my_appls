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

#include "tester.h"

using namespace std;
using namespace yarp::os;


double Tester::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool Tester::updateModule()
{

    switch (count)
    {
        case 0: yError() << "I'm the yarpLogger TESTER!!!!";  count ++;break;
    case 1: yWarning() << "I'm the yarpLogger TESTER!!!!"; count ++;break;
    case 2: yDebug() << "I'm the yarpLogger TESTER!!!!"; count ++; break;
        case 3: yInfo() << "I'm the yarpLogger TESTER!!!!"; count ++; break;
    case 4: yTrace() << "I'm the yarpLogger TESTER!!!!"; count =0; break;
    }

    return true;
}
// Message handler. Just echo all received messages.
bool Tester::respond(const Bottle& command, Bottle& reply)
{
//    reply.clear();
//    if(command.get(0).asString()=="help")
//    {
//        reply.addVocab(Vocab::encode("many"));
//        reply.addString("Available commands are:");
//        reply.addString("move <linear_velocity>");
//    }
//    else if(command.get(0).asString()=="move")
//    {

//        reply.addString("OK.move" + command.get(1).asString());
//    }
//    else
//    {
//        reply.addString("you");
//        reply.addString("said: ");
//        reply.append(command);
//    }
//    m_rpcPort.reply(reply);
    return true;
}

// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool Tester::configure(yarp::os::ResourceFinder &rf)
{
    count=0;
    return true;
}

// Interrupt function.
bool Tester::interruptModule()
{

    return true;
}
// Close function, to perform cleanup.
bool Tester::close()
{
    // optional, close port explicitly
    yInfo() << "Calling close function\n";
    return true;
}


Tester::Tester():count(0)
{;}
Tester::~Tester(){;}


