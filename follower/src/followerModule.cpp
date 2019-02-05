#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <math.h>

#include "FollowerModule.h"
#include "Person3DRetriver.h"


#include "../../../yarp/src/libYARP_OS/include/yarp/os/Bottle.h"
using namespace std;
using namespace yarp::os;


//------------------------ buffer helper test ---------------------

double FollowerModule::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}


// This is our main function. Will be called periodically every getPeriod() seconds
bool FollowerModule::updateModule()
{
    Target_t targetpoint({0,0,0}, false);


    switch(m_targetType)
    {
        case FollowerTargetType::person:
        { targetpoint= (dynamic_cast<Person3DPPointRetriver*>(m_pointRetriver_ptr))->getTarget(); break;}

        case FollowerTargetType::redball:
        { targetpoint= (dynamic_cast<Ball3DPPointRetriver*>(m_pointRetriver_ptr))->getTarget(); break;}

        default: break;
    };

    m_follower.followTarget(targetpoint);

    return true;
}




// Message handler. Just echo all received messages.
bool FollowerModule::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    yError() << "rpc command=" <<command.toString();
    if (command.get(0).asString()=="help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("start");
        reply.addString("stop");
        reply.addString("verbose 0/1");
        return true;
    }
    else if(command.get(0).asString()=="start")
    {
        yError() << "he ricevuto start";
        m_follower.start();
        reply.addString("OK.started");
    }
    else if(command.get(0).asString()=="stop")
    {
        yError() << "he ricevuto stop";
        reply.addString("OK.stopped");
        m_follower.stop();
    }
    else
    {
        reply.addString("you");
        reply.addString("said");
        reply.append(command);
    }
    m_rpcPort.reply(reply);
    return true;

}



// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool FollowerModule::configure(yarp::os::ResourceFinder &rf)
{

    // 1) configure the follower
    if(!m_follower.configure(rf))
    {
        yError() << "Error reading configuration file";
        return false;
    }

    m_targetType=m_follower.getTargetType();

    // 2) read period and input port of this module from config file
    std::string inputPortName;
    Bottle config_group = rf.findGroup("GENERAL");
    if (config_group.isNull())
    {
        yError() << "Missing GENERAL group! the module uses default value!";
    }
    else
    {
        if (config_group.check("period"))  { m_period = config_group.find("period").asBool(); }
        if (config_group.check("inputPort"))  {inputPortName = config_group.find("inputPort").asString(); }
    }

    // 3) initialize the target retriever
    if(m_targetType == FollowerTargetType::redball)
    {
        m_pointRetriver_ptr = new Ball3DPPointRetriver();
    }
    else //person or default
    {
        m_pointRetriver_ptr = new Person3DPPointRetriver();
    }


    if(! m_pointRetriver_ptr->init("/follower/" + inputPortName +":i"))
    {
        yError() << "Error in initializing the Target Retriever";
        return false;
    }

    m_rpcPort.open("/follower/rpc");
    attach(m_rpcPort);

    return true;
}
// Interrupt function.
bool FollowerModule::interruptModule()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();

    if(m_pointRetriver_ptr!=nullptr)
        m_pointRetriver_ptr->deinit();

    m_follower.close();
    return true;
}
// Close function, to perform cleanup.
bool FollowerModule::close()
{

    if(m_pointRetriver_ptr!=nullptr)
        m_pointRetriver_ptr->deinit();

    delete m_pointRetriver_ptr;
    m_pointRetriver_ptr = nullptr;

    m_follower.close();

    return true;
}


FollowerModule::FollowerModule():m_period(m_defaultPeriod), m_targetType(FollowerTargetType::person)
{}
FollowerModule::~FollowerModule(){;}


//------------------------------------------------
// private function
//------------------------------------------------
