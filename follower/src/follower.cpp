#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>

#include "follower.h"
using namespace std;
using namespace yarp::os;


//------------------------ buffer helper test ---------------------

double Follower::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return 0.5;
}
void Follower::sendOutput()
{
    static yarp::os::Stamp stamp;

    //send data to baseControl module
    if (m_port_commands_output.getOutputCount() == 0)
    {
        //if I have not connection I don't send anything
        return;
    }
    stamp.update();
    Bottle &b = m_port_commands_output.prepare();
    m_port_commands_output.setEnvelope(stamp);
    b.clear();
    //like joystick
    b.addInt(3);//write cartesian speed
    b.addDouble(100);    // angle in deg
    b.addDouble(0);    // lin_vel in m/s
    b.addDouble(0);    // ang_vel in deg/s
    b.addDouble(100);

 /*
    b.addInt(2);                    // polar speed commands
    b.addDouble(m_control_out.linear_dir);    // angle in deg
    b.addDouble(m_control_out.linear_vel);    // lin_vel in m/s
    b.addDouble(m_control_out.angular_vel);    // ang_vel in deg/s
    b.addDouble(100);*/

    m_port_commands_output.write();
    }

// This is our main function. Will be called periodically every getPeriod() seconds
bool Follower::updateModule()
{
    sendOutput();
   return true;
}
// Message handler. Just echo all received messages.
bool Follower::respond(const Bottle& command, Bottle& reply)
{
    cout << "Got something, echo is on" << endl;
    if (command.get(0).asString() == "quit")
        return false;
    else
        reply = command;
    return true;
}





// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool Follower::configure(yarp::os::ResourceFinder &rf)
{
    m_port_commands_output.open("/follower/control:o");

    if(!initTransformClient())
        return false;


    return true;
}
// Interrupt function.
bool Follower::interruptModule()
{

    m_port_commands_output.interrupt();
    m_port_commands_output.close();
    return true;
}
// Close function, to perform cleanup.
bool Follower::close()
{

    m_driver.close();
    // optional, close port explicitly
    cout << "Calling close function\n";
    return true;
}


Follower::Follower():m_transformClient(nullptr)
{;}
Follower::~Follower(){;}


//------------------------------------------------
// private function
//------------------------------------------------

bool Follower::initTransformClient(void)
{
    // Prepare properties for the FrameTransformClient
    yarp::os::Property propTfClient;
    propTfClient.put("device", "transformClient");
    propTfClient.put("local", "/transformClient-follower");
    propTfClient.put("remote", "/transformServer");

    // Try to open the driver
    bool ok_open = m_driver.open(propTfClient);
    if (!ok_open)
    {
        yError() << "Unable to open the FrameTransformClient driver.";
        return false;
    }

    // Try to retrieve the view
    bool ok_view = m_driver.view(m_transformClient);
    if (!ok_view || m_transformClient == 0)
    {
        yError() << "Unable to retrieve the FrameTransformClient view.";
        return false;
    }

    //from now I can use m_transformClient
    return true;
}