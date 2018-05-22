#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "buffHelpTest.h"
using namespace std;
using namespace yarp::os;


//------------------------ buffer helper test ---------------------

double BuffHelpTest::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return 0.5;
}
// This is our main function. Will be called periodically every getPeriod() seconds
bool BuffHelpTest::updateModule()
{
    bool ret = true;
    for(int i=0; i<NUM_THREADS; i++)
    {
        if(!threadList[i].isRunning())
        {
            yError()<< "thread "<< i << "is not running";
            ret = false;
        }
    }
    buffsHelper.printBuffers();
    return ret;
}
// Message handler. Just echo all received messages.
bool BuffHelpTest::respond(const Bottle& command, Bottle& reply)
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
bool BuffHelpTest::configure(yarp::os::ResourceFinder &rf)
{
    for(int i=0; i<NUM_THREADS; i++)
        threadList[i].start();

    return true;
}
// Interrupt function.
bool BuffHelpTest::interruptModule()
{
    cout << "Interrupted!!" << endl;
    for(int i=0; i<NUM_THREADS; i++)
        threadList[i].stop();
    yDebug() << "------------------------------------";
    buffsHelper.printBuffers();
    yDebug() << "------------------------------------";
    return true;
}
// Close function, to perform cleanup.
bool BuffHelpTest::close()
{
    // optional, close port explicitly
    cout << "Calling close function\n";
    return true;
}


BuffHelpTest::BuffHelpTest():buffsHelper(NUMJOINTS)
{
    for(int i=0; i<NUM_THREADS; i++)
        threadList[i].init(&buffsHelper, i);

    yDebug() << "------------------------------------";
    buffsHelper.printBuffers();
    yDebug() << "------------------------------------";
}
BuffHelpTest::~BuffHelpTest(){;}