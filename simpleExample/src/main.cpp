#include "SimpleExample.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>



using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    Network yarp;
    /* create your module */
    SimpleExample module;
    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);
    yDebug() << "Configuring and starting module. \n";
    module.runModule(rf);   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    yDebug()<<"Main returning...";
    return 0;
}
