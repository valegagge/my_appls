#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/Port.h>

#include <stdio.h>
using namespace yarp::os;

#define CONNECTION_TIMEOUT     180.0

bool exists(std::string &targetport)
{
    ContactStyle style;
    style.quiet = true;
    style.timeout = CONNECTION_TIMEOUT;
    return NetworkBase::exists(targetport, style);
}


int main(int argc, char *argv[])
{
    Network yarp;
    std::string port_robotIntarface="/icub/yarprobotinterface";
    double timeout=60.0;
    std::string request="is_ready";
    std::string reply="[ok]";
    const std::string logName = "checkRobotInterface: ";

    // 1. opening the port
    yarp::os::Port port;
    //port.setTimeout((float)((timeout>0.0) ? timeout : CONNECTION_TIMEOUT));
    while(!port.open("..."))
    {
        yError() << logName << "Error opening temp port";
        SystemClock::delaySystem(1.0);
    }

    ContactStyle style;
    style.quiet = true;
    style.timeout = (timeout>0.0) ? timeout : CONNECTION_TIMEOUT;
    bool ret;
    bool isConnected=false;
    bool responseIsOk=false;

    yInfo() << logName << "starting";
    double startTime = SystemClock::nowSystem();
    // 2. loop to check if yarprobot interface is running
    while(SystemClock::nowSystem() <= startTime+timeout)
    {
        if(!isConnected)
        {
            ret = NetworkBase::connect(port.getName(), port_robotIntarface, style);
            if(ret)
            {
                isConnected = true;
                yInfo() << logName << "Connected";
            }
        }
        else //already connected
        {
            Bottle msg, response;
            msg.fromString(request);
            ret = port.write(msg, response);
            if(ret && response.size()>0)
            {
                if(response.toString() == reply)
                {
                    responseIsOk = true;
                    yInfo() << logName << "Get response OK!!!! " << response.toString();
                    break;
                }
                else
                {
                    yInfo() << logName << "Get response but not equal " << response.toString();
                    responseIsOk=false;
                }
            }
            else
            {
                responseIsOk=false;
            }
        }
        SystemClock::delaySystem(1.0);
    }//end while

    if(responseIsOk)
        return 0;
    else
        return -1;

}
