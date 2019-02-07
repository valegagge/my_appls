

#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcClient.h>

#include <string>

#include "TargetRetriver.h"
#include "follower.h"



class FollowerModule:public yarp::os::RFModule
{
public:

    FollowerModule();
    ~FollowerModule();

    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();
private:

    Follower m_follower;
    double m_period;
    double const m_defaultPeriod=0.01;

    FollowerTargetType         m_targetType;
    TargetRetriver*            m_pointRetriver_ptr;

    yarp::os::Port m_rpcPort;

};


