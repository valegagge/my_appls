

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

#ifdef TICK_SERVER
#include <tick_server.h>
#include <ReturnStatus.h>
#endif




#ifdef TICK_SERVER
class FollowerModule:public yarp::os::RFModule, public TickServer
#else
class FollowerModule : public RFModule
#endif
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

    #ifdef TICK_SERVER
    ReturnStatus request_tick(const std::string& params = "") override;
    ReturnStatus request_status() override;
    ReturnStatus request_halt() override;
    #endif

private:

    Follower m_follower;
    double m_period;
    double const m_defaultPeriod=0.01;

    FollowerTargetType         m_targetType;
    TargetRetriver*            m_pointRetriver_ptr;

    yarp::os::Port m_rpcPort;

};


