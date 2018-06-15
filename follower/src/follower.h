

#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

class Follower:public yarp::os::RFModule
{
public:

    Follower();
    ~Follower();

    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);
    // Interrupt function.
    bool interruptModule();

    bool close();
private:

    yarp::os::IFrameTransform* m_transformClient;
    yarp::dev::PolyDriver      m_driver;


    yarp::os::BufferedPort<yarp::os::Bottle>  m_port_commands_output;

    void sendOutput();
    bool initTransformClient(void);
    bool get


};