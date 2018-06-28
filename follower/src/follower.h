

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

    yarp::dev::IFrameTransform* m_transformClient;
    yarp::dev::PolyDriver      m_driver;

#if 0
    // correct but maybe .. i try to invert (as suggested by silvio)
    const std::string target_frame_id = "base_link"; //"head_leopard_right";
    const std::string source_frame_id = "head_leopard_left";
#else
    // marco.accame: ok, it works.
    // conclusion: there is a bug in the methods. it is used an inverse matrix.
    const std::string target_frame_id = "head_leopard_left";
    const std::string source_frame_id = "base_link";
#endif
    yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort; //From this port I receive the data from Pf3dtraker

    yarp::os::BufferedPort<yarp::os::Bottle>  m_port_commands_output;//test only!!!used in sendOutput
    void sendOutput(); //only for test. it simulates joystick

    void followBall(); //core function call in updateModule.

    //get transform matrix from left camera to mobile base. Pf3dtraker use the left camera.
    bool getMatrix(yarp::sig::Matrix &transform);

    bool getBallPointTrasformed(yarp::sig::Vector &pointBallInput, yarp::sig::Vector &pointBallOutput);


    bool initTransformClient(void);


};