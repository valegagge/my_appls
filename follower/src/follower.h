

#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Property.h>

#include <string>

#include "TargetRetriver.h"
#include "Person3DRetriver.h"

class FollowerConfig
{
public:
    double factorDist2Vel;
    double factorAng2Vel;
    std::string inputPortName;
    std::string outputPortName;
    double distanceThreshold = 0.8;
    double angleThreshold = 3.0;

    FollowerConfig()
    {
        //init with default values
        factorDist2Vel = 0.8;
        factorAng2Vel = 0.8;
        inputPortName = "ballPoint";
        outputPortName = "commands";
        distanceThreshold = 0.8;
        angleThreshold = 3.0;
    }

    void print(void)
    {
        yInfo() << "The follower module has been configure with following values";
        yInfo() << "factorDist2Vel=" << factorDist2Vel;
        yInfo() << "factorAng2Vel=" << factorAng2Vel;
        yInfo() << "inputPortName=" << inputPortName;
        yInfo() << "factorDist2Vel=" << outputPortName;
        yInfo() << "distanceThreshold=" << distanceThreshold;
        yInfo() << "angleThreshold=" << angleThreshold;
    }
};



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
    //Ball3DPPointRetriver       m_pointRetriver;
    Person3DPPointRetriver     m_pointRetriver;

#if 0
    // correct but maybe .. i try to invert (as suggested by silvio)
    const std::string target_frame_id = "base_link"; //"head_leopard_right";
    const std::string source_frame_id = "head_leopard_left";
#else
    // marco.accame: ok, it works.
    // conclusion: there is a bug in the methods. it is used an inverse matrix.
    const std::string target_frame_id = "head_leopard_left";
    const std::string source_frame_id = "mobile_base_body_link";//"base_link";
#endif
    yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort; //From this port I receive the data from Pf3dtraker
    yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPort2baseCtr; //I send commands to baseControl interruptModule
    yarp::os::BufferedPort<yarp::os::Property>  m_outputPort2gazeCtr; //I send commands to the gaze controller

    FollowerConfig m_cfg;



    void followBall(); //core function call in updateModule.

    //get transform matrix from left camera to mobile base. Pf3dtraker use the left camera.
    bool getMatrix(yarp::sig::Matrix &transform);

    bool getBallPointTrasformed(yarp::sig::Vector &pointBallInput, yarp::sig::Vector &pointBallOutput);

    bool initTransformClient(void);

    bool readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg);

    bool sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity);
    bool sendCommand2GazeControl(double x, double y, double z);
    bool sendCommand2GazeControl_lookAtPixel(double u, double v);
    bool sendCommand2GazeControl_lookAtPoint(const  yarp::sig::Vector &x);


    // ---- TEST STUFF
     bool moveRobot(void);
     yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPortJoystick;//test only!!!used in sendOutput
     void sendOutputLikeJoystick(); //only for test. it simulates joystick


};
