

#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcClient.h>

#include <string>

#include "TargetRetriver.h"
#include "Person3DRetriver.h"
#include "simFramePainter.h"

class FollowerConfig
{
public:
    double factorDist2Vel;
    double factorAng2Vel;
    std::string inputPortName;
    std::string outputPortName;
    double distanceThreshold = 0.8;
    double angleThreshold = 3.0;
    std::string targetType;
    struct
    {
        double angular;
        double linear;
    }velocityLimits;
    double angleMinBeforeMove;
    bool paintGazeFrame;

    FollowerConfig()
    {
        //init with default values
        factorDist2Vel = 0.8;
        factorAng2Vel = 0.8;
        inputPortName = "targetPoint";
        outputPortName = "commands";
        distanceThreshold = 0.8;
        angleThreshold = 3.0;
        targetType = "person";
        velocityLimits.angular = 30; //degree/sec
        velocityLimits.linear = 3; //m/s
        angleMinBeforeMove = 10.0; //degree
        paintGazeFrame = false;
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
        yInfo() << "targetType=" << targetType;
    }
};

enum class FollowerTargetType
{
    redball, person
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

    FollowerTargetType         m_targetType;
    TargetRetriver*            m_pointRetriver_ptr; //the target retriver read the input port and get the target point.


    const std::string m_redBallFrameId = "head_leopard_left";
    const std::string m_personFrameId = "depth_center";
    const std::string m_baseFrameId = "mobile_base_body_link";
    std::string m_targetFrameId;

//     bool m_targetBoxIsCreated;
//     yarp::os::RpcClient m_worldInterfacePort;
//     const std::string m_nameTargetBox="targetBox2";
    bool m_onSimulation;
    SimManager * m_simmanager_ptr;

    yarp::os::Port m_rpcPort;

    yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPort2baseCtr; //I send commands to baseControl interruptModule
    yarp::os::BufferedPort<yarp::os::Property>  m_outputPort2gazeCtr; //I send commands to the gaze controller

    FollowerConfig m_cfg;



    void followTarget(Target_t &target); //core function call in updateModule.

    //get transform matrix from left camera to mobile base. Pf3dtraker use the left camera.
    bool getMatrix(yarp::sig::Matrix &transform);

    bool transformPointInBaseFrame(yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput);
    bool transformPointInHeadFrame(std::string frame_src, yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput);

    bool initTransformClient(void);

    bool readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg);

    bool sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity);
    bool sendCommand2GazeControl(double x, double y, double z);
    bool sendCommand2GazeControl_lookAtPixel(double u, double v);
    bool sendCommand2GazeControl_lookAtPoint(const  yarp::sig::Vector &x);
    void paintTargetPoint(const  yarp::sig::Vector &target);
    void paintTargetPoint2(yarp::sig::Vector &target);
    bool isRunningInsimulation(void) {return((m_simmanager_ptr==nullptr) ? false :true);}


    // ---- TEST STUFF
     bool moveRobot(void);
     yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPortJoystick;//test only!!!used in sendOutput
     void sendOutputLikeJoystick(); //only for test. it simulates joystick


};
