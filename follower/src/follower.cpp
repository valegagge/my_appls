#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <math.h>

#include "follower.h"


#include "../../../yarp/src/libYARP_OS/include/yarp/os/Bottle.h"
using namespace std;
using namespace yarp::os;


//------------------------ buffer helper test ---------------------

void Follower::sendOutputLikeJoystick()
{
    static yarp::os::Stamp stamp;

    //send data to baseControl module
    if (m_outputPortJoystick.getOutputCount() == 0)
    {
        //if I have not connection I don't send anything
        return;
    }
    stamp.update();
    Bottle &b = m_outputPortJoystick.prepare();
    m_outputPortJoystick.setEnvelope(stamp);
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

    m_outputPortJoystick.write();
}


void Follower::followTarget(Target_t &target)
{
    //1. read target poosition
    if(!target.second)
    {
        //yError() << "FOLLOWER: I can't see the ball!!!";
        //lancia il timer: se per x secondi non ho ancora un target valido ==> vai alla navigazione da solo verso last target
        return;
    }
    else
    {
        m_lastValidTarget = target;
        //ferma il timer
        yDebug() << "FOLLOWER: ho un target valido!!!" << target.first[0] << target.first[1] << target.first[2] ;
    }

    if(m_stateMachine_st != FollowerStateMachine::running)
    {
        return; //forse e' da anticipare per il discorso del timer??
    }

    
    //3. transform the ball-point from camera point of view to base point of view.
    yarp::sig::Vector targetOnCamFrame(3), targetOnBaseFrame;
    targetOnCamFrame[0] = target.first[0];
    targetOnCamFrame[1] = target.first[1];
    targetOnCamFrame[2] = target.first[2];

    if(!transformPointInBaseFrame(targetOnCamFrame, targetOnBaseFrame))
    {
        return;
    }

    //4. x axis is the first element, y is on second. (In order to calculate the distance see mobile base frame)
    double distance =  sqrt(pow(targetOnBaseFrame[0], 2) + pow(targetOnBaseFrame[1], 2));

    const double RAD2DEG  = 180.0/M_PI;
    double angle = atan2(targetOnBaseFrame[1], targetOnBaseFrame[0]) * RAD2DEG;

    double lin_vel  = 0.0;
    double ang_vel = 0.0;

   yDebug() << "distance=" << distance ;
    if(distance > m_cfg.distanceThreshold)
    {
        lin_vel = m_cfg.factorDist2Vel *distance;
    }
    else
        cout << "the Distance is under threshold!! " <<endl; //only for debug purpose



    if(fabs(angle) >m_cfg.angleThreshold)
        ang_vel = m_cfg.factorAng2Vel * angle;
    else
        cout << "the angle is under threshold!! " <<endl; // //only for debug purpose

    //if the angle difference is minor of angleMinBeforeMove than set linear velocity to 0
    // in order to rotate and after moving.
    if((abs(angle) < m_cfg.angleMinBeforeMove) && (fabs(angle) >m_cfg.angleThreshold))
        lin_vel = 0.0;

    //saturate velocities
    if(ang_vel > m_cfg.velocityLimits.angular)
        ang_vel= m_cfg.velocityLimits.angular;

    if(lin_vel> m_cfg.velocityLimits.linear)
        lin_vel = m_cfg.velocityLimits.linear;

    yDebug() << "sendCommand2BaseControl linvel=" << lin_vel <<"ang_vel" <<ang_vel ;
    sendCommand2BaseControl(0.0, lin_vel, ang_vel );

    //5. send commands to gaze control
//     double ballPointU, ballPointV;
//     (dynamic_cast<Ball3DPPointRetriver*>(m_pointRetriver_ptr))->getTargetPixelCoord(ballPointU, ballPointV);
    //yDebug() << "Point of image: " << ballPointU << ballPointV;
    //sendCommand2GazeControl_lookAtPixel(ballPointU, ballPointV);
//    yDebug() << "Look at point " << targetOnCamFrame.toString();
    sendCommand2GazeControl_lookAtPoint(targetOnBaseFrame);


    if(!isRunningInsimulation())
        return;

    //6. paint in gazebo the target on cam and the final target
    if(m_cfg.paintGazeFrame)
    {
        yDebug() << "paint gaze frame";
        yarp::sig::Vector targetOnHeadFrame;
        if(transformPointInHeadFrame(m_transformData.targetFrameId, targetOnCamFrame, targetOnHeadFrame))
        {
            targetOnHeadFrame[2]+=0.20;
            m_simmanager_ptr->PaintGazeFrame(targetOnHeadFrame);
        }
        else
        {
            yError() << "error in transforming cam frame to head frame";
        }
    }
    yDebug() << "paint target frame";
    yarp::sig::Vector target2Paint= targetOnBaseFrame;
    target2Paint[2]=0.0; //I don't want z axis
    m_simmanager_ptr->PaintTargetFrame(target2Paint);
}

bool Follower::start()
{
    m_stateMachine_st = FollowerStateMachine::running;
    return true;
}

bool Follower::stop()
{
    m_stateMachine_st = FollowerStateMachine::configured;
    return true;
}

bool Follower::getMatrix(yarp::sig::Matrix &transform)
{
    bool res = m_transformData.transformClient->getTransform (m_transformData.targetFrameId, m_transformData.baseFrameId, transform);
    if(res)
    {
//         yDebug() << "FOLLOWER: i get the transform matrix:"; // << transform.toString();
//
//         std::cout << transform.toString() << std::endl << std::endl;
    }
    else
    {
        yError() << "FOLLOWER: error in getting transform matrix";
    }

    return res;
}

bool Follower::transformPointInBaseFrame(yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput)
{
    bool res = m_transformData.transformClient->transformPoint(m_transformData.targetFrameId, m_transformData.baseFrameId, pointInput, pointOutput);
    if(res)
    {
        //        yDebug() << "FOLLOWER: point (" << pointInput.toString() << ") has been transformed in (" << pointOutput.toString() << ")";
    }
    else
    {
        yError() << "FOLLOWER: error in transformPointInBaseFrame()";
    }

    return res;
}

bool Follower::transformPointInHeadFrame(std::string frame_src, yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput)
{
    bool res = m_transformData.transformClient->transformPoint(frame_src, "head_link", pointInput, pointOutput);
    if(res)
    {
        //        yDebug() << "FOLLOWER: point (" << pointBallInput.toString() << ") has been transformed in (" << pointBallOutput.toString() << ")";
    }
    else
    {
        yError() << "FOLLOWER: error in getting transform point in head frame";
    }

    return res;
}



bool Follower::readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg)
{
    Bottle config_group = rf.findGroup("GENERAL");
    if (config_group.isNull())
    {
        yError() << "Missing GENERAL group! the module uses default value!";
    }
    else
    {
        if (config_group.check("inputPort"))  {cfg.inputPortName = config_group.find("inputPort").asString(); }
        if (config_group.check("outputPort"))  { cfg.outputPortName = config_group.find("outputPort").asString(); }
        if (config_group.check("targetType"))  { cfg.targetType = config_group.find("targetType").asString(); }
    }


    config_group = rf.findGroup("TRAJECTORY");
    if (config_group.isNull())
    {
        yError() << "Missing TRAJECTORY group! the module uses default value!";
    }
    else
    {
        if (config_group.check("factorDist2Vel")) { cfg.factorDist2Vel = config_group.find("factorDist2Vel").asDouble(); }
        if (config_group.check("factorAng2Vel"))  { cfg.factorAng2Vel = config_group.find("factorAng2Vel").asDouble(); }
        if (config_group.check("distanceThreshold"))  { cfg.distanceThreshold = config_group.find("distanceThreshold").asDouble(); }
        if (config_group.check("angleThreshold"))  { cfg.angleThreshold = config_group.find("angleThreshold").asDouble(); }
        if (config_group.check("angularVelLimit"))  { cfg.velocityLimits.angular = config_group.find("angularVelLimit").asDouble(); }
        if (config_group.check("linearVelLimit"))  { cfg.velocityLimits.linear = config_group.find("linearVelLimit").asDouble(); }
        if (config_group.check("angleMinBeforeMove"))  { cfg.angleMinBeforeMove = config_group.find("angleMinBeforeMove").asDouble(); }
    }

    cfg.print();
    return true;}



// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool Follower::configure(yarp::os::ResourceFinder &rf)
{
    m_outputPortJoystick.open("/follower/test-joystick:o");//test


    if(!readConfig(rf, m_cfg))
    {
        yError() << "Error reading configuration file";
        return false;
    }

    if(m_cfg.targetType == "redball")
    {
        m_transformData.targetFrameId = m_transformData.redBallFrameId;
        m_targetType = FollowerTargetType::redball;
    }
    else //person or default
    {
        m_transformData.targetFrameId = m_transformData.personFrameId;
        m_targetType = FollowerTargetType::person;
    }


    if(!m_outputPort2baseCtr.open("/follower/" + m_cfg.outputPortName + ":o"))
    {
        yError() << "Error opening output port for base control";
        return false;
    }

    //if(!m_outputPort2gazeCtr.open("/follower/" + "gazetargets" + ":o"))
     if(!m_outputPort2gazeCtr.open("/follower/gazetargets:o"))
    {
        yError() << "Error opening output port for gaze control";
        return false;
    }

    if(m_onSimulation)
    {
        m_simmanager_ptr = new SimManager();
        m_simmanager_ptr->init("SIM_CER_ROBOT", "/follower/worldInterface/rpc");
    }

    if(!initTransformClient())
        return false;

    m_stateMachine_st=FollowerStateMachine::configured;
    return true;
}


// Close function, to perform cleanup.
bool Follower::close()
{

    m_stateMachine_st = FollowerStateMachine::none;
    m_transformData.driver.close();
    m_outputPort2baseCtr.interrupt();
    m_outputPort2baseCtr.close();

    m_outputPort2gazeCtr.interrupt();
    m_outputPort2gazeCtr.close();

    if(m_simmanager_ptr)
        m_simmanager_ptr->deinit();


    return true;
}


Follower::Follower(): m_targetType(FollowerTargetType::person),m_onSimulation(true), m_simmanager_ptr(nullptr), m_stateMachine_st(FollowerStateMachine::none)
{
    m_transformData.transformClient = nullptr;
    m_lastValidTarget.second = false;
}

Follower::~Follower()
{
    delete m_simmanager_ptr;
}

FollowerTargetType Follower::getTargetType(void)
{return m_targetType;}


FollowerStateMachine Follower::getState(void)
{
    return m_stateMachine_st;
}

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
    bool ok_open = m_transformData.driver.open(propTfClient);
    if (!ok_open)
    {
        yError() << "Unable to open the FrameTransformClient driver.";
        return false;
    }

    // Try to retrieve the view
    bool ok_view = m_transformData.driver.view(m_transformData.transformClient);
    if (!ok_view || m_transformData.transformClient == 0)
    {
        yError() << "Unable to retrieve the FrameTransformClient view.";
        return false;
    }

    //from now I can use m_transformData.transformClient
    return true;
}

//test function: if you want use it pu it in update()
bool Follower::moveRobot(void)
{

    if(m_outputPort2baseCtr.getOutputCount() == 0)
        return true;

    static double absTime = yarp::os::Time::now();
    static double startTime = yarp::os::Time::now();
    static int direction = 1.0;

    double currTime = yarp::os::Time::now();

    if(currTime - absTime >120.0)// after 2 minutes stop robot
    {
        //stop moveRobot
        sendCommand2BaseControl(0.0, 0.0, 0.0);
        cout << "_";
        return true;

    }
    if(currTime-startTime < 10.0)
    {
        sendCommand2BaseControl(0.0, 0.0, 10.0*direction );
        cout << ".";
    }
    else
    {
        startTime = yarp::os::Time::now();
        (direction >0)? direction = -1.0 : direction=1.0;
        cout <<"|";
    }
    cout.flush();
    return true;
}



bool Follower::sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity)
{
    static yarp::os::Stamp stamp;

    stamp.update();
    //send velocity commands to the base control
    if (m_outputPort2baseCtr.getOutputCount() > 0)
    {
        Bottle &b = m_outputPort2baseCtr.prepare();
        m_outputPort2baseCtr.setEnvelope(stamp);
        b.clear();
        b.addInt(2);                    // polar speed commands
        b.addDouble(linearDirection);    // angle in deg
        b.addDouble(linearVelocity);    // lin_vel in m/s
        b.addDouble(angularVelocity);    // ang_vel in deg/s
        b.addDouble(100);
        m_outputPort2baseCtr.write();
    }

    return true;

}

bool Follower::sendCommand2GazeControl(double x, double y, double z)
{
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();
    //p.put("control-frame","left");
    p.put("control-frame","gaze");
    p.put("target-type","cartesian");

    Bottle location = yarp::os::Bottle();
    Bottle &val = location.addList();
    val.addDouble(x);
    val.addDouble(y);
    val.addDouble(z);
    p.put("target-location",location.get(0));
    m_outputPort2gazeCtr.write();

    return true;

}


bool Follower::sendCommand2GazeControl_lookAtPixel(double u, double v)
{
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();
    p.put("control-frame","left");
    p.put("target-type","image");
    p.put("image","left");

    Bottle location = yarp::os::Bottle();
    Bottle &val = location.addList();
    val.addDouble(u);
    val.addDouble(v);
    p.put("target-location",location.get(0));
    m_outputPort2gazeCtr.write();

    return true;

}


bool Follower::sendCommand2GazeControl_lookAtPoint(const  yarp::sig::Vector &x)
{
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();
    if(m_targetType==FollowerTargetType::person)
        p.put("control-frame","depth_center");
    else
        p.put("control-frame","left");

    p.put("target-type","cartesian");

    Bottle target;
    target.addList().read(x);
    p.put("target-location",target.get(0));


    yDebug() << "Command to gazectrl: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;

}


