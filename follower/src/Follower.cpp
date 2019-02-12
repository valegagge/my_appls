/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Follower.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <math.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>

#include "Follower.h"


using namespace std;
using namespace yarp::os;
using namespace FollowerTarget;

void FollowerConfig::print(void)
{
    yInfo() << "The follower module has been configure with following values";
    yInfo() << "factorDist2Vel=" << factorDist2Vel;
    yInfo() << "factorAng2Vel=" << factorAng2Vel;
    yInfo() << "inputPortName=" << inputPortName;
    yInfo() << "factorDist2Vel=" << outputPortName;
    yInfo() << "distanceThreshold=" << distanceThreshold;
    yInfo() << "angleThreshold=" << angleThreshold;
    yInfo() << "targetType=" << targetType;
    yInfo() << "angularVelLimit=" << velocityLimits.angular;
    yInfo() << "linearVelLimit=" << velocityLimits.linear;
    yInfo() << "angleMinBeforeMove=" << angleMinBeforeMove;
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

void Follower::followTarget(Target_t &target)
{
    //1. read target position
    if(!target.second)
    {
        if(m_cfg.debug.enabled)
            yDebug() << "I can't see the target!!!";
        //TODO:in here start a timer with timeout T seconds (configurable).
        //      if timer expires and I don't see the target than start autonomous navigation toward last valid position of target
        return;
    }
    else
    {
        m_lastValidTarget = target;
        //TODO: stop here the target
        if(m_cfg.debug.enabled)
            yDebug() << "I received a valid target!!! (I see the target=" << target.first[0] << target.first[1] << target.first[2] <<")" ;
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if(m_stateMachine_st != FollowerStateMachine::running)
        {
            return;
        }
    }



    //2. transform the ball-point from camera point of view to base point of view.
    yarp::sig::Vector targetOnCamFrame(3), targetOnBaseFrame;
    targetOnCamFrame[0] = target.first[0];
    targetOnCamFrame[1] = target.first[1];
    targetOnCamFrame[2] = target.first[2];

    if(!transformPointInBaseFrame(targetOnCamFrame, targetOnBaseFrame))
    {
        return;
    }

    //3. Calculate linear velocity and angular velocity to send to  base control module

    //x axis is the first element, y is on second. (In order to calculate the distance see mobile base frame)
    double distance =  sqrt(pow(targetOnBaseFrame[0], 2) + pow(targetOnBaseFrame[1], 2));

    const double RAD2DEG  = 180.0/M_PI;
    double angle = atan2(targetOnBaseFrame[1], targetOnBaseFrame[0]) * RAD2DEG;

    double lin_vel  = 0.0;
    double ang_vel = 0.0;


    if(distance > m_cfg.distanceThreshold)
    {
        lin_vel = m_cfg.factorDist2Vel *distance;
    }
    else
    {
        if(m_cfg.debug.enabled)
            yDebug() <<  "the distance is under threshold!! ";
    }


    if(fabs(angle) >m_cfg.angleThreshold)
        ang_vel = m_cfg.factorAng2Vel * angle;
    else
    {
        if(m_cfg.debug.enabled)
            yDebug() <<  "the angle is under threshold!! ";
    }

    //if the angle difference is minor of angleMinBeforeMove than set linear velocity to 0
    // in order to rotate and after moving.
    if((abs(angle) < m_cfg.angleMinBeforeMove) && (fabs(angle) >m_cfg.angleThreshold))
        lin_vel = 0.0;

    //saturate velocities
    if(ang_vel > m_cfg.velocityLimits.angular)
        ang_vel= m_cfg.velocityLimits.angular;

    if(lin_vel> m_cfg.velocityLimits.linear)
        lin_vel = m_cfg.velocityLimits.linear;

    if(m_cfg.debug.enabled)
        yDebug() << "sendCommand2BaseControl linvel=" << lin_vel <<"ang_vel" <<ang_vel ;

    sendCommand2BaseControl(0.0, lin_vel, ang_vel );

    //4. send commands to gaze control in order to follow the target with the gaze
    //     double ballPointU, ballPointV;
    //     (dynamic_cast<Ball3DPPointRetriver*>(m_pointRetriver_ptr))->getTargetPixelCoord(ballPointU, ballPointV);
    //yDebug() << "Point of image: " << ballPointU << ballPointV;
    //sendCommand2GazeControl_lookAtPixel(ballPointU, ballPointV);
    //    yDebug() << "Look at point " << targetOnCamFrame.toString();
    sendCommand2GazeControl_lookAtPoint(targetOnBaseFrame);

    //the following steps lose interest on real robot
    if(!isRunningInsimulation())
        return;

    //5. paint in gazebo the targets on final target and on cam(optional)
    yarp::sig::Vector target2Paint= targetOnBaseFrame;
    target2Paint[2]=0.0; //I don't want z axis
    m_simmanager_ptr->PaintTargetFrame(target2Paint);

    if(m_cfg.debug.paintGazeFrame)
    {
        yarp::sig::Vector targetOnHeadFrame;
        if(transformPointInHeadFrame(m_transformData.targetFrameId, targetOnCamFrame, targetOnHeadFrame))
        {
            targetOnHeadFrame[2]+=0.20;
            m_simmanager_ptr->PaintGazeFrame(targetOnHeadFrame);
        }

    }
}


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

     if(!m_outputPort2gazeCtr.open("/follower/gazetargets:o"))
    {
        yError() << "Error opening output port for gaze control";
        return false;
    }

    if(m_onSimulation)
    {
        m_simmanager_ptr = new SimManager();
        m_simmanager_ptr->init("SIM_CER_ROBOT", "/follower/worldInterface/rpc", m_cfg.debug.enabled);
    }

    if(!initTransformClient())
        return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateMachine_st=FollowerStateMachine::configured;
    return true;
}


// Close function, to perform cleanup.
bool Follower::close()
{

    std::lock_guard<std::mutex> lock(m_mutex);
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


bool Follower::start()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateMachine_st = FollowerStateMachine::running;
    return true;
}


bool Follower::stop()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateMachine_st = FollowerStateMachine::configured;
    return true;
}



FollowerTargetType Follower::getTargetType(void)
{
    return m_targetType;
}


FollowerStateMachine Follower::getState(void)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stateMachine_st;
}

//------------------------------------------------
// private function
//------------------------------------------------

bool Follower::transformPointInBaseFrame(yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput)
{
    bool res = m_transformData.transformClient->transformPoint(m_transformData.targetFrameId, m_transformData.baseFrameId, pointInput, pointOutput);
    if(res)
    {
        //        yDebug() << "point (" << pointInput.toString() << ") has been transformed in (" << pointOutput.toString() << ")";
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
        //        yDebug() << "point (" << pointBallInput.toString() << ") has been transformed in (" << pointBallOutput.toString() << ")";
    }
    else
    {
        yError() << "Error in getting transform point from " << frame_src <<" to head_link";
    }

    return res;
}



bool Follower::readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg)
{
    Bottle config_group = rf.findGroup("GENERAL");
    if (config_group.isNull())
    {
        yWarning() << "Missing GENERAL group! the module uses default value!";
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
        yWarning() << "Missing TRAJECTORY group! the module uses default value!";
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


    config_group = rf.findGroup("DEBUG");
    if (config_group.isNull())
    {
        yWarning() << "Missing DEBUG group! the module uses default value!";
    }
    else
    {
        if (config_group.check("enable")) { cfg.debug.enabled = config_group.find("enable").asBool(); }
        if (config_group.check("paintGazeFrame"))  { cfg.debug.paintGazeFrame = config_group.find("paintGazeFrame").asBool(); }
        if (config_group.check("startWithoutCommand"))  { cfg.debug.startWithoutCommand= config_group.find("startWithoutCommand").asBool(); }
    }

    cfg.print();
    return true;

}

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

    if(m_cfg.debug.enabled)
        yDebug() << "Command to gazectrl: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;

}


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// TEST FUNCTION ///////////////////////////////////////////////////////

//test function: if you want use it put it in update()
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
        //cout << "_";
        return true;

    }
    if(currTime-startTime < 10.0)
    {
        sendCommand2BaseControl(0.0, 0.0, 10.0*direction );
        //cout << ".";
    }
    else
    {
        startTime = yarp::os::Time::now();
        (direction >0)? direction = -1.0 : direction=1.0;
        //cout <<"|";
    }
    //cout.flush();
    return true;
}


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

    m_outputPortJoystick.write();
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
