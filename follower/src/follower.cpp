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

double Follower::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return 0.01;
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

 /*
    b.addInt(2);                    // polar speed commands
    b.addDouble(m_control_out.linear_dir);    // angle in deg
    b.addDouble(m_control_out.linear_vel);    // lin_vel in m/s
    b.addDouble(m_control_out.angular_vel);    // ang_vel in deg/s
    b.addDouble(100);*/

    m_outputPortJoystick.write();
    }

// This is our main function. Will be called periodically every getPeriod() seconds
bool Follower::updateModule()
{
//    sendOutput();
    Target_t targetpoint({0,0,0}, false);


    switch(m_targetType)
    {
        case FollowerTargetType::person:
        { targetpoint= (dynamic_cast<Person3DPPointRetriver*>(m_pointRetriver_ptr))->getTarget(); break;}

        case FollowerTargetType::redball:
        { targetpoint= (dynamic_cast<Ball3DPPointRetriver*>(m_pointRetriver_ptr))->getTarget(); break;}

        default: break;
    };

    followTarget(targetpoint);
//    moveRobot();
   return true;
}


void Follower::followTarget(Target_t &target)
{
    //1. get the transform matrix
    //this step is not necessary... we use it only for debug purpose
    yarp::sig::Matrix transform;
    if(!getMatrix(transform))
    {
        yError() << "FOLLOWER: cannot get the matrix";
        return;
    }

    //2. read target poosition
    if(!target.second)
    {
        //yError() << "FOLLOWER: I can't see the ball!!!";
        return;
    }
    else
    {
        yDebug() << "FOLLOWER: ho un target valido!!!" << target.first[0] << target.first[1] << target.first[2] ;
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
        if(transformPointInHeadFrame(m_targetFrameId, targetOnCamFrame, targetOnHeadFrame))
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


bool Follower::getMatrix(yarp::sig::Matrix &transform)
{
    bool res = m_transformClient->getTransform (m_targetFrameId, m_baseFrameId, transform);
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
    bool res = m_transformClient->transformPoint(m_targetFrameId, m_baseFrameId, pointInput, pointOutput);
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
    bool res = m_transformClient->transformPoint(frame_src, "head_link", pointInput, pointOutput);
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


// Message handler. Just echo all received messages.
bool Follower::respond(const Bottle& command, Bottle& reply)
{
    cout << "Got something, echo is on" << endl;
    if (command.get(0).asString() == "quit")
        return false;
    else
        reply = command;
    return true;
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
        if (config_group.check("factorDist2Vel")) { cfg.factorDist2Vel = config_group.find("factorDist2Vel").asDouble(); }
        if (config_group.check("factorAng2Vel"))  { cfg.factorAng2Vel = config_group.find("factorAng2Vel").asDouble(); }
        if (config_group.check("inputPort"))  {cfg.inputPortName = config_group.find("inputPort").asString(); }
        if (config_group.check("outputPort"))  { cfg.outputPortName = config_group.find("outputPort").asString(); }
        if (config_group.check("distanceThreshold"))  { cfg.distanceThreshold = config_group.find("distanceThreshold").asDouble(); }
        if (config_group.check("angleThreshold"))  { cfg.angleThreshold = config_group.find("angleThreshold").asDouble(); }
        if (config_group.check("targetType"))  { cfg.targetType = config_group.find("targetType").asString(); }
    }

    cfg.print();
    return true;
}



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
        m_pointRetriver_ptr = new Ball3DPPointRetriver();
        m_targetFrameId = m_redBallFrameId;
        m_targetType = FollowerTargetType::redball;
    }
    else //person or default
    {
        m_pointRetriver_ptr = new Person3DPPointRetriver();
        m_targetFrameId = m_personFrameId;
        m_targetType = FollowerTargetType::person;
    }


    if(! m_pointRetriver_ptr->init("/follower/" + m_cfg.inputPortName +":i"))
    {
        yError() << "Error in initializing the Target Retriver";
        return false;
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

//     if(!m_worldInterfacePort.open("/follower/worldInterface/rpc"))
//     {
//         yError() << "Error opening worldInterface rpc port!!";
//         return false;
//     }

    if(!initTransformClient())
        return false;


    return true;
}
// Interrupt function.
bool Follower::interruptModule()
{

    m_outputPortJoystick.interrupt();
    m_outputPortJoystick.close();

    m_pointRetriver_ptr->deinit();

    m_outputPort2baseCtr.interrupt();
    m_outputPort2baseCtr.close();

    m_outputPort2gazeCtr.interrupt();
    m_outputPort2gazeCtr.close();

//     m_worldInterfacePort.interrupt();
//     m_worldInterfacePort.close();
    if(isRunningInsimulation())
        m_simmanager_ptr->deinit();

    if(m_pointRetriver_ptr!=nullptr)
        m_pointRetriver_ptr->deinit();

    return true;
}
// Close function, to perform cleanup.
bool Follower::close()
{

    m_driver.close();
    // optional, close port explicitly
    cout << "Calling close function\n";
    return true;
}


Follower::Follower():m_transformClient(nullptr), m_targetType(FollowerTargetType::person), m_pointRetriver_ptr(nullptr),
                    m_onSimulation(true), m_simmanager_ptr(nullptr)
{;}
Follower::~Follower(){;}


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
    bool ok_open = m_driver.open(propTfClient);
    if (!ok_open)
    {
        yError() << "Unable to open the FrameTransformClient driver.";
        return false;
    }

    // Try to retrieve the view
    bool ok_view = m_driver.view(m_transformClient);
    if (!ok_view || m_transformClient == 0)
    {
        yError() << "Unable to retrieve the FrameTransformClient view.";
        return false;
    }

    //from now I can use m_transformClient
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

//     Bottle target =  yarp::os::Bottle();
//     Bottle &val = target.addList();
//     val.addList().read(x);
    Bottle target;
    target.addList().read(x);
    p.put("target-location",target.get(0));


    yDebug() << "Command to gazectrl: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;

}


// void Follower::paintTargetPoint(const  yarp::sig::Vector &target)
// {
//     if( (!m_targetBoxIsCreated) && (m_worldInterfacePort.asPort().getOutputCount() >0 ))
//     {
//         yDebug() << "I'm about to create the target frame...";
//         Bottle cmd, ans;
//         cmd.clear();
//         ans.clear();
//
//         cmd.addString("makeFrame");
//         cmd.addDouble(0.2); //size
//         cmd.addDouble(target[0]);
//         cmd.addDouble(target[1]); //y
//         cmd.addDouble(target[2]); //z
//         cmd.addDouble(0); //r
//         cmd.addDouble(0); //p
//         cmd.addDouble(0); //y
//         //orange color on central ball
//         cmd.addInt(255); //red
//         cmd.addInt(128); //green
//         cmd.addInt(0); //blue
//         cmd.addString("SIM_CER_ROBOT::mobile_base_body_link");
//         cmd.addString(m_nameTargetBox); //box obj name
//
//         m_worldInterfacePort.write(cmd, ans);
//         yDebug() << "follower: makeBox= " << cmd.toString() << "  Ans=" << ans.toString();
//         if(ans.toString() == m_nameTargetBox)
//         {
//             m_targetBoxIsCreated = true;
//             return;
//         }
//         else
//             m_targetBoxIsCreated = false;
//     }
//
//     if(!m_targetBoxIsCreated)
//     {
//         return;
//     }
//
//     // Prapare bottle containg command to send in order to get the current position
//     Bottle cmdGet, ansGet, cmdSet, ansSet;
//     cmdGet.clear();
//     ansGet.clear();
//     cmdSet.clear();
//     ansSet.clear();
//     cmdGet.addString("getPose");
//     cmdGet.addString(m_nameTargetBox);
//     cmdGet.addString("SIM_CER_ROBOT::mobile_base_body_link");
//     m_worldInterfacePort.write(cmdGet, ansGet);
//     //read the answer
//
//     //send command for new position
//     cmdSet.addString("setPose");
//     cmdSet.addString(m_nameTargetBox);
//     cmdSet.addDouble(target[0]);
//     cmdSet.addDouble(target[1]);
//     cmdSet.addDouble(target[2]); // z
//     cmdSet.addDouble(ansGet.get(3).asDouble()); // r
//     cmdSet.addDouble(ansGet.get(4).asDouble()); // p
//     cmdSet.addDouble(ansGet.get(5).asDouble()); // y
//     cmdSet.addString("SIM_CER_ROBOT::mobile_base_body_link");
//     m_worldInterfacePort.write(cmdSet, ansSet);
//
// }
//
//
// void Follower::paintTargetPoint2(yarp::sig::Vector &target)
// {
//     static bool isCreated=false;
//     static const std::string nameTargetBox="frameGaze";
//     yarp::sig::Vector targetOutput;
//
//     if(!transformPointInHeadFrame(m_targetFrameId, target, targetOutput))
//     {
//         yError() << "I cannot piant the source frame!!";
//         return;
//     }
//
//
//     if( (!isCreated) && (m_worldInterfacePort.asPort().getOutputCount() >0 ))
//     {
//         yDebug() << "I'm about to create the target frame...";
//         Bottle cmd, ans;
//         cmd.clear();
//         ans.clear();
//
//         cmd.addString("makeFrame");
//         cmd.addDouble(0.2); //size
//         cmd.addDouble(targetOutput[0]);
//         cmd.addDouble(targetOutput[1]); //y
//         cmd.addDouble(targetOutput[2]); //z
//         cmd.addDouble(0); //r
//         cmd.addDouble(0); //p
//         cmd.addDouble(0); //y
//         //orange color on central ball
//         cmd.addInt(0); //red
//         cmd.addInt(0); //green
//         cmd.addInt(0); //blue
//         cmd.addString("SIM_CER_ROBOT::head_link");
//         cmd.addString(nameTargetBox); //box obj name
//
//         m_worldInterfacePort.write(cmd, ans);
//         yDebug() << "follower: makeFramegaze= " << cmd.toString() << "  Ans=" << ans.toString();
//         if(ans.toString() == nameTargetBox)
//         {
//             isCreated = true;
//             return;
//         }
//         else
//             isCreated = false;
//     }
//
//     if(!isCreated)
//     {
//         return;
//     }
//
//     // Prapare bottle containg command to send in order to get the current position
//     Bottle cmdGet, ansGet, cmdSet, ansSet;
//     cmdGet.clear();
//     ansGet.clear();
//     cmdSet.clear();
//     ansSet.clear();
//     cmdGet.addString("getPose");
//     cmdGet.addString(nameTargetBox);
//     cmdGet.addString("SIM_CER_ROBOT::head_link");
//     m_worldInterfacePort.write(cmdGet, ansGet);
//     //read the answer
//
//     //send command for new position
//     cmdSet.addString("setPose");
//     cmdSet.addString(nameTargetBox);
//     cmdSet.addDouble(targetOutput[0]);
//     cmdSet.addDouble(targetOutput[1]);
//     cmdSet.addDouble(targetOutput[2]); // z
//     cmdSet.addDouble(ansGet.get(3).asDouble()); // r
//     cmdSet.addDouble(ansGet.get(4).asDouble()); // p
//     cmdSet.addDouble(ansGet.get(5).asDouble()); // y
//     cmdSet.addString("SIM_CER_ROBOT::head_link");
//     m_worldInterfacePort.write(cmdSet, ansSet);
//
// }
//
//
