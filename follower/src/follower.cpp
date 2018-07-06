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
    followBall();
//    moveRobot();
   return true;
}

void Follower::followBall(void)
{
    //1. get the transform matrix
    //this step is not necessary... we use it only for debug purpose
    yarp::sig::Matrix transform;
    if(!getMatrix(transform))
    {
        yError() << "FOLLOWER: cannot get the matrix";
        return;
    }

    //2. read ball poosition
    Bottle *b = m_inputPort.read();

//     yDebug() << "Vedo pallina " << b->get(6).asDouble();
//     yDebug() << "pos pallina" << b->get(0).asDouble() << b->get(1).asDouble() << b->get(2).asDouble();
    bool ballIsTracked = (b->get(6).asDouble() == 1.0) ? true : false;

    if(!ballIsTracked)
    {
        yError() << "FOLLOWER: I can't see the ball!!!";
        return;
    }

    
    //3. transform the ball-point from camera point of view to base point of view.
    yarp::sig::Vector pointBallInput(3), pointBallOutput;
    pointBallInput[0] = b->get(0).asDouble();
    pointBallInput[1] = b->get(1).asDouble();
    pointBallInput[2] = b->get(2).asDouble();

    double ballPointU = b->get(4).asDouble(); //u and V are the the coordinate x any of image.
    double ballPointV = b->get(5).asDouble();

    if(!getBallPointTrasformed(pointBallInput, pointBallOutput))
    {
        yError() << "FOLLOWER: error in getBallPointTrasformed()";
        return;
    }

    //4. x axis is the first element, y is on second. (In order to calculate the distance see mobile base frame)
    double distance =  sqrt(pow(pointBallOutput[0], 2) + pow(pointBallOutput[1], 2));

    const double RAD2DEG  = 180.0/M_PI;
    double angle = atan2(pointBallOutput[1], pointBallOutput[0]) * RAD2DEG;

    double lin_vel  = 0.0;
    double ang_vel = 0.0;


    if(distance > m_cfg.distanceThreshold)
    {
        lin_vel = m_cfg.factorDist2Vel *distance;
    }
    else
        cout << "the Distance is under threshold!! " <<endl; //only for debug pupose



    if(fabs(angle) >m_cfg.angleThreshold)
        ang_vel = m_cfg.factorAng2Vel * angle;
    else
        cout << "the angle is under threshold!! " <<endl; // //only for debug pupose


    sendCommand2BaseControl(0.0, lin_vel, ang_vel );

    //5. send commands to gaze control
    sendCommand2GazeControl_lookAtPixel(ballPointU, ballPointV);
}


bool Follower::getMatrix(yarp::sig::Matrix &transform)
{
    bool res = m_transformClient->getTransform (target_frame_id, source_frame_id, transform);
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

bool Follower::getBallPointTrasformed(yarp::sig::Vector &pointBallInput, yarp::sig::Vector &pointBallOutput)
{
    bool res = m_transformClient->transformPoint(target_frame_id, source_frame_id, pointBallInput, pointBallOutput);
    if(res)
    {
//        yDebug() << "FOLLOWER: point (" << pointBallInput.toString() << ") has been transformed in (" << pointBallOutput.toString() << ")";
    }
    else
    {
        yError() << "FOLLOWER: error in getting transform point";
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

    if(! m_inputPort.open("/follower/" + m_cfg.inputPortName +":i"))
    {
        yError() << "Error opening input port";
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


    if(!initTransformClient())
        return false;


    return true;
}
// Interrupt function.
bool Follower::interruptModule()
{

    m_outputPortJoystick.interrupt();
    m_outputPortJoystick.close();

    m_inputPort.interrupt();
    m_inputPort.close();

    m_outputPort2baseCtr.interrupt();
    m_outputPort2baseCtr.close();

    m_outputPort2gazeCtr.interrupt();
    m_outputPort2gazeCtr.close();
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


Follower::Follower():m_transformClient(nullptr)
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