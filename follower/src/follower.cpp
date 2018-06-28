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
void Follower::sendOutput()
{
    static yarp::os::Stamp stamp;

    //send data to baseControl module
    if (m_port_commands_output.getOutputCount() == 0)
    {
        //if I have not connection I don't send anything
        return;
    }
    stamp.update();
    Bottle &b = m_port_commands_output.prepare();
    m_port_commands_output.setEnvelope(stamp);
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

    m_port_commands_output.write();
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

    if(!getBallPointTrasformed(pointBallInput, pointBallOutput))
    {
        yError() << "FOLLOWER: error in getBallPointTrasformed()";
        return;
    }

    //4. the distance from the bal is on x
    double distance =  sqrt(pow(pointBallOutput[0], 2) + pow(pointBallOutput[1], 2));

    const double RAD2DEG  = 180.0/M_PI;
    double angle = atan2(pointBallOutput[1], pointBallOutput[0]) * RAD2DEG;

    double lin_vel  = 0.0;
    double ang_vel = 0.0;
    const double factorDist2Vel = 0.8;
    const double factorAng2Vel = 0.8;


    if(distance > 0.8)
        lin_vel = factorDist2Vel *distance;
    else
        cout << "distanza e' zero!!! " <<endl;



    if(fabs(angle) >5)
        ang_vel = factorAng2Vel * angle;
    else
        cout << "angolo e' zero!!! " <<endl;


    sendCommand2BaseControl(0.0, lin_vel, ang_vel );


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





// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool Follower::configure(yarp::os::ResourceFinder &rf)
{
    m_port_commands_output.open("/follower/control:o");

    m_inputPort.open("/follower/ballPoint:i");

    m_outputPort2baseCtr.open("/follower/commands:o");

    if(!initTransformClient())
        return false;


    return true;
}
// Interrupt function.
bool Follower::interruptModule()
{

    m_port_commands_output.interrupt();
    m_port_commands_output.close();

    m_inputPort.interrupt();
    m_inputPort.close();

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


// bool Follower::rotateRobot(double angle)
// {
//     static bool isDone = false;
//     static double targetAngle = angle;
//
//     double currTime = yarp::os::Time::now();
//
//     double currentAngle = 0.0; //to be done
//     static const double threshold  = 3.0;
//
//     if(isDone)
//         return true;
//
//     double diff  = angle-currentAngle;
//     if (fabs(diff) > threshold)
//     {
//         sendCommand2BaseControl(0.0, 0.0, 0.8*diff );
//
//     }
//     else
//     {
//         cout <<"it is done!!!!" <<endl;
//         isDone = true;
//     }
//
//     cout.flush();
//     return true;
// }

bool Follower::sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity)
{
    static yarp::os::Stamp stamp;

    stamp.update();
    //send the motors commands and the status to the yarp ports
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