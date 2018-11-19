/*
 * Copyright (C)2018  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email:  valentina.gaggero@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <math.h>
#include <iostream>
#include "ballJoystickControl.h"


using namespace yarp::os;
using namespace std;

ballJoystickControl::ballJoystickControl():m_threadPeriod(0.01), m_ballName("sphere1"), m_ballIsCreated(false)
{;}

bool ballJoystickControl::configure(ResourceFinder &rf)
{
    string ctrlName;
    string robotName;
    string localName;

    // get params from the RF
    ctrlName = rf.check("local", Value("ballJoystickControl")).asString();
    robotName = rf.check("robot", Value("SIM_CER_ROBOT")).asString();

    //localName = "/" + ctrlName+ "/rpc";
    if(!m_worldInterfacePort.open("/" + ctrlName+ "/rpc"))
    {
        yError() << "Error opening rpc port!!";
        return false;
    }

    if(!m_port_joystick_input.open("/" + ctrlName+ "/joiystick:i"))
    {
        yError() << "Error opening input port!!";
        return false;
    }

    m_ballName = rf.check("ballName", Value("sphere1")).asString();

    m_gain_fowardBack = rf.check("gain_x_axis", Value(0.001)).asDouble();
    m_gain_leftRight = rf.check("gain_y_axis", Value(0.001)).asDouble();

    printCfg();


    return true;
}

bool ballJoystickControl::createBall(void)
{

    yDebug() << "I'm about to create the red ball...";
    Bottle cmd, ans;
    cmd.clear();
    ans.clear();
    //makeSphere 0.1 0 0 1.15 0 0 0 255 0 0 "" "pallina2"
    cmd.addString("makeSphere");
    cmd.addDouble(0.1); //radius
    cmd.addDouble(0); //x
    cmd.addDouble(0); //y
    cmd.addDouble(1.15); //z
    cmd.addDouble(0); //r
    cmd.addDouble(0); //p
    cmd.addDouble(0); //y
    cmd.addInt(255); //red
    cmd.addInt(0); //green
    cmd.addInt(0); //blue
    cmd.addString(""); //frame name
    cmd.addString(m_ballName);
    m_worldInterfacePort.write(cmd, ans);
    //yDebug() << "BALL-JOYSTICK-CONTROL: makeSphere= " << cmd.toString() << "  Ans=" << ans.toString();
    if(ans.toString() == m_ballName)
        m_ballIsCreated = true;
    else
        m_ballIsCreated = false;
    return m_ballIsCreated;

}
void ballJoystickControl::printCfg(void)
{
    yDebug() << "I'm about to start with following configuration: ";
    yDebug() << "ballName=" << m_ballName << " gain_fowardBack=" << m_gain_fowardBack << "gain_leftRight=" << m_gain_leftRight;
}

bool ballJoystickControl::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    reply.addString("Unknown command.");
    return true;
}

bool ballJoystickControl::close()
{
    m_worldInterfacePort.interrupt();
    m_worldInterfacePort.close();

    m_port_joystick_input.interrupt();
    m_port_joystick_input.close();
    return true;
}

double ballJoystickControl::getPeriod()
{
    return m_threadPeriod;
}

bool ballJoystickControl::updateModule()
{

    if( (!m_ballIsCreated) && (m_worldInterfacePort.asPort().getOutputCount() >0 ))
    {
        bool ret = createBall();
        if(!ret)
        {
            yWarning() << "ATTENTION: I could NOT create the ball!!";
            yWarning() << "Do you want continue anyway??";
            char input[255];
                cin >> input;
                if (input[0]!='y' && input[0]!='Y')
                {
                    m_ballIsCreated = true;
                    yInfo()<< "OK! You need to create a ball with name " << m_ballName;
                }
                else
                {
                    yInfo ( "Quitting...\n");
                    return false;
                }
        }
        else
        {
            yInfo() << "Ball has been created with name " << m_ballName << " !!";
        }

    }

    if (m_port_joystick_input.getInputCount() == 0)
    {
        return true;
    }

    Bottle *b = m_port_joystick_input.read(false); //false ==> doesn't wait data
    if(!b)
        return true;

    //read data of all axis
    if(!joystickMng.readValues(b))
        return true;

    if(!joystickMng.buttonsValuesHasMeaning())
        return true;

    //get value of interesing axis.
    double val_forward = joystickMng.getValue(joystickButtons::Axis::RIGHT_VERTICAL);
    double val_left_right = joystickMng.getValue(joystickButtons::Axis::RIGHT_HORIZONTAL);

    saturate(val_forward, 100);
    saturate(val_left_right, 100);

    val_forward = val_forward*m_gain_fowardBack;
    val_left_right = val_left_right*m_gain_leftRight;


    // Prapare bottle containg command to send in order to get the current position
    Bottle cmdGet, ansGet, cmdSet, ansSet;
    cmdGet.clear();
    ansGet.clear();
    cmdSet.clear();
    ansSet.clear();
    cmdGet.addString("getPose");
    cmdGet.addString(m_ballName);
    m_worldInterfacePort.write(cmdGet, ansGet);
    //read the answer
    double x = ansGet.get(0).asDouble();
    double y = ansGet.get(1).asDouble();
    //yDebug() << "BALL-JOYSTICK-CONTROL: cmd-GET= " << cmdGet.toString() << "  Ans=" << ansGet.toString();

    //Sum the calulated delta
    x+=val_forward;
    y+=val_left_right;

    //send command for new position
    cmdSet.addString("setPose");
    cmdSet.addString(m_ballName);
    cmdSet.addDouble(x);
    cmdSet.addDouble(y);
    cmdSet.addDouble(ansGet.get(2).asDouble()); // z
    cmdSet.addDouble(ansGet.get(3).asDouble()); // r
    cmdSet.addDouble(ansGet.get(4).asDouble()); // p
    cmdSet.addDouble(ansGet.get(5).asDouble()); // y
    m_worldInterfacePort.write(cmdSet, ansSet);
    //yDebug() << "BALL-JOYSTICK-CONTROL: cmd-SET= " << cmdSet.toString() << "  Ans=" << ansSet.toString();


}


void ballJoystickControl::saturate(double& v, double sat_lim)
{
    if (v < -sat_lim) v = -sat_lim;
    if (v > sat_lim) v = sat_lim;
}
