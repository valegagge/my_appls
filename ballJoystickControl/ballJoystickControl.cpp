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
#include "ballJoystickControl.h"

using namespace yarp::os;
using namespace std;

enum joystickAxis
{

    DUMMY              = 0,
    LEFT_VERTICAL      = 1, //left joystick button to move forward and back
    LEFT_HORIZONTAL    = 2,
    RIGHT_HORIZONTAL   = 3, //right joystick button to move left and right
    LEFT_SHIFT         = 4, //security button for ball move. here is not used
    RIGHT_VERTICAL     = 5,
    RIGHT_SHIFT        = 6, //security button for ball move
    DIGITAL_HORIZONTAL = 7,
    DIGITAL_VERTICAL   = 8
};

#define AXIS_NUM               10

ballJoystickControl::ballJoystickControl():m_threadPeriod(0.01), m_ballName("sphere1")
{
;
}

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

    m_gain_fowardBack = rf.check("gain_fowardBack", Value(0.1)).asDouble();
    m_gain_leftRight = rf.check("gain_leftRight", Value(0.1)).asDouble();

    return true;
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

inline bool buttonIsPressed(double *data, joystickAxis ja)
{
    if (fabs(data[ja]) < 10)
        return false;
    else
        return true;
}

bool ballJoystickControl::updateModule()
{
    if (m_port_joystick_input.getInputCount() == 0)
    {
        return true;
    }

    double axis[20];
    static int rightSecurityButtonCount = 0;
    static int rightButtonCount = 0;
    static int leftButtonCount = 0;
    Bottle *b = m_port_joystick_input.read(false); //false ==> doesn't wait data
    if(!b)
        return true;

    //read data of all axis
    for (int j = 0; j < AXIS_NUM; j++)
    {
        axis[j] = b->get(j).asDouble();
    }

    #define RIGHT_SECURITY_BUTTON_IS_PUSHED  ((fabs(axis[joystickAxis::LEFT_SHIFT]) < 10 && fabs(axis[joystickAxis::RIGHT_SHIFT]) > 10))
    #define RIGHT_SECURITY_BUTTON_THRESHOLD  10

    if(RIGHT_SECURITY_BUTTON_IS_PUSHED)
        rightSecurityButtonCount++;
    else
        rightSecurityButtonCount = 0;


    if(rightSecurityButtonCount < RIGHT_SECURITY_BUTTON_THRESHOLD)
    {
        rightButtonCount=0;
        leftButtonCount=0;
        return true;
        yError() << "sec button is not pressed";
    }

    if((buttonIsPressed(axis, joystickAxis::LEFT_VERTICAL)) && (buttonIsPressed(axis, joystickAxis::RIGHT_HORIZONTAL)) )
    {
        rightButtonCount++; leftButtonCount++; yError() << "both are pressed";
    }
    else if(buttonIsPressed(axis, joystickAxis::LEFT_VERTICAL))
    {
        rightButtonCount=0; leftButtonCount++;  yError() << "LEFT is  pressed";
    }
    else if (buttonIsPressed(axis, joystickAxis::RIGHT_HORIZONTAL))
    {
        rightButtonCount++; leftButtonCount=0;  yError() << "RIGHT is  pressed";
    }
    else
    {
        rightButtonCount=0;
        leftButtonCount=0;yError() << "NONE are pressed";
    }

    if((rightButtonCount < RIGHT_SECURITY_BUTTON_THRESHOLD) && (leftButtonCount < RIGHT_SECURITY_BUTTON_THRESHOLD) )
    {
//         rightButtonCount=0;
//         leftButtonCount=0;
        yError() << "both are NOT PRESSED ENOUGHT";
        return true;
    }

    //get value of interesing axis.
    double val_forward = axis[joystickAxis::LEFT_VERTICAL];
    double val_left_right = axis[joystickAxis::RIGHT_HORIZONTAL];

    saturate(val_forward, 100);
    saturate(val_left_right, 100);

    val_forward = val_forward*m_gain_fowardBack;
    val_left_right = val_left_right*m_gain_leftRight;


    // Prapare bottle containg command to send
    Bottle cmdGet, ansGet, cmdSet, ansSet;
    cmdGet.addString("getPose");
    cmdGet.addString(m_ballName);
    double x = ansGet.get(0).asDouble();
    double y = ansGet.get(1).asDouble();
    m_worldInterfacePort.write(cmdGet, ansGet);
    yDebug() << "BALL-JOYSTICK-CONTROL: cmd-GET= " << cmdGet.toString() << "  Ans=" << ansGet.toString();

    x+=val_forward;
    y+=val_left_right;

    cmdSet.addString("setPose");
    cmdSet.addString(m_ballName);
    cmdSet.addDouble(x);
    cmdSet.addDouble(y);
    cmdSet.addDouble(ansGet.get(2).asDouble());
    cmdSet.addDouble(ansGet.get(3).asDouble());
    cmdSet.addDouble(ansGet.get(4).asDouble());
    cmdSet.addDouble(ansGet.get(5).asDouble());
    m_worldInterfacePort.write(cmdSet, ansSet);
    yDebug() << "BALL-JOYSTICK-CONTROL: cmd-SET= " << cmdSet.toString() << "  Ans=" << ansSet.toString();


}


void ballJoystickControl::saturate(double& v, double sat_lim)
{
    if (v < -sat_lim) v = -sat_lim;
    if (v > sat_lim) v = sat_lim;
}
