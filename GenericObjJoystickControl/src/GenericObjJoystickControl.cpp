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
#include "GenericObjJoystickControl.h"


using namespace yarp::os;
using namespace std;



ObjectTypes::eValues ObjectTypes::string2value(std::string &str)
{
    if(str == ball_str) {return(eValues::ball);}
    else if(str == box_str){return(eValues::box);}
    else{return(eValues::none);}
}

std::string ObjectTypes::value2string(ObjectTypes::eValues val)
{
    switch(val)
    {
        case eValues::ball: return(ball_str);
        case eValues::box:  return(box_str);
        default:            return(none_str);
    }
}



GenericObjJoystickControl::GenericObjJoystickControl():m_threadPeriod(0.01), m_objName("myObject"), m_objIsCreated(false), m_objType(ObjectTypes::eValues::none)
{;}

bool GenericObjJoystickControl::configure(ResourceFinder &rf)
{
    string ctrlName;
    string robotName;
    string localName;

    // get params from the RF
    ctrlName = rf.check("local", Value("genericObjJoystickControl")).asString();
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

    m_objName = rf.check("objName", Value("myObject")).asString();
    string objtype_str = rf.check("create", Value(ObjectTypes::none_str.c_str())).asString();

    m_objType = ObjectTypes::string2value(objtype_str);

    m_gain_fowardBack = rf.check("gain_x_axis", Value(0.001)).asDouble();
    m_gain_leftRight = rf.check("gain_y_axis", Value(0.001)).asDouble();
    m_gain_rotation = rf.check("gain_yaw", Value(0.001)).asDouble();
    printCfg();


    return true;
}

bool GenericObjJoystickControl::createBall(void)
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
    cmd.addString(m_objName);
    m_worldInterfacePort.write(cmd, ans);
    //yDebug() << "BALL-JOYSTICK-CONTROL: makeSphere= " << cmd.toString() << "  Ans=" << ans.toString();
    if(ans.toString() == m_objName)
        m_objIsCreated = true;
    else
        m_objIsCreated = false;
    return m_objIsCreated;

}




bool GenericObjJoystickControl::createBox(void)
{

    yDebug() << "I'm about to create the red box...";
    Bottle cmd, ans;
    cmd.clear();
    ans.clear();
    //makeSphere 0.1 0 0 1.15 0 0 0 255 0 0 "" "pallina2"
    cmd.addString("makeBox");
    cmd.addDouble(0.1); //radius
    cmd.addDouble(0.2); //radius
    cmd.addDouble(0.3); //radius
    cmd.addDouble(2.0); //x
    cmd.addDouble(2.0); //y
    cmd.addDouble(0.15); //z
    cmd.addDouble(0); //r
    cmd.addDouble(0); //p
    cmd.addDouble(0); //y
    cmd.addInt(255); //red
    cmd.addInt(0); //green
    cmd.addInt(0); //blue
    cmd.addString(""); //frame name
    cmd.addString(m_objName);
    m_worldInterfacePort.write(cmd, ans);
    yDebug() << "BALL-JOYSTICK-CONTROL: makeBox= " << cmd.toString() << "  Ans=" << ans.toString();
    if(ans.toString() == m_objName)
        m_objIsCreated = true;
    else
        m_objIsCreated = false;
    return m_objIsCreated;

}
void GenericObjJoystickControl::printCfg(void)
{
    yDebug() << "I'm about to start with following configuration: ";
    yDebug() << "ObjName=" << m_objName << "ObjType=" << ObjectTypes::value2string(m_objType) << "gain_x_axis=" << m_gain_fowardBack << "gain_y_axis=" << m_gain_leftRight << "gain_yaw_rotation="<< m_gain_rotation;
}

bool GenericObjJoystickControl::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    reply.addString("Unknown command.");
    return true;
}

bool GenericObjJoystickControl::close()
{
    m_worldInterfacePort.interrupt();
    m_worldInterfacePort.close();

    m_port_joystick_input.interrupt();
    m_port_joystick_input.close();
    return true;
}

double GenericObjJoystickControl::getPeriod()
{
    return m_threadPeriod;
}

bool GenericObjJoystickControl::updateModule()
{

    if( (!m_objIsCreated) && (m_worldInterfacePort.asPort().getOutputCount() >0 ))
    {
        bool ret = false;
        switch(m_objType)
        {
            case ObjectTypes::eValues::ball: ret = createBall();break;
            case ObjectTypes::eValues::box: ret = createBox();break;
            default : ret = true; m_objIsCreated=true;//do nothing
        };


        if(!ret)
        {
            yWarning() << "ATTENTION: I could NOT create the object!!";
            yWarning() << "Do you want continue anyway??";
            char input[255];
                cin >> input;
                if (input[0]=='y' || input[0]=='Y')
                {
                    m_objIsCreated = true;
                    yInfo()<< "OK! You need to create an object with name " << m_objName;
                }
                else
                {
                    yInfo ( "Quitting...\n");
                    return false;
                }
        }
        else
        {
            if(m_objType != ObjectTypes::eValues::none)
                yInfo() << ObjectTypes::value2string(m_objType) << " has been created with name " << m_objName << " !!";
            else
                yInfo()  << "I didn't create any object. I'm ready to move " << m_objName << " !!";
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
    double val_forward = joystickMng.getValue(joystickButtons::Axis::LEFT_VERTICAL);
    double val_left_right = joystickMng.getValue(joystickButtons::Axis::LEFT_HORIZONTAL);
    double val_rotation = joystickMng.getValue(joystickButtons::Axis::RIGHT_HORIZONTAL);

    saturate(val_forward, 100);
    saturate(val_left_right, 100);
    saturate(val_rotation, 100);

    val_forward = val_forward*m_gain_fowardBack;
    val_left_right = val_left_right*m_gain_leftRight;
    val_rotation = val_rotation*m_gain_rotation;

//     // Prapare bottle containg command to send in order to get the current position
//     Bottle cmdGet, ansGet, cmdSet, ansSet;
//     cmdGet.clear();
//     ansGet.clear();
//     cmdSet.clear();
//     ansSet.clear();
//     cmdGet.addString("getPose");
//     cmdGet.addString(m_objName);
//     m_worldInterfacePort.write(cmdGet, ansGet);
//     //read the answer
//     double x = ansGet.get(0).asDouble();
//     double y = ansGet.get(1).asDouble();
//     double yaw = ansGet.get(5).asDouble();
//     //yDebug() << "BALL-JOYSTICK-CONTROL: cmd-GET= " << cmdGet.toString() << "  Ans=" << ansGet.toString();
//
//     //Sum the calulated delta
//     x+=val_forward;
//     y+=val_left_right;
//     yaw+=val_rotation;

    Bottle cmdSet, ansSet;
    //send command for new position
    cmdSet.addString("setPose");
    cmdSet.addString(m_objName);
    cmdSet.addDouble(val_forward);
    cmdSet.addDouble(val_left_right);
    cmdSet.addDouble(0); // z
    cmdSet.addDouble(0); // r
    cmdSet.addDouble(0); // p
    cmdSet.addDouble(val_rotation); // y
    cmdSet.addString(m_objName+linkstr);
    m_worldInterfacePort.write(cmdSet, ansSet);
   // yDebug() << "BALL-JOYSTICK-CONTROL: cmd-SET= " << cmdSet.toString() << "  Ans=" << ansSet.toString();

    return true;
}


void GenericObjJoystickControl::saturate(double& v, double sat_lim)
{
    if (v < -sat_lim) v = -sat_lim;
    if (v > sat_lim) v = sat_lim;
}
