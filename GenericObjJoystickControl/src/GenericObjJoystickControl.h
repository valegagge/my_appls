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

#ifndef GENERICOBJJOYSTICKCONTROL_H
#define GENERICOBJJOYSTICKCONTROL_H

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include "joystick.h"

namespace ObjectTypes
{
    enum class eValues {ball, box, none};
    static const std::string ball_str = "ball";
    static const std::string box_str = "box";
    static const std::string none_str ="none";

    eValues string2value(std::string &str);
    std::string value2string(eValues val);
};


class GenericObjJoystickControl: public yarp::os::RFModule
{

public:
    GenericObjJoystickControl();

    bool configure(yarp::os::ResourceFinder &rf);

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool close();

    double getPeriod();

    bool   updateModule();

private:

    GeniricObjJoystickInterpreter joystickMng;

    double              m_threadPeriod;
    std::string         m_objName;
    joystickButtons     buttons;
    bool                m_objIsCreated;
    ObjectTypes::eValues m_objType;

    yarp::os::RpcClient m_worldInterfacePort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_joystick_input;

    double m_gain_fowardBack = 0.001;
    double m_gain_leftRight = 0.001;
    double m_gain_rotation = 0.001;
    const std::string linkstr="::link";

    void saturate(double& v, double sat_lim);
    void printCfg(void);
    bool createBall(void);
    bool createBox(void);

};

#endif //GENERICOBJJOYSTICKCONTROL_H
