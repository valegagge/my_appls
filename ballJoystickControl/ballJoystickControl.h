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

#ifndef BALLJOYSTICKCONTROL_H
#define BALLJOYSTICKCONTROL_H

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

class ballJoystickControl: public yarp::os::RFModule
{

public:
    ballJoystickControl();

    bool configure(yarp::os::ResourceFinder &rf);

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool close();

    double getPeriod();

    bool   updateModule();

private:

    double              m_threadPeriod;
    std::string         m_ballName;

    yarp::os::RpcClient m_worldInterfacePort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_joystick_input;

    double m_gain_fowardBack = 0.0001; //??? da decidere ==> ora sposto di 10 cm
    double m_gain_leftRight = 0.0001; //??? da decidere ==> ora sposto di 10 cm


    void saturate(double& v, double sat_lim);

};

#endif //BALLJOYSTICKCONTROL_H
