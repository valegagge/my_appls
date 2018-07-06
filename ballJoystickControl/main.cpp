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
#include <yarp/os/Network.h>



#include <yarp/os/ResourceFinder.h>

#include <yarp/os/Log.h>



//#include <iostream>
//#include <iomanip>
//#include <string>

#include "ballJoystickControl.h"


using namespace std;
using namespace yarp::os;





int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("SIM_CERROBOT");
    rf.setDefaultConfigFile("ballJoystickCtrl.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("Possible options: ");
        yInfo("'robot <name>' the robot name for remote connection.");
        yInfo("'local <name>' the local port name.");
        yInfo("'rate <r>' sets the threads rate (default 20ms).");
        yInfo("'joystick_connect' tries to automatically connect to the joystickCtrl output.");

        yInfo("''");
        yInfo("example: BallJoystickCtrl --robot SIM_CER_ROBOT --joystick_connect ");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    ballJoystickControl mod;

    return mod.runModule(rf);
}
