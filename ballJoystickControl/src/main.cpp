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

#include "ballJoystickControl.h"


using namespace std;
using namespace yarp::os;





int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("SIM_CER_ROBOT");
    rf.setDefaultConfigFile("ballJoystickCtrl.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("Possible options: ");
        yInfo("'robot <name>' the robot name for remote connection.");
        yInfo("'local <name>' the local port name.");
        yInfo("'ballName <name>' the name used to identify the ball in gazebo. At startup the module try to craete a sphere with 'name'. In case of error ask to the use if he/she wanto to contiue.");
        yInfo("'gain_x_axis <value>' the gain to apply to the value read by joystick control to move the ball on X axis. Default value is 0.001");
        yInfo("'gain_y_axis <value>' the gain to apply to the value read by joystick control to move the ball on Y axis. Default value is 0.001.");

        yInfo("''");
        yInfo("example: BallJoystickCtrl --robot SIM_CER_ROBOT --ballName myBall --gain_x_axis 0.0004 --gain_y_axis 0.0004");
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
