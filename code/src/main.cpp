
/******************************************************************************
* Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/
/**
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <yarp/os/all.h>
#include <TestGetPidApp.h>


int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
 
    /* create your module */
    TestGetPidApp testgetpid;
 
    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);
 
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    testgetpid.runModule(rf);   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    yarp::os::Network::fini();
 
    return 0;
}