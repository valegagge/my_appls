
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

#include <iostream>
#include <iomanip>
 
#include <yarp/conf/environment.h>
#include <yarp/os/RFModule.h>

#include <TestGetPidApp.h>

using namespace std;
using namespace yarp::os;
 
double TestGetPidApp::getPeriod()
{
    /* module periodicity (seconds), called implicitly by the module. */
    return 0.1;
}
 
/* This is our main function. Will be called periodically every getPeriod() seconds */
bool TestGetPidApp::updateModule()
{
    for (auto p : all_parts) 
    {
        if(bodyPart_pidCtrlDev_map[p]->getPids(yarp::dev::VOCAB_PIDTYPE_POSITION, vectorOfPid))
        {
            yDebug() << "Error getting pid from " << bodyPart_name_map[p];
        }
    }

    return true;
}
 
    
bool TestGetPidApp::addPolyDriver(bodyPart_enum part)
{
    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/" + robot_name + "/" + "head");
    options.put("local", "/testGetPidApp/" + robot_name + "/" + bodyPart_name_map[part]);
    yarp::dev::PolyDriver* dd = new yarp::dev::PolyDriver(options);
    if(!dd->isValid())
    {
        yError() << "Error in opening the polyDriver for " + bodyPart_name_map[part];
        return false;
    }
    
    bodyPart_devDriver_map[part] = dd;

    yarp::dev::IPidControl* iPid;
    if(!dd->view(iPid))
    {
        yError() << "Error in opening the IPidControl interface for " + bodyPart_name_map[part];
        return false;
    }
    bodyPart_pidCtrlDev_map[part]=iPid;

    return true;
}

/* Configure function. Receive a previously initialized
    resource finder object. Use it to configure your module.
    If you are migrating from the old module, this is the function
    equivalent to the "open" method. */
bool TestGetPidApp::configure(yarp::os::ResourceFinder &rf)
{
    //robot_name= yarp::conf::environment::get_string("YARP_ROBOT_NAME", "iCubGenova02");
    robot_name="icub";
    for (auto p : all_parts) 
    {
        if(!addPolyDriver(p))
        {
            return false;
        }
    }
    
    return true;
}
 
 
/* Close function, to perform cleanup. */
bool TestGetPidApp::close()
{
    for (auto p : all_parts) 
    {
        if(bodyPart_devDriver_map[p])
        {
            delete(bodyPart_devDriver_map[p]);
            bodyPart_devDriver_map[p]=nullptr;
        }

    }
return true;
}
