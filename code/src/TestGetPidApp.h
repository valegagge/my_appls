
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

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/os/all.h>
#include <map>
#include <string>

enum bodyPart_enum
{
    head      = 0,
    left_arm  = 1,
    right_arm = 2,
    torso     = 3,
    left_leg  = 4,
    right_leg = 5
};

class TestGetPidApp:public yarp::os::RFModule
{
    
private:
    std::map<bodyPart_enum, yarp::dev::PolyDriver* > bodyPart_devDriver_map;
    std::map<bodyPart_enum, yarp::dev::IPidControl* > bodyPart_pidCtrlDev_map;
    static constexpr std::initializer_list<bodyPart_enum> all_parts = {bodyPart_enum::head, bodyPart_enum::left_arm, bodyPart_enum::right_arm, bodyPart_enum::torso, bodyPart_enum::left_leg, bodyPart_enum::right_leg };

    /*static constexpr*/ std::map<bodyPart_enum, std::string> bodyPart_name_map = {{bodyPart_enum::head, "head"},
                                                                {bodyPart_enum::left_arm, "left_arm"},
                                                                {bodyPart_enum::right_arm, "right_arm"},
                                                                {bodyPart_enum::torso, "torso"},
                                                                {bodyPart_enum::left_leg, "left_leg"},
                                                                {bodyPart_enum::right_leg, "right_leg"} };
    yarp::dev::Pid vectorOfPid[20]{};
    std::string robot_name;

    bool addPolyDriver(bodyPart_enum part);

public:
 
    double getPeriod();
 
    bool updateModule();
 
    bool configure(yarp::os::ResourceFinder &rf); 

    bool close();
};