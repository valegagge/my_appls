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

#ifndef JOYSTICKBUTTONS_H
#define JOYSTICKBUTTONS_H

#include <math.h>
#include <yarp/os/Bottle.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>


class joystickButtons
{
public :
    enum Axis
    {
        DUMMY              = 0,
        LEFT_VERTICAL      = 1, //left joystick button to move forward and back
        LEFT_HORIZONTAL    = 2,
        RIGHT_HORIZONTAL   = 3, //right joystick button to move left and right
        LEFT_SHIFT         = 4, //security button for robot move. here is not used
        RIGHT_VERTICAL     = 5,
        RIGHT_SHIFT        = 6, //security button for ball move
        DIGITAL_HORIZONTAL = 7,
        DIGITAL_VERTICAL   = 8
    };

    static const int axisNum = 10;
    static const int minButtonValue = 10;
    static const int minButtonCount = 10;
    double values[axisNum];
    int count[axisNum];

    joystickButtons()
    {
        for(int i=0; i<axisNum; i++)
        {
            values[i] = 0;
            count[i] = 0;
        }
    };

    bool isPressed(Axis a) //a button is pressed if the read value is bigger than minButtonValue
    {
        if (fabs(values[a]) < minButtonValue)
            return false;
        else
            return true;
    };


     bool isValid(Axis a) //if the button is pressed for a while
    {
        if (count[a] < minButtonValue)
            return false;
        else
            return true;
    };

};






class ballJoystickInterpreter
{
public :
    joystickButtons buttons;

    bool securityButtonIsPressed(void)
    {
         if( (!buttons.isPressed(joystickButtons::Axis::LEFT_SHIFT)) && (buttons.isPressed(joystickButtons::Axis::RIGHT_SHIFT)) )
             return true;
         else
             return false;
    };

    bool securityButtonIsValid(void)
    {
        if(buttons.isValid(joystickButtons::Axis::RIGHT_SHIFT))
            return true;
        else
            return false;
    };


    bool readValues(yarp::os::Bottle *b)
    {
        for (int j = 0; j < buttons.axisNum; j++)
        {
            buttons.values[j] = b->get(j).asDouble();
        }

        if(securityButtonIsPressed())
        {
            buttons.count[joystickButtons::Axis::RIGHT_SHIFT]++;
           // yError() << "securityButtonIsPressed!!!";
        }
        else
             buttons.count[joystickButtons::Axis::RIGHT_SHIFT] = 0;


        if(!securityButtonIsValid())
        {
            //yError() << "securityButton is NOT valid!!!";
            buttons.count[joystickButtons::Axis::RIGHT_VERTICAL] = 0;
            buttons.count[joystickButtons::Axis::RIGHT_HORIZONTAL] = 0;
            return true;
        }

        //yError() << "securityButtonIsValid!!!";
        if((buttons.isPressed(joystickButtons::Axis::RIGHT_VERTICAL)) && (buttons.isPressed(joystickButtons::Axis::RIGHT_HORIZONTAL)) )
        {
           buttons.count[joystickButtons::Axis::RIGHT_HORIZONTAL]++;
           buttons.count[joystickButtons::Axis::RIGHT_VERTICAL]++; //yError() << "both are pressed!!!";
        }
        else if(buttons.isPressed(joystickButtons::Axis::RIGHT_VERTICAL))
        {
            buttons.count[joystickButtons::Axis::RIGHT_HORIZONTAL]=0;
            buttons.count[joystickButtons::Axis::RIGHT_VERTICAL]++; //yError() << "right vert pressed!!!";
        }
        else if (buttons.isPressed(joystickButtons::Axis::RIGHT_HORIZONTAL))
        {
            buttons.count[joystickButtons::Axis::RIGHT_HORIZONTAL]++;
            buttons.count[joystickButtons::Axis::RIGHT_VERTICAL]=0; //yError() << "RIGHT_HORIZONTAL pressed!!!";
        }
        else
        {
            buttons.count[joystickButtons::Axis::RIGHT_HORIZONTAL]=0; //yError() << "NONE pressed!!!";
            buttons.count[joystickButtons::Axis::RIGHT_VERTICAL]=0;
        }
        return true;
    };

    bool buttonsValuesHasMeaning(void) //the application can use the values of buttons only if almost one of them is valid.
    {
        if( (!buttons.isValid(joystickButtons::Axis::RIGHT_HORIZONTAL)) && (!buttons.isValid(joystickButtons::Axis::RIGHT_VERTICAL)) )
            return false;
        else
            return true;
    }

    double getValue(joystickButtons::Axis a)
    {
        return buttons.values[a];
    }
};


#endif //JOYSTICKBUTTONS_H
