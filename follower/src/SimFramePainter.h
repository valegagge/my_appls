
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file SimFramePainter.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef SIMPAINTERS_H
#define SIMPAINTERS_H


#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>
#include <string>

namespace FollowerTarget
{

    class SimFramePainter
    {
        public:
            SimFramePainter(std::string name, std::string frameRef, yarp::os::RpcClient* worldPort, bool debugOn=false ):
            m_isCreated(false),
            m_nameOfFrame(name),
            m_worldInterfacePort_ptr(worldPort),
            m_frameIdOfRef(frameRef){;}
            void paint(const yarp::sig::Vector &point);
            void erase(void);
        private:
            bool m_isCreated;
            std::string m_nameOfFrame;
            yarp::os::RpcClient* m_worldInterfacePort_ptr; //shared pointer
            std::string m_frameIdOfRef;
            bool m_debugOn;
    };


    class SimManager
    {
        public:
            bool init(std::string robotName, std::string rpcNamePort);
            bool deinit(void);
            void PaintGazeFrame(const yarp::sig::Vector &point);
            void PaintTargetFrame(const yarp::sig::Vector &point);

        private:
            yarp::os::RpcClient m_worldInterfacePort;
            SimFramePainter * gazeFramePainter_ptr;
            SimFramePainter * targetFramePainter_ptr;
    };

}
//NOTE: What happen if I try to use a not opened port? I need a status variable?? (TODO check)

#endif
