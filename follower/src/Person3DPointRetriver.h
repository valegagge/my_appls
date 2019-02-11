/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file Person3DPointRetriver.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */
#include "TargetRetriver.h"
#include "AssistiveRehab/skeleton.h"

class Person3DPointRetriver : public TargetRetriver
{
public:
    Target_t getTarget(void);
private:
    assistive_rehab::SkeletonWaist m_sk_target;
};

