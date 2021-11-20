//--------------------------------------------------
// Genetic Algorithm 2D
// robotScript.h
// Date: 2021-11-17
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef ROBOT_SCRIPT_H
#define ROBOT_SCRIPT_H
#include <atta/pch.h>
#include <atta/scriptSystem/script.h>
#include <atta/componentSystem/components/transformComponent.h>
#include "geneComponent.h"

class RobotScript : public atta::Script
{
public:
    void update(atta::Entity entity, float dt) override;

private:
    float sensorActionResult(atta::EntityId eid, atta::TransformComponent* t, GeneComponent* g);
};

ATTA_REGISTER_SCRIPT(RobotScript)

#endif// ROBOT_SCRIPT_H
