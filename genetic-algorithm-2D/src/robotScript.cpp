//--------------------------------------------------
// Genetic Algorithm 2D
// robotScript.cpp
// Date: 2021-11-17
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "robotScript.h"
#include "common.h"
#include <atta/componentSystem/componentManager.h>
#include <atta/graphicsSystem/drawer.h>
using namespace atta;

#define WORLD_SIZE 5

void RobotScript::update(Entity entity, float dt)
{
    //----- Entity data -----//
    TransformComponent* t = entity.getComponent<TransformComponent>();
    GeneComponent* gene = entity.getComponent<GeneComponent>();
    float angle = -t->orientation.toEuler().z;
    float speed = gene->linearVelocity*dt;

    //----- Update position -----//
    t->position.x += cos(angle)*speed;
    t->position.y += sin(angle)*speed;

    // Solve collision
    if(common::isInCollision(entity.getId(), t))
    {
        t->position.x -= cos(angle)*speed;
        t->position.y -= sin(angle)*speed;
    }

    //----- Update sensor -----//
    // Rotate based on sensor input
    float sensorAction = sensorActionResult(entity.getId(), t, gene);
    t->orientation.rotateAroundAxis(vec3(0,0,1), sensorAction);
}

float RobotScript::sensorActionResult(EntityId eid, TransformComponent* t, GeneComponent* g)
{
    float sensorActionResult = 0;

    // TODO I don't think I should need to invert this angle...
    float robotAngle = -t->orientation.toEuler().z;
    for(unsigned i = 0; i < GeneComponent::numSensors; i++)
    {
        float sensorAngle = robotAngle+g->sensorAngle[i];
        float sx = t->position.x+cos(sensorAngle)*g->sensorRange[i];
        float sy = t->position.y+sin(sensorAngle)*g->sensorRange[i];
        vec2 ln = normalize(vec2(sx,sy)-vec2(t->position));

        bool sensorActivated = false;// If sensor detected something

        // Check if sensor detected a wall
        if(sx <= -WORLD_SIZE || sx >= WORLD_SIZE || sy <= -WORLD_SIZE || sy >= WORLD_SIZE)
            sensorActivated = true;

        // Check if sensor detected obstacle or robot
        Factory* factoryObstacles = ComponentManager::getPrototypeFactory(10);
        Factory* factoryRobots = ComponentManager::getPrototypeFactory(9);
        std::vector<EntityId> entitiesToCheck = factoryObstacles->getCloneIds();

        for(EntityId robot : factoryRobots->getCloneIds())
            if(robot != eid)
                entitiesToCheck.push_back(robot);

        if(!sensorActivated)
        {
            for(EntityId obstacle : entitiesToCheck)
            {
                TransformComponent* tc = ComponentManager::getEntityComponent<TransformComponent>(obstacle);

                float radius = tc->scale.x/2.0f;
                vec2 c = vec2(tc->position)-vec2(t->position);
                vec2 cn = c;
                cn.normalize();
                double d = dot(cn, ln);
                float dx = c.length()*d;
                float dy = c.length()*sqrt(1.0 - d*d);
                //if(eid == 17 && obstacle == 31 && i == 1)
                //    LOG_DEBUG("RObot", "dx:$0, dy:$1, s:$2, r:$3 -> c:$4 l:$5", dx, dy, g->sensorRange[i], radius, c, ln);

                if((dx <= g->sensorRange[i] && dx >= 0 && dy <= radius) ||// Line distance
                   (dx >= g->sensorRange[i] && (vec2(sx, sy)-vec2(tc->position)).length() <= radius))// End point distance
                {
                    sensorActivated = true;
                    //break;
                }
            }
        }

        if(sensorActivated)
            sensorActionResult -= g->sensorAction[i]*g->angularVelocity;
    }
    return sensorActionResult;
}
