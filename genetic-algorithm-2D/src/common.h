//--------------------------------------------------
// Genetic Algorithm 2D
// common.h    
// Date: 2021-11-19
// By Breno Cunha Queiroz  
//--------------------------------------------------
#ifndef COMMON_H
#define COMMON_H
#include <atta/pch.h>
#include <atta/componentSystem/componentManager.h>
#include <atta/componentSystem/components/transformComponent.h>
using namespace atta;

#define WORLD_SIZE 5

namespace common
{
    bool isInCollision(EntityId eid, TransformComponent* t)
    {
        bool inCollision = false;
        const float radius = t->scale.x/2.0f;

        // Check wall collision
        if(t->position.x > WORLD_SIZE-radius || 
            t->position.x < -WORLD_SIZE+radius ||
            t->position.y > WORLD_SIZE-radius || 
            t->position.y < -WORLD_SIZE+radius)
            inCollision = true;

        // Check obstacle collision
        Factory* factory = ComponentManager::getPrototypeFactory(10);
        for(EntityId obstacle : factory->getCloneIds())
        {
            TransformComponent* tc = ComponentManager::getEntityComponent<TransformComponent>(obstacle);

            float obsRadius = tc->scale.x/2.0f;
            float dist = (vec2(tc->position) - vec2(t->position)).length();

            if(dist <= radius + obsRadius)
            {
                inCollision = true;
                break;
            }
        }

        // Check robot collision
        factory = ComponentManager::getPrototypeFactory(9);
        for(EntityId robot :  factory->getCloneIds())
        {
            if(robot == eid)
                continue;
            TransformComponent* tc = ComponentManager::getEntityComponent<TransformComponent>(robot);

            float oRadius = tc->scale.x/2.0f;
            float dist = (vec2(tc->position) - vec2(t->position)).length();

            if(dist <= radius + oRadius)
            {
                inCollision = true;
                break;
            }
        }
        return inCollision;
    }

    float angleAverage(float angle0, float angle1)
    {
        float a = std::min(angle0, angle1);
        float b = std::max(angle0, angle1);
        if(b-a < M_PI)
            return (b+a)/2.0f;
        else
        {
            float res = (a+2*M_PI+b)/2.0f;
            if(res >= 2*M_PI)
                res -= 2*M_PI;
            return res;
        }
    }
}
#endif// COMMON_H
