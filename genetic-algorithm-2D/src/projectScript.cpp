//--------------------------------------------------
// Genetic Algorithm 2D
// projectScript.cpp
// Date: 2021-11-17
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "projectScript.h"
#include <atta/componentSystem/componentManager.h>
#include <atta/componentSystem/components/transformComponent.h>
#include "geneComponent.h"
#include <atta/graphicsSystem/drawer.h>
#include <imgui.h>
using namespace atta;

Project::Project():
    _maxIterationTime(10000), _currIterationTime(0), _running(false)
{

}

void Project::onStart()
{
	LOG_DEBUG("Project", "onStart");
    randomizeObstacles();
    _running = true;
}

void Project::onStop()
{
	LOG_DEBUG("Project", "onStop");
    Drawer::clear<Drawer::Line>(StringId("robotSensor"));
    _running = false;
}

void Project::onUpdateBefore(float delta)
{
    if(_currIterationTime >= _maxIterationTime)
    {
        _currIterationTime = 0;
        randomizeObstacles();
    }
    _currIterationTime += delta;
}

void Project::onAttaLoop()
{
    // Clear robot sensor lines
    Drawer::clear<Drawer::Line>(StringId("robotSensor"));

    // Draw robot sensor lines
    if(_running)
    {
        Factory* factory = ComponentManager::getPrototypeFactory(9);
        for(EntityId robot :  factory->getCloneIds())
        {
            TransformComponent* t = ComponentManager::getEntityComponent<TransformComponent>(robot);
            GeneComponent* gene = ComponentManager::getEntityComponent<GeneComponent>(robot);
            float angle = -t->orientation.toEuler().z;

            for(unsigned i = 0; i < GeneComponent::numSensors; i++)
            {
                float sensorAngle = angle+gene->sensorAngle[i];
                Drawer::add<Drawer::Line>(Drawer::Line(
                            vec3(t->position.x, t->position.y, 0.05), 
                            vec3(t->position.x+cos(sensorAngle)*gene->sensorRange[i], t->position.y+sin(sensorAngle)*gene->sensorRange[i], 0.05),
                            vec4(1,0,0,1),
                            vec4(1,0,0,1)),
                            StringId("robotSensor"));
            }
        }
    }
}

void Project::randomizeObstacles()
{
    // For each obstacle
    Factory* factory = ComponentManager::getPrototypeFactory(10);
    for(EntityId obstacle :  factory->getCloneIds())
    {
        TransformComponent* t = ComponentManager::getEntityComponent<TransformComponent>(obstacle);
        t->position.x = (rand()%8000/1000.0f)-4.0f;
        t->position.y = (rand()%8000/1000.0f)-4.0f;

        float scale = (rand()%1000/1000.0f)+0.3f;
        t->scale.x = scale;
        t->scale.y = scale;
    }
}

void Project::onUIRender()
{
    ImGui::Begin("Project");
    ImGui::Text("My project window");
    ImGui::End();
}
