//--------------------------------------------------
// Genetic Algorithm 2D
// projectScript.cpp
// Date: 2021-11-17
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "projectScript.h"
#include "common.h"
#include <atta/componentSystem/componentManager.h>
#include <atta/componentSystem/components/transformComponent.h>
#include "geneComponent.h"
#include "GAComponent.h"
#include <atta/graphicsSystem/drawer.h>
#include <imgui.h>
using namespace atta;

#define GA_EID 0
#define OBSTACLE_PROTOTYPE_EID 10
#define ROBOT_PROTOTYPE_EID 9

#define WORLD_SIZE 5

Project::Project():
    _maxIterationTime(10000), _currIterationTime(0), _running(false)
{

}

void Project::onStart()
{
	LOG_DEBUG("Project", "onStart");

    _running = true;
    // Reset GA component
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);
    ga->currGen = 1;
    ga->currEval = 1;
    ga->currEvalTime = 0;

    // Clear fitness vector
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
    ga->robotFitness.clear();
    ga->robotFitness.push_back(std::vector<float>(factory->getMaxClones()));

    randomizeObstacles();
    randomizeRobotsPositions();
    randomizeRobotsGenes();
}

void Project::onStop()
{
	LOG_DEBUG("Project", "onStop");
    Drawer::clear<Drawer::Line>(StringId("robotSensor"));
    _running = false;
}

void Project::onUpdateBefore(float delta)
{
    _running = true;
}

void Project::onUpdateAfter(float delta)
{
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);

    updateRobotsBounds();

    ga->currEvalTime += delta;
    if(ga->currEvalTime > ga->maxEvalTime)
    {
        // Finished one evaluation
        ga->currEvalTime = 0;
        updateRobotsFitness();
        randomizeObstacles();
        randomizeRobotsPositions();

        ga->currEval++;
        if(ga->currEval > ga->evalsPerGen)
        {
            // Finished also one generation
            ga->currEval = 1;
            crossRobots();
            mutateRobots();

            Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
            ga->robotFitness.push_back(std::vector<float>(factory->getMaxClones()));
            ga->currGen++;
        }
    }
}

void Project::onAttaLoop()
{
    // Clear robot sensor lines
    Drawer::clear<Drawer::Line>(StringId("robotSensor"));

    // Draw robot sensor lines
    if(_running)
    {
        Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
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
    Factory* factory = ComponentManager::getPrototypeFactory(OBSTACLE_PROTOTYPE_EID);
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

void Project::randomizeRobotsPositions()
{
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);

    // Reset bound vector
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);
    ga->robotBounds.resize(factory->getMaxClones());

    // For each robot
    unsigned i = 0;
    for(EntityId robot :  factory->getCloneIds())
    {
        TransformComponent* t = ComponentManager::getEntityComponent<TransformComponent>(robot);
        do
        {
            t->position.x = (rand()%8000/1000.0f)-4.0f;
            t->position.y = (rand()%8000/1000.0f)-4.0f;
            t->orientation.rotateAroundAxis(vec3(0,0,1), 2*(rand()%314/314.0f));
        } while(common::isInCollision(robot, t));

        ga->robotBounds[i].pMin = pnt2(t->position.x, t->position.y);
        ga->robotBounds[i].pMax = pnt2(t->position.x, t->position.y);
        i++;
    }
}

void Project::randomizeRobotsGenes()
{
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
    // For each robot
    for(EntityId robot :  factory->getCloneIds())
    {
        GeneComponent* gene = ComponentManager::getEntityComponent<GeneComponent>(robot);

        gene->linearVelocity = rand()%1000/1000.0f*GeneComponent::maxLinearVelocity;
        gene->angularVelocity = rand()%1000/1000.0f*GeneComponent::maxAngularVelocity;
        for(unsigned i = 0; i < GeneComponent::numSensors; i++)
        {
            gene->sensorAngle[i] = (rand()%1000/1000.0f)*2*M_PI;
            gene->sensorRange[i] = rand()%1000/1000.0f*GeneComponent::maxRange;
            gene->sensorAction[i] = (rand()%2000/1000.0f) - 1.0f;
        }
    }
}

void Project::updateRobotsBounds()
{
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);
    unsigned i = 0;
    for(EntityId robot : factory->getCloneIds())
    {
        TransformComponent* t = ComponentManager::getEntityComponent<TransformComponent>(robot);
        ga->robotBounds[i] = unionb(ga->robotBounds[i], pnt2(t->position.x, t->position.y));
        i++;
    }
}

void Project::updateRobotsFitness()
{
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);
    unsigned i = 0;
    for(EntityId robot : factory->getCloneIds())
    {
        float x = ga->robotBounds[i].pMax.x -ga->robotBounds[i].pMin.x;
        float y = ga->robotBounds[i].pMax.y -ga->robotBounds[i].pMin.y;
        float fitness = x*y/((WORLD_SIZE*2)*(WORLD_SIZE*2));

        ga->robotFitness.back()[i] = ((ga->robotFitness.back()[i]*(ga->currEval-1))+fitness)/float(ga->currEval);
        i++;
    }
}

void Project::crossRobots()
{
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);

    std::vector<float> robotFitness = ga->robotFitness.back();

    EntityId bestRobot = -1;
    float bestFitness = 0;
    for(unsigned i = 0; i < robotFitness.size(); i++)
        if(robotFitness[i] >= bestFitness)
        {
            bestRobot = EntityId(i);
            bestFitness = robotFitness[i];
        }

    LOG_SUCCESS("Project", "Generation finished, the best robot was [w]$0[], with fitness of [w]$1[]", factory->getFirstCloneId()+bestRobot, bestFitness);
    GeneComponent* bestGene = ComponentManager::getEntityComponent<GeneComponent>(factory->getFirstCloneId()+bestRobot);

    unsigned i = -1;
    for(EntityId robot : factory->getCloneIds())
    {
        i++;
        if(robot == bestRobot)
            continue;

        GeneComponent* gene = ComponentManager::getEntityComponent<GeneComponent>(robot);

        gene->linearVelocity = (gene->linearVelocity+bestGene->linearVelocity)/2.0f;
        gene->angularVelocity = (gene->angularVelocity+bestGene->angularVelocity)/2.0f;
        for(unsigned i = 0; i < GeneComponent::numSensors; i++)
        {
            gene->sensorAngle[i] = (gene->sensorAngle[i]+bestGene->sensorAngle[i])/2.0f;
            gene->sensorRange[i] = (gene->sensorRange[i]+bestGene->sensorRange[i])/2.0f;
            gene->sensorAction[i] = (gene->sensorAction[i]+bestGene->sensorAction[i])/2.0f;
        }
    }
}

void Project::mutateRobots()
{
    Factory* factory = ComponentManager::getPrototypeFactory(ROBOT_PROTOTYPE_EID);
    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);

    std::vector<float> robotFitness = ga->robotFitness.back();

    EntityId bestRobot = -1;
    float bestFitness = 0;
    for(unsigned i = 0; i < robotFitness.size(); i++)
        if(robotFitness[i] >= bestFitness)
        {
            bestRobot = EntityId(i);
            bestFitness = robotFitness[i];
        }

    unsigned i = -1;
    for(EntityId robot : factory->getCloneIds())
    {
        i++;
        if(robot == bestRobot)
            continue;

        if((rand()%1000/1000.0f) <= ga->mutationRate)
        {
            // Create randomGene
            GeneComponent randomGene;
            randomGene.linearVelocity = rand()%1000/1000.0f*GeneComponent::maxLinearVelocity;
            randomGene.angularVelocity = rand()%1000/1000.0f*GeneComponent::maxAngularVelocity;
            for(unsigned i = 0; i < GeneComponent::numSensors; i++)
            {
                randomGene.sensorAngle[i] = (rand()%1000/1000.0f)*2*M_PI;
                randomGene.sensorRange[i] = rand()%1000/1000.0f*GeneComponent::maxRange;
                randomGene.sensorAction[i] = (rand()%2000/1000.0f) - 1.0f;
            }

            // Mutate robot with random gene
            GeneComponent* gene = ComponentManager::getEntityComponent<GeneComponent>(robot);
            gene->linearVelocity = (gene->linearVelocity+randomGene.linearVelocity)/2.0f;
            gene->angularVelocity = (gene->angularVelocity+randomGene.angularVelocity)/2.0f;
            for(unsigned i = 0; i < GeneComponent::numSensors; i++)
            {
                gene->sensorAngle[i] = (gene->sensorAngle[i]+randomGene.sensorAngle[i])/2.0f;
                gene->sensorRange[i] = (gene->sensorRange[i]+randomGene.sensorRange[i])/2.0f;
                gene->sensorAction[i] = (gene->sensorAction[i]+randomGene.sensorAction[i])/2.0f;
            }
        }
    }
}

void Project::onUIRender()
{
    ImGui::Begin("Project");
    std::vector<float> bestPerGen;

    GAComponent* ga = ComponentManager::getEntityComponent<GAComponent>(GA_EID);
    for(auto robotsFitness : ga->robotFitness)
    {
        float m = 0;
        for(float fitness : robotsFitness)
            m = std::max(m, fitness);
        bestPerGen.push_back(m);
    }

    ImGui::Text("Best robot fitness");
    ImGui::PlotLines("###BestRobotFitness", bestPerGen.data(), bestPerGen.size(),
        0, NULL, 0.0f, 1.0f, ImVec2(1500.0f, 120.0f));

    ImGui::End();
}
