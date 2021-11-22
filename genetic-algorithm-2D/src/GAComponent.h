//--------------------------------------------------
// Genetic Algorithm 2D
// GAComponent.h    
// Date: 2021-11-19
// By Breno Cunha Queiroz  
//--------------------------------------------------
#ifndef GA_COMPONENT_H
#define GA_COMPONENT_H
#include <atta/pch.h>
#include <atta/core/math/math.h>
#include <atta/componentSystem/components/component.h>
#include <atta/componentSystem/componentRegistry.h>
#include <atta/fileSystem/serializer/serializer.h>
using namespace atta;

struct GAComponent final : public Component
{  
    std::vector<std::vector<float>> robotFitness;// For each generation, fitness of each robot
    std::vector<bnd2f> robotBounds;// For each robot, the area explored
    float mutationRate;
    uint32_t crossingType;
    uint32_t fitnessSmooth;
    uint32_t predationInterval;

    uint32_t currGen;// Current generation
    uint32_t currEval;// Current evaluation
    uint32_t evalsPerGen;// Number of evaluations per generation
    float currEvalTime;// Current evaluation time in seconds
    float maxEvalTime;// Maximum evaluation time in seconds
}; 
ATTA_REGISTER_COMPONENT(GAComponent);
   
template<>
inline ComponentRegistry::Description TypedComponentRegistry<GAComponent>::description = 
{
    "GA",
    {
        { ComponentRegistry::AttributeType::CUSTOM, offsetof(GAComponent, robotFitness), "robotFitness" },
        { ComponentRegistry::AttributeType::CUSTOM, offsetof(GAComponent, robotBounds), "robotBounds" },
        { ComponentRegistry::AttributeType::FLOAT32, offsetof(GAComponent, mutationRate), "mutationRate", 0.0f, 1.0f, 0.05f },
        { ComponentRegistry::AttributeType::UINT32, offsetof(GAComponent, crossingType), "crossingType", {}, {}, {}, 
            {"Best fitness", "Best smooth"} },
        { ComponentRegistry::AttributeType::UINT32, offsetof(GAComponent, fitnessSmooth), "fitnessSmooth", 1u, 10u },
        { ComponentRegistry::AttributeType::UINT32, offsetof(GAComponent, predationInterval), "predationInterval", 1u, 50u },

        { ComponentRegistry::AttributeType::UINT32, offsetof(GAComponent, currGen), "currGen", 1u, 1000u },
        { ComponentRegistry::AttributeType::UINT32, offsetof(GAComponent, currEval), "currEval", 1u, 50u },
        { ComponentRegistry::AttributeType::UINT32, offsetof(GAComponent, evalsPerGen), "evalsPerGen", 1u, 50u },
        { ComponentRegistry::AttributeType::FLOAT32, offsetof(GAComponent, currEvalTime), "currEvalTime", 0.0f, 60.0f},
        { ComponentRegistry::AttributeType::FLOAT32, offsetof(GAComponent, maxEvalTime), "maxEvalTime", 0.0f, 60.0f},
    },
    // Max instances
    1,
    // Serialize
    {
        {"robotFitness", [](std::ostream& os, void* data)
            {
                std::vector<std::vector<float>>* robotFitness = static_cast<std::vector<std::vector<float>>*>(data);
                for(auto generation : *robotFitness) 
                {
                    for(float fitness : generation) 
                        write(os, fitness);               
                    write(os, float(-1));        
                }
                write(os, float(-2));        
            }
        },
        {"robotBounds", [](std::ostream& os, void* data)
            {
                std::vector<bnd2f>* robotBounds = static_cast<std::vector<bnd2f>*>(data);
                for(bnd2f robotBound : *robotBounds)
                    write(os, robotBound);
                bnd2f b;
                b.pMin = pnt2(-100,-100);
                b.pMax = pnt2(-100,-100);
                write(os, b);
            }
        }
    },
    // Deserialize
    {
        {"robotFitness", [](std::istream& is, void* data)
            {
                std::vector<std::vector<float>>* robotFitness = static_cast<std::vector<std::vector<float>>*>(data);
                float fitness;
                read(is, fitness);
                while(fitness != -2.0f)                
                {
                    if(fitness == -1.0f)
                        robotFitness->push_back(std::vector<float>());
                    else
                        robotFitness->back().push_back(fitness);       
                    read(is, fitness);
                }          
            }
        },
        {"robotBounds", [](std::istream& is, void* data)
            {
                std::vector<bnd2f>* robotBounds = static_cast<std::vector<bnd2f>*>(data);
                bnd2f bound;
                read(is, bound);
                while(bound.pMin.x != -100)
                {
                    robotBounds->push_back(bound);       
                    read(is, bound);
                }          
            }
        }
    }
}; 
     
#endif// GA_COMPONENT_H
