//--------------------------------------------------
// Genetic Algorithm 2D
// geneComponent.h    
// Date: 2021-11-18
// By Breno Cunha Queiroz  
//--------------------------------------------------
#ifndef GENE_COMPONENT_H
#define GENE_COMPONENT_H
#include <atta/pch.h>
#include <atta/componentSystem/components/component.h>
#include <atta/componentSystem/componentRegistry.h>

namespace atta
{      
    struct GeneComponent final : public Component
    {  
        static constexpr float maxLinearVelocity = 1.0f;
        static constexpr float maxAngularVelocity = 2*M_PI;
        static constexpr unsigned numSensors = 5;
        static constexpr float maxRange = 2.0f;

        float fitness;

        // Gene
        float linearVelocity;// Robot linar velocity (meters/s)
        float angularVelocity;// Robot angular velocity (radians/s)
        float sensorAngle[numSensors];// Angle for each sensor in radians (sensorAngle ∈ [0, 2π])
        float sensorRange[numSensors];// Maximum distance to trigger the sensor
        float sensorAction[numSensors];// If the sensor i was trigged, rotate by sensorAction[i]*angularVelocity[i]*dt (sensorAction ∈ [-1.0f,1.0f])
    }; 
    ATTA_REGISTER_COMPONENT(GeneComponent)
       
    template<>
    inline ComponentRegistry::Description TypedComponentRegistry<GeneComponent>::description = 
    {
        "Gene",
        {
            { ComponentRegistry::AttributeType::FLOAT32, offsetof(GeneComponent, fitness),          "fitness", 0.0f, 100.0f, 0.05f },
            { ComponentRegistry::AttributeType::FLOAT32, offsetof(GeneComponent, linearVelocity),   "linearVelocity", 0.0f, GeneComponent::maxLinearVelocity },
            { ComponentRegistry::AttributeType::FLOAT32, offsetof(GeneComponent, angularVelocity),  "angularVelocity", 0.0f, GeneComponent::maxAngularVelocity },
            { ComponentRegistry::AttributeType::FLOAT32, offsetof(GeneComponent, sensorAngle),      "sensorAngle", 0.0f, float(2*M_PI) },
            { ComponentRegistry::AttributeType::FLOAT32, offsetof(GeneComponent, sensorRange),      "sensorRange", 0.0f, GeneComponent::maxRange },
            { ComponentRegistry::AttributeType::FLOAT32, offsetof(GeneComponent, sensorAction),     "sensorAction", -1.0f, 1.0f },
        }
    }; 
}    
     
#endif// GENE_COMPONENT_H
