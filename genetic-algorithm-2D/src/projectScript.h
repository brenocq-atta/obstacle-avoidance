//--------------------------------------------------
// Genetic Algorithm 2D
// projectScript.h
// Date: 2021-11-17
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef PROJECT_SCRIPT_H
#define PROJECT_SCRIPT_H
#include <atta/pch.h>
#include <atta/scriptSystem/projectScript.h>

class Project : public atta::ProjectScript
{
public:
    Project();

	void onStart() override;
	void onStop() override;
	void onUpdateBefore(float delta) override;

    void onUIRender() override;

    void onAttaLoop() override;
private:
    void randomizeObstacles();

    const float _maxIterationTime;
    float _currIterationTime;
    bool _running;
};

ATTA_REGISTER_PROJECT_SCRIPT(Project)

#endif// PROJECT_SCRIPT_H
