#include "FrameRateController.h"
#include <GLFW/glfw3.h>

void FrameRateController::InitializeFrameRateController()
{
	TotalTime = NewTime = CurrentTime = DeltaTime = 0.0;
	FixedDelta = 0.016f;
}

void FrameRateController::SetFrameRateLimit(unsigned int Limit)
{
	DeltaTime = 1.0f / Limit;
	glfwSetTime(0);
	CurrentTime = (float)glfwGetTime();
}
void FrameRateController::UpdateFrameTime()
{
	NewTime = (float)glfwGetTime();
	DeltaTime = NewTime - CurrentTime;
	CurrentTime = NewTime;


	TotalTime += DeltaTime;
}

FrameRateController::~FrameRateController()
{
}
