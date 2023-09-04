/* Start Header -----------------------------------------------------------------
File: BoundingUtils.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once

class Engine;

class FrameRateController
{

public:
	FrameRateController() {};
	~FrameRateController();


public:

	void InitializeFrameRateController();
	void SetFrameRateLimit(unsigned int Limit);
	void UpdateFrameTime();

	float TotalTime;
	float FixedDelta;
	float DeltaTime;
	float CurrentTime;
	float NewTime;
};

extern FrameRateController* g_FrameRateController;

