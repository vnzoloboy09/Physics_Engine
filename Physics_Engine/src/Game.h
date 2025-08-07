#pragma once

#include "raylib.h"
#include "FlatBody.h"
#include "FlatWorld.h"
#include <vector>

class Game {
private:
	Camera2D camera = { 0 }; 
	Vector2 minCam, maxCam;
	FlatWorld* world = nullptr;

	double totalWorldTimeStep = 0;
	int totalBodyCount = 0;
	int totalSampleCount = 0;
	std::string worldStepTimeString;
	std::string bodyCountString;

public:
	Game();
	~Game();

	void Init();
	void HandleKeyInput();
	void HandleMouseInput();
	void HandleInput();
	void Update(float dt);
	void Render();
	void Quit();

	void WrapScreen();
};