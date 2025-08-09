#pragma once

#include "raylib.h"
#include "FlatEntity.h"
#include <vector>
#include <string>

class Game {
private:
	Camera2D camera = { 0 }; 
	Vector2 minCam, maxCam;
	FlatWorld* world = nullptr;

	double totalWorldTimeStep = 0;
	size_t totalBodyCount = 0;
	int totalSampleCount = 0;
	std::string worldStepTimeString;
	std::string bodyCountString;

	std::vector<FlatEntity*> entities;
	std::vector<FlatEntity*> removalEntities;

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
};