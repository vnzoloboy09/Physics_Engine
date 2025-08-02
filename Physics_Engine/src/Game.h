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

public:
	Game();
	~Game();

	void Init();
	void HandleKeyInput();
	void HandleMouseInput();
	void HandleInput();
	void Update(float dt);
	void Redner();
	void Quit();

	void WrapScreen();
};