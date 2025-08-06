#pragma once
#include <vector>
#include <memory> 
#include "raylib.h"
#include "FlatBody.h"

class FlatWorld {
public:
	static const float MIN_BODY_SIZE;  // m^2
	static const float MAX_BODY_SIZE;

	static const float MIN_DENSITY; // g/cm^3
	static const float MAX_DENSITY;

	static const int MIN_ITERATIONS = 1;
	static const int MAX_ITERATIONS = 128;

	FlatVector gravity;

	std::vector<std::unique_ptr<FlatBody>> bodyList;

public:
	FlatWorld();

	void AddBody(std::unique_ptr<FlatBody> body);
	void RemoveBody(const FlatBody* body);
	bool GetBody(int id, FlatBody*& body);
	void Step(int iterations, float dt);

	bool Collide(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& normal, float& depth);

	void ResolveCollision(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& normal, float& depth);

	int BodyCount() const;
};    
