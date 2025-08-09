#pragma once
#include <vector>
#include <memory> 
#include "raylib.h"
#include "FlatBody.h"
#include "FlatManifold.h"

class FlatWorld {
private:
	FlatVector gravity;
	std::vector<FlatBody*> bodyList;
	std::vector<FlatManifold*> contactList;

public:
	static const float MIN_BODY_SIZE;  // m^2
	static const float MAX_BODY_SIZE;

	static const float MIN_DENSITY; // g/cm^3
	static const float MAX_DENSITY;

	static const int MIN_ITERATIONS = 1;
	static const int MAX_ITERATIONS = 128;

	std::vector<FlatVector> contactPointsList;

public:
	FlatWorld();

	void AddBody(FlatBody*& body);
	void RemoveBody(FlatBody*& body);
	bool GetBody(const int& id, FlatBody*& body);
	void Step(int iterations, float dt);
	void SeparateBodies(FlatBody*& bodyA, FlatBody*& bodyB, const FlatVector& mtv);

	void ResolveCollision(FlatManifold*& contact);

	size_t BodyCount() const;
};    
