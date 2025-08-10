#pragma once
#include <vector>
#include <tuple>

#include "raylib.h"
#include "FlatBody.h"
#include "FlatManifold.h"

class FlatWorld {
private:
	using ContactPair = std::tuple<int, int>;

	FlatVector gravity;
	std::vector<FlatBody*> bodyList;
	std::vector<ContactPair> contactPair;

public:
	static const float MIN_BODY_SIZE;  // m^2
	static const float MAX_BODY_SIZE;

	static const float MIN_DENSITY; // g/cm^3
	static const float MAX_DENSITY;

	static const int MIN_ITERATIONS = 1;
	static const int MAX_ITERATIONS = 128;

public:
	FlatWorld();
	~FlatWorld();

	void AddBody(FlatBody*& body);
	void RemoveBody(FlatBody*& body);
	bool GetBody(const int& id, FlatBody*& body);
	void Step(int iterations, float dt);
	size_t BodyCount() const;

private:
	void StepBodies(const int& totalItertaion, const float& dt);
	void BroadPhase();
	void NarrowPhase();
	void ResolveCollisionBasic(FlatManifold& contact);
	void ResolveCollisionWithRotation(FlatManifold& contact);
	void ResolveCollisionWithRotationAndFriction(FlatManifold& contact);
	void SeparateBodies(FlatBody*& bodyA, FlatBody*& bodyB, const FlatVector& mtv);
};    
