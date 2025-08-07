#pragma once

#include "FlatBody.h"
#include "FlatVector.h"
#include <memory.h>

class FlatManifold {
public:
	FlatBody* bodyA;
	FlatBody* bodyB;
	const FlatVector normal;
	const float depth;

	const FlatVector contact1;
	const FlatVector contact2;
	const int contactCount;

	FlatManifold(FlatBody* bodyA, FlatBody* bodyB, FlatVector nor, float dep,
		FlatVector ct1, FlatVector ct2, int ctCount);
	~FlatManifold();
};