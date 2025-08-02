#pragma once

#include "FlatVector.h"

class FlatAABB {
public:
	const FlatVector min;
	const FlatVector max;

	FlatAABB();
	FlatAABB(FlatVector min, FlatVector max);
	FlatAABB(float minX, float minY, float maxX, float maxY);
};