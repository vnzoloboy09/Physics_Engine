#pragma once

#include "FlatVector.h"
#include "raylib.h"
#include <vector>

class FlatConverter {
public:
	static Vector2 ToVector2(FlatVector v);
	static FlatVector ToFlatVector(Vector2 v);
	static std::vector<Vector2> ToVector2List(const std::vector<FlatVector>& flatVertices);
};