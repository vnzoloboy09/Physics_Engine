#pragma once

#include "FlatVector.h"

static class FlatMath {
public:
	static float Clamp(float value, float min, float max);
	static int Clamp(int value, int min, int max);

	static float Length(FlatVector v);
	static float Distance(FlatVector a, FlatVector b);
	static FlatVector Normalize(FlatVector v);
	static float Dot(FlatVector a, FlatVector b);
	static float Cross(FlatVector a, FlatVector b);
};
