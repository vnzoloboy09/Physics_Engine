#pragma once

#include "FlatVector.h"

class FlatMath {
public:
	static float Clamp(float& value, const float& min, const float& max);
	static int Clamp(int& value, const int& min, const int& max);

	static float Length(const FlatVector& v);
	static float Distance(const FlatVector& a, const FlatVector& b);
	static FlatVector Normalize(const FlatVector& v);
	static float Dot(const FlatVector& a, const FlatVector& b);
	static float Cross(const FlatVector& a, const FlatVector& b);
};
