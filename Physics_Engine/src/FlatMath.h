#pragma once

#include "FlatVector.h"

class FlatMath {
public:
	static const float SMALL_AMOUNT; 

public:
	static float Clamp(float& value, const float& min, const float& max);
	static int Clamp(int& value, const int& min, const int& max);

	static float Length(const FlatVector& v);
	static float LengthSquared(const FlatVector& v);
	static float Distance(const FlatVector& a, const FlatVector& b);
	static float DistanceSquared(const FlatVector& a, const FlatVector& b);
	static FlatVector Normalize(const FlatVector& v);
	static float Dot(const FlatVector& a, const FlatVector& b);
	static float Cross(const FlatVector& a, const FlatVector& b);
	static bool NearlyEqual(const float& a, const float& b);
	static bool NearlyEqual(const FlatVector& a, const FlatVector& b);
};
