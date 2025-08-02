#include "FlatMath.h"

#include <math.h>

float FlatMath::Clamp(float value, float min, float max) {
	if (min == max) return min;
	if (min > value) return min;
	if (max < value) return max;
}

int FlatMath::Clamp(int value, int min, int max) {
	if (min == max) return min;
	if (min > value) return min;
	if (max < value) return max;
}

float FlatMath::Length(FlatVector v) {
	return sqrt(v.x * v.x + v.y * v.y);
}

float FlatMath::Distance(FlatVector a, FlatVector b) {
	FlatVector d = a - b;
	return sqrt(d.x * d.x + d.y * d.y);
}

FlatVector FlatMath::Normalize(FlatVector v) {
	float magnitude = Length(v);
	if (magnitude == 0) return { 0.0f, 0.0f };
	return { v.x / magnitude, v.y / magnitude };
}

float FlatMath::Dot(FlatVector a, FlatVector b) {
	return a.x * b.x + b.y * a.y;
}

float FlatMath::Cross(FlatVector a, FlatVector b) {
	return a.x * b.y - a.y * b.x;
}