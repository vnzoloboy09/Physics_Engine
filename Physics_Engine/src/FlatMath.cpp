#include "FlatMath.h"

#include <math.h>

const float FlatMath::SMALL_AMOUNT = 0.0005f; // 1/2 of a milimeter

float FlatMath::Clamp(float& value, const float& min, const float& max) {
	if (min == max) return min;
	if (min > value) return min;
	if (max < value) return max;
	return value;
}

int FlatMath::Clamp(int& value, const int& min, const int& max) {
	if (min == max) return min;
	if (min > value) return min;
	if (max < value) return max;
	return value;
}

float FlatMath::Length(const FlatVector& v) {
	return sqrt(v.x * v.x + v.y * v.y);
}

float FlatMath::LengthSquared(const FlatVector& v) {
	return v.x * v.x + v.y * v.y;
}

float FlatMath::Distance(const FlatVector& a, const FlatVector& b) {
	FlatVector d = a - b;
	return sqrt(d.x * d.x + d.y * d.y);
}

float FlatMath::DistanceSquared(const FlatVector& a, const FlatVector& b) {
	FlatVector d = a - b;
	return d.x * d.x + d.y * d.y;
}

FlatVector FlatMath::Normalize(const FlatVector& v) {
	float magnitude = FlatMath::Length(v);
	if (magnitude < 1e-6f) return { 0.0f, 0.0f }; // Avoid division by near-zero
	return { v.x / magnitude, v.y / magnitude };
}

float FlatMath::Dot(const FlatVector& a, const FlatVector& b) {
	return a.x * b.x + b.y * a.y;
}

float FlatMath::Cross(const FlatVector& a, const FlatVector& b) {
	return a.x * b.y - a.y * b.x;
}

bool FlatMath::NearlyEqual(const float& a, const float& b) {
	return std::abs(a - b) < SMALL_AMOUNT;
}

bool FlatMath::NearlyEqual(const FlatVector& a, const FlatVector& b) {
	return FlatMath::DistanceSquared(a, b) < SMALL_AMOUNT * SMALL_AMOUNT;
}