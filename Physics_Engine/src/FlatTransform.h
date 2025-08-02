#pragma once

class FlatVector;
#include <cmath>

class FlatTransform {
public:
	const float positionX;
	const float positionY;
	const float sin;
	const float cos;

public:
	FlatTransform(FlatVector position, float angle);
	FlatTransform(float x, float y, float angle);
	FlatTransform();
};