#pragma once

#include <memory>
#include "raylib.h"
#include "FlatBody.h"
#include "FlatWorld.h"

class FlatEntity final {
public:
	FlatBody* body;
	Color color;

	FlatEntity(FlatBody*& _body);
	FlatEntity(FlatBody*& _body, const Color& color);

	FlatEntity(FlatWorld*& world, const float& radius, const bool& isStatic, const FlatVector& position);

	FlatEntity(FlatWorld*& world, const float& width, const float& height, const bool& isStatic, const FlatVector& postion);

	void Render(FlatBody::ShapeType shape);
};