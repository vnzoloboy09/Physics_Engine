#include "FlatConverter.h"

Vector2 FlatConverter::ToVector2(FlatVector v) {
	Vector2 vector2 = { v.x, v.y };
	return vector2;
}

FlatVector FlatConverter::ToFlatVector(Vector2 v) {
	return FlatVector(v.x, v.y);
}

std::vector<Vector2> FlatConverter::ToVector2List(const std::vector<FlatVector>& flatVertices)
{
    std::vector<Vector2> result(flatVertices.size());
	for (int i = 0; i < flatVertices.size(); i++) {
		result[i] = { flatVertices[i].x, flatVertices[i].y };
	}
	return result;
}

