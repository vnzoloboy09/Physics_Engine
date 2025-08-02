#pragma once

#include "FlatVector.h"
#include <vector>

static class Collisions {
private:
	static void ProjectVertices(std::vector<FlatVector> vertices, FlatVector axis, float& min, float& max);
	static void ProjectCircle(FlatVector center, float radius, FlatVector axis, float& min, float& max);
	static int FindClosePointOnPolygon(FlatVector circleCenter, std::vector<FlatVector> vertices);
	static FlatVector FindArithmeticMean(std::vector<FlatVector> vertices);

public:
	static bool IntersectCircles(FlatVector centerA, float radiusA, 
		FlatVector centerB, float radiusB, FlatVector& normal, float& depth);

	static bool IntersectPolygons(std::vector<FlatVector> verticesA, std::vector<FlatVector> verticesB,
		FlatVector& normal, float& depth);

	static bool IntersectPolygons(FlatVector centerA, std::vector<FlatVector> verticesA, 
		FlatVector centerB, std::vector<FlatVector> verticesB, FlatVector& normal, float& depth);

	static bool IntersectCirclePolygon(FlatVector circleCenter, float cirleRadius,
		std::vector<FlatVector> vertices, FlatVector& normal, float& depth);

	static bool IntersectCirclePolygon(FlatVector circleCenter, float cirleRadius,
		FlatVector polygonCenter, std::vector<FlatVector> vertices, FlatVector& normal, float& depth);
};