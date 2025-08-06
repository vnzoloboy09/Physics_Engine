#pragma once

#include "FlatVector.h"
#include <vector>

class Collisions {
private:
	static void ProjectVertices(const std::vector<FlatVector>& vertices, const FlatVector& axis, float& min, float& max);
	static void ProjectCircle(FlatVector center, float radius, FlatVector axis, float& min, float& max);
	static int FindClosePointOnPolygon(FlatVector circleCenter, std::vector<FlatVector> vertices);
	static FlatVector FindArithmeticMean(std::vector<FlatVector> vertices);

public:
	static bool IntersectCircles(const FlatVector& centerA, const float& radiusA, 
		const FlatVector& centerB, const float& radiusB, FlatVector& normal, float& depth);

	static bool IntersectPolygons(const std::vector<FlatVector>& verticesA, const std::vector<FlatVector>& verticesB,
		FlatVector& normal, float& depth);

	static bool IntersectPolygons(const FlatVector& centerA, const std::vector<FlatVector>& verticesA, 
		const FlatVector& centerB, const std::vector<FlatVector>& verticesB, FlatVector& normal, float& depth);

	static bool IntersectCirclePolygon(const FlatVector& circleCenter, const float& cirleRadius,
		const std::vector<FlatVector>& vertices, FlatVector& normal, float& depth);

	static bool IntersectCirclePolygon(const FlatVector& circleCenter, const float& cirleRadius,
		const FlatVector& polygonCenter, const std::vector<FlatVector>& vertices, FlatVector& normal, float& depth);
};