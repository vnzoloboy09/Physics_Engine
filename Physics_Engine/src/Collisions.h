#pragma once

#include "FlatVector.h"
#include"FlatBody.h"
#include <vector>

class Collisions {
private:
	static void ProjectVertices(const std::vector<FlatVector>& vertices, const FlatVector& axis, float& min, float& max);
	static void ProjectCircle(const FlatVector& center, const float& radius, const FlatVector& axis, float& min, float& max);
	static int FindClosePointOnPolygon(const FlatVector& circleCenter, const std::vector<FlatVector>& vertices);

public:
	static bool IntersectAABB(const FlatAABB& a, const FlatAABB& b);

	static bool IntersectCircles(const FlatVector& centerA, const float& radiusA,
		const FlatVector& centerB, const float& radiusB, FlatVector& normal, float& depth);

	static bool IntersectPolygons(const FlatVector& centerA, const std::vector<FlatVector>& verticesA,
		const FlatVector& centerB, const std::vector<FlatVector>& verticesB, FlatVector& normal, float& depth);

	static bool IntersectCirclePolygon(const FlatVector& circleCenter, const float& cirleRadius,
		const FlatVector& polygonCenter, const std::vector<FlatVector>& vertices, FlatVector& normal, float& depth);

	static void FindContactPoints(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& contact1, FlatVector& contact2, int& contactCount);

	static bool Collide(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& normal, float& depth);

	static void PointSegmentDistance(const FlatVector& p, const FlatVector& a, const FlatVector& b,
		float& distanceSquare, FlatVector& contact);
	
private:
	static void FindCircleContactPoint(const FlatVector& centerA, const float& radiusA,
		const FlatVector& centerB, FlatVector& contact);

	static void FindPolygonContactPoint(const std::vector<FlatVector> verticesA, const std::vector<FlatVector> verticesB, 
		FlatVector& contact1, FlatVector& contact2, int& contactCount);

	static void FindCirclePolygonContactPoint(const FlatVector& centerA, const float& radiusA,
		const FlatVector& centerB, const std::vector<FlatVector>& polygonVertices,FlatVector& contact);
};