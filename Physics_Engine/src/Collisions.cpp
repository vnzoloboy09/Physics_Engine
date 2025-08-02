#include "Collisions.h"
#include "FlatMath.h"
#include <limits>
#include <algorithm>
#include <iostream>

void Collisions::ProjectVertices(std::vector<FlatVector> vertices, FlatVector axis, float& _min, float& _max) {
	_min = std::numeric_limits<float>::max();
	_max = std::numeric_limits<float>::min();

	for (int i = 0; i < vertices.size(); i++) {
		FlatVector v = vertices[i];
		float projection = FlatMath::Dot(v, axis);

		if (projection < _min) { _min = projection; }
		if (projection > _max) { _max = projection; }
	}
}

void Collisions::ProjectCircle(FlatVector center, float radius, FlatVector axis, float& min, float& max) {
	FlatVector direction = FlatMath::Normalize(axis);
	FlatVector directionAndRadius = direction * radius;

	FlatVector point1 = center + directionAndRadius;
	FlatVector point2 = center - directionAndRadius;

	min = FlatMath::Dot(point1, axis);
	max = FlatMath::Dot(point2, axis);
	if (min > max) std::swap(min, max);
}

int Collisions::FindClosePointOnPolygon(FlatVector circleCenter, std::vector<FlatVector> vertices) {
	int result = -1;
	float minDistance = std::numeric_limits<float>::max();

	for (int i = 0; i < vertices.size(); i++) {
		float distance = FlatMath::Distance(vertices[i], circleCenter);
		if (minDistance > distance) {
			minDistance = distance;
			result = i;
		}
	}
	return result;
}

FlatVector Collisions::FindArithmeticMean(std::vector<FlatVector> vertices) {
	float sumX = 0.0f;
	float sumY = 0.0f;

	for (int i = 0; i < vertices.size(); i++) {
		sumX += vertices[i].x;
		sumY += vertices[i].y;
	}

	return FlatVector(sumX / vertices.size(), sumY / vertices.size());
}

bool Collisions::IntersectCircles(FlatVector centerA, float radiusA, 
	FlatVector centerB, float radiusB, FlatVector& normal, float& depth) 
{
	normal = FlatVector();
	depth = 0.0f; 

	float distance = FlatMath::Distance(centerA, centerB);
	float radii = radiusA + radiusB;

	if (distance >= radii) return false;
	
	normal = FlatMath::Normalize(centerB - centerA);
	depth = radii - distance;

	return true;
}

bool Collisions::IntersectPolygons(std::vector<FlatVector> verticesA, std::vector<FlatVector> verticesB,
	FlatVector& normal, float& depth) 
{
	normal = FlatVector();
	depth = std::numeric_limits<float>::max();

	for (int i = 0; i < verticesA.size(); i++) {
		FlatVector va = verticesA[i];
		FlatVector vb = verticesA[(i + 1) % verticesA.size()];
	
		FlatVector edge = vb - va;
		FlatVector axis = FlatVector(-edge.y, edge.x);
		axis = FlatMath::Normalize(axis);

		float minA, maxA, minB, maxB;

		ProjectVertices(verticesA, axis, minA, maxA);
		ProjectVertices(verticesB, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA) {
			return false;
		}
		float axisDepth = std::min(maxB- minA, maxA - minB);
		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}
	}

	for (int i = 0; i < verticesB.size(); i++) {
		FlatVector va = verticesB[i];
		FlatVector vb = verticesB[(i + 1) % verticesB.size()];

		FlatVector edge = vb - va;
		FlatVector axis = FlatVector(-edge.y, edge.x);
		axis = FlatMath::Normalize(axis);

		float minA, maxA, minB, maxB;

		ProjectVertices(verticesA, axis, minA, maxA);
		ProjectVertices(verticesB, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA) {
			return false;
		}

		if (minA >= maxB || minB >= maxA) {
			return false;
		}
		float axisDepth = std::min(maxB - minA, maxA - minB);
		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}
	}

	FlatVector centerA = FindArithmeticMean(verticesA);
	FlatVector centerB = FindArithmeticMean(verticesB);

	FlatVector direction = centerB - centerA;

	if (FlatMath::Dot(direction, normal) < 0.0f) {
		normal = -normal;
	}

	return true;
}

bool Collisions::IntersectPolygons(FlatVector centerA, std::vector<FlatVector> verticesA,
	FlatVector centerB, std::vector<FlatVector> verticesB, FlatVector& normal, float& depth) 
{
	normal = FlatVector();
	depth = std::numeric_limits<float>::max();

	for (int i = 0; i < verticesA.size(); i++) {
		FlatVector va = verticesA[i];
		FlatVector vb = verticesA[(i + 1) % verticesA.size()];

		FlatVector edge = vb - va;
		FlatVector axis = FlatVector(-edge.y, edge.x);
		axis = FlatMath::Normalize(axis);

		float minA, maxA, minB, maxB;

		ProjectVertices(verticesA, axis, minA, maxA);
		ProjectVertices(verticesB, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA) {
			return false;
		}
		float axisDepth = std::min(maxB - minA, maxA - minB);
		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}
	}

	for (int i = 0; i < verticesB.size(); i++) {
		FlatVector va = verticesB[i];
		FlatVector vb = verticesB[(i + 1) % verticesB.size()];

		FlatVector edge = vb - va;
		FlatVector axis = FlatVector(-edge.y, edge.x);
		axis = FlatMath::Normalize(axis);

		float minA, maxA, minB, maxB;

		ProjectVertices(verticesA, axis, minA, maxA);
		ProjectVertices(verticesB, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA) {
			return false;
		}

		if (minA >= maxB || minB >= maxA) {
			return false;
		}
		float axisDepth = std::min(maxB - minA, maxA - minB);
		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}
	}

	FlatVector direction = centerB - centerA;

	if (FlatMath::Dot(direction, normal) < 0.0f) {
		normal = -normal;
	}

	return true;
}

bool Collisions::IntersectCirclePolygon(FlatVector circleCenter, float cirleRadius,
	std::vector<FlatVector> vertices, FlatVector& normal, float& depth) 
{
	normal = FlatVector();
	depth = std::numeric_limits<float>::max();
	FlatVector axis = FlatVector();
	float axisDepth = 0.0f;
	float minA, maxA, minB, maxB;

	for (int i = 0; i < vertices.size(); i++) {
		FlatVector va = vertices[i];
		FlatVector vb = vertices[(i + 1) % vertices.size()];

		FlatVector edge = vb - va;
		axis = FlatVector(-edge.y, edge.x);
		axis = FlatMath::Normalize(axis);

		ProjectVertices(vertices, axis, minA, maxA);
		ProjectCircle(circleCenter, cirleRadius, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA) {
			return false;
		}

		axisDepth = std::min(maxB - minA, maxA - minB);

		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}
	}

	FlatVector closestPoint = vertices[FindClosePointOnPolygon(circleCenter, vertices)];
	axis = FlatMath::Normalize(closestPoint - circleCenter);

	ProjectVertices(vertices, axis, minA, maxA);
	ProjectCircle(circleCenter, cirleRadius, axis, minB, maxB);

	if (minA >= maxB || minB >= maxA) {
		return false;
	}

	axisDepth = std::min(maxB - minA, maxA - minB);

	if (axisDepth < depth) {
		depth = axisDepth;
		normal = axis;
 	}

	FlatVector polygonCenter = FindArithmeticMean(vertices);

	FlatVector direction = polygonCenter - circleCenter;

	if (FlatMath::Dot(direction, normal) < 0.0f) {
		normal = -normal;
	}
	
	return true;
}

bool Collisions::IntersectCirclePolygon(FlatVector circleCenter, float cirleRadius,
	FlatVector polygonCenter, std::vector<FlatVector> vertices, FlatVector& normal, float& depth) 
{
	normal = FlatVector();
	depth = std::numeric_limits<float>::max();
	FlatVector axis = FlatVector();
	float axisDepth = 0.0f;
	float minA, maxA, minB, maxB;

	for (int i = 0; i < vertices.size(); i++) {
		FlatVector va = vertices[i];
		FlatVector vb = vertices[(i + 1) % vertices.size()];

		FlatVector edge = vb - va;
		axis = FlatVector(-edge.y, edge.x);
		axis = FlatMath::Normalize(axis);

		ProjectVertices(vertices, axis, minA, maxA);
		ProjectCircle(circleCenter, cirleRadius, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA) {
			return false;
		}

		axisDepth = std::min(maxB - minA, maxA - minB);

		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}
	}

	FlatVector closestPoint = vertices[FindClosePointOnPolygon(circleCenter, vertices)];
	axis = FlatMath::Normalize(closestPoint - circleCenter);

	ProjectVertices(vertices, axis, minA, maxA);
	ProjectCircle(circleCenter, cirleRadius, axis, minB, maxB);

	if (minA >= maxB || minB >= maxA) {
		return false;
	}

	axisDepth = std::min(maxB - minA, maxA - minB);

	if (axisDepth < depth) {
		depth = axisDepth;
		normal = axis;
	}

	FlatVector direction = polygonCenter - circleCenter;

	if (FlatMath::Dot(direction, normal) < 0.0f) {
		normal = -normal;
	}

	return true;
}