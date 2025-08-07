#include "Collisions.h"
#include "FlatMath.h"
#include <algorithm>

void Collisions::ProjectVertices(const std::vector<FlatVector>& vertices, const FlatVector& axis, float& _min, float& _max) {
	_min = FLT_MAX;
	_max = -FLT_MAX;

	for (auto& v : vertices) {
		float projection = FlatMath::Dot(v, axis);

		if (projection < _min) { _min = projection; }
		if (projection > _max) { _max = projection; }
	}
}

void Collisions::ProjectCircle(const FlatVector& center, const float& radius, const FlatVector& axis, float& min, float& max) {
	FlatVector direction = FlatMath::Normalize(axis);
	FlatVector directionAndRadius = direction * radius;

	FlatVector point1 = center + directionAndRadius;
	FlatVector point2 = center - directionAndRadius;

	min = FlatMath::Dot(point1, axis);
	max = FlatMath::Dot(point2, axis);
	if (min > max) std::swap(min, max);
}

int Collisions::FindClosePointOnPolygon(const FlatVector& circleCenter, const std::vector<FlatVector>& vertices) {
	int result = -1;
	float minDistance = FLT_MAX;

	for (int i = 0; i < vertices.size(); i++) {
		float distance = FlatMath::Distance(vertices[i], circleCenter);
		if (minDistance > distance) {
			minDistance = distance;
			result = i;
		}
	}
	return result;
}

bool Collisions::IntersectAABB(const FlatAABB& a, const FlatAABB& b) {
	if (a.max.x <= b.min.x || b.max.x < a.min.x ||
		a.max.y <= b.min.y || b.max.y < a.min.y) {
		return false;
	}

	return true;
}

bool Collisions::IntersectCircles(
	const FlatVector& centerA, const float& radiusA,
	const FlatVector& centerB, const float& radiusB, 
	FlatVector& normal, float& depth)
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

bool Collisions::IntersectPolygons(
	const FlatVector& centerA, const std::vector<FlatVector>& verticesA,
	const FlatVector& centerB, const std::vector<FlatVector>& verticesB, 
	FlatVector& normal, float& depth)
{
	normal = FlatVector();
	depth = FLT_MAX;

	for (int i = 0; i < verticesA.size(); i++) {
		const FlatVector& va = verticesA[i];
		const FlatVector& vb = verticesA[(i + 1) % verticesA.size()];

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
		const FlatVector& va = verticesB[i];
		const FlatVector& vb = verticesB[(i + 1) % verticesB.size()];

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

	FlatVector direction = centerB - centerA;

	if (FlatMath::Dot(direction, normal) < 0.0f) {
		normal = -normal;
	}

	return true;
}

bool Collisions::IntersectCirclePolygon(const FlatVector& circleCenter, const float& cirleRadius,
	const FlatVector& polygonCenter, const std::vector<FlatVector>& vertices, FlatVector& normal, float& depth)
{
	normal = FlatVector();
	depth = FLT_MAX;
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

void Collisions::FindContactPoints(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& contact1, FlatVector& contact2, int& contactCount) {
	FlatBody::ShapeType shapeTypeA = bodyA->shapeType;
	FlatBody::ShapeType shapeTypeB = bodyB->shapeType;
	
	contact1 = FlatVector();
	contact2 = FlatVector();
	contactCount = 0;

	if (shapeTypeA == FlatBody::ShapeType::Box) {
		if (shapeTypeB == FlatBody::ShapeType::Box) {
			FindPolygonContactPoint(bodyA->GetTransformVertices(), bodyB->GetTransformVertices(), contact1, contact2, contactCount);
		}
		else if (shapeTypeB == FlatBody::ShapeType::Circle) {
			FindCirclePolygonContactPoint(bodyB->GetPosition(), bodyB->radius, bodyA->GetPosition(), bodyA->GetTransformVertices(), contact1);
			contactCount = 1;
		}
	}
	else if (shapeTypeA == FlatBody::ShapeType::Circle) {
		if (shapeTypeB == FlatBody::ShapeType::Box) {
			FindCirclePolygonContactPoint(bodyA->GetPosition(), bodyA->radius, bodyB->GetPosition(), bodyB->GetTransformVertices(), contact1);
			contactCount = 1;
		}
		else if (shapeTypeB == FlatBody::ShapeType::Circle) {
			FindCircleContactPoint(bodyA->GetPosition(), bodyA->radius, bodyB->GetPosition(), contact1);
			contactCount = 1;
		}
	}
}

void Collisions::FindCircleContactPoint(const FlatVector& centerA, const float& radiusA,
	const FlatVector& centerB, FlatVector& contact)
{
	FlatVector ab = centerB - centerA;
	FlatVector direction = FlatMath::Normalize(ab);
	contact = centerA + direction * radiusA;
}

void Collisions::FindPolygonContactPoint(const std::vector<FlatVector> verticesA, const std::vector<FlatVector> verticesB,
	FlatVector& contact1, FlatVector& contact2, int& contactCount)
{
	float distanceSquared;
	FlatVector cp;
	float minDisSq = FLT_MAX;

	for (auto& p : verticesA) {
		for (int i = 0; i < verticesB.size(); i++) {
			const FlatVector& va = verticesB[i];
			const FlatVector& vb = verticesB[(i+1) % verticesB.size()];

			PointSegmentDistance(p, va, vb, distanceSquared, cp);

			if (FlatMath::NearlyEqual(distanceSquared, minDisSq)) {
				if (!FlatMath::NearlyEqual(cp, contact1)) {
					contact2 = cp;
					contactCount = 2;
				}
			}
			else if (distanceSquared < minDisSq) {
				minDisSq = distanceSquared;
				contact1 = cp;
				contactCount = 1;
			}
		}
	}

	for (auto& p : verticesB) {
		for (int i = 0; i < verticesA.size(); i++) {
			const FlatVector& va = verticesA[i];
			const FlatVector& vb = verticesA[(i + 1) % verticesA.size()];

			PointSegmentDistance(p, va, vb, distanceSquared, cp);

			if (FlatMath::NearlyEqual(distanceSquared, minDisSq)) {
				if (!FlatMath::NearlyEqual(cp, contact1)) {
					contact2 = cp;
					contactCount = 2;
				}
			}
			else if (distanceSquared < minDisSq) {
				minDisSq = distanceSquared;
				contact1 = cp;
				contactCount = 1;
			}
		}
	}
}

void Collisions::PointSegmentDistance(const FlatVector& p, const FlatVector& a, const FlatVector& b,
	float& distanceSquared, FlatVector& cp) 
{
	FlatVector ab = b - a;
	FlatVector ap = p - a;

	float proj = FlatMath::Dot(ap, ab);
	float abLenSq = FlatMath::LengthSquared(ab);
	float dis = proj / abLenSq;

	if (dis < 0.0f) {
		cp = a;
	}
	else if (dis > 1.0f) {
		cp = b;
	}
	else {
		cp = a + ab * dis;
	}

	distanceSquared = FlatMath::DistanceSquared(p, cp);
}

void Collisions::FindCirclePolygonContactPoint(const FlatVector& centerA, const float& radiusA,
	const FlatVector& centerB, const std::vector<FlatVector>& polygonVertices, FlatVector& outContact) 
{
	float distanceSquared;
	float minDistanceSquared = FLT_MAX;
	FlatVector contact;

	for (int i = 0; i < polygonVertices.size(); i++) {
		FlatVector va = polygonVertices[i];
		FlatVector vb = polygonVertices[(i + 1) % polygonVertices.size()];

		PointSegmentDistance(centerA, va, vb, distanceSquared, contact);

		if (distanceSquared < minDistanceSquared) {
			minDistanceSquared = distanceSquared;
			outContact = contact;
		}
	}
}

bool Collisions::Collide(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& normal, float& depth) {
	normal = FlatVector();
	depth = 0.0f;

	FlatBody::ShapeType shapeTypeA = bodyA->shapeType;
	FlatBody::ShapeType shapeTypeB = bodyB->shapeType;

	if (shapeTypeA == FlatBody::ShapeType::Box) {
		if (shapeTypeB == FlatBody::ShapeType::Box) {
			return IntersectPolygons(bodyA->GetPosition(), bodyA->GetTransformVertices(),
				bodyB->GetPosition(), bodyB->GetTransformVertices(), normal, depth);
		}
		else if (shapeTypeB == FlatBody::ShapeType::Circle) {
			bool result = IntersectCirclePolygon(bodyB->GetPosition(), bodyB->radius,
				bodyA->GetPosition(), bodyA->GetTransformVertices(), normal, depth);

			normal = -normal;
			return result;
		}
	}
	else if (shapeTypeA == FlatBody::ShapeType::Circle) {
		if (shapeTypeB == FlatBody::ShapeType::Box) {
			return IntersectCirclePolygon(bodyA->GetPosition(), bodyA->radius,
				bodyB->GetPosition(), bodyB->GetTransformVertices(), normal, depth);
		}
		else if (shapeTypeB == FlatBody::ShapeType::Circle) {
			return IntersectCircles(bodyA->GetPosition(), bodyA->radius,
				bodyB->GetPosition(), bodyB->radius, normal, depth);
		}
	}

	return false;
}
