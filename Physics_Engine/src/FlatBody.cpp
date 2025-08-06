#include "FlatBody.h"
#include "Def.h"
#include "FlatMath.h"
#include "FlatWorld.h"
#include <iostream>

FlatBody::FlatBody(FlatVector _position, float _density, float _mass, float _restitution, float _area,
	bool _b_IsStatic, float _radius, float w, float h, ShapeType shape) :
	position(_position),
	density(_density),
	mass(_mass),
	restitution(_restitution),
	area(_area),
	b_IsStatic(_b_IsStatic),
	radius(_radius),
	width(w),
	height(h),
	shapeType(shape),
	vertices(shape == ShapeType::Box? CreateBoxVertices(w, h) : std::vector<FlatVector>()),
	invMass(!_b_IsStatic ? 1.0f / _mass : 0.0f )
{
	force = FlatVector();

	if (shape == ShapeType::Box) {
		transformVertices.resize(vertices.size());
		triangles = CreateBoxTriangles();
	}
	else {
		triangles.resize(0);
		transformVertices.resize(0);
	}
	rotation = 0.0f;
	rotaionVelocity = 0.0f;
	b_TransformUpdateRequired = true;
	b_AabbUpdateRequired = true;
}

std::vector<FlatVector> FlatBody::CreateBoxVertices(int width, int height) {
	float left = -width / 2.0f;
	float right = left + width;
	float bottom = height / 2.0f;
	float top = bottom - height;

	std::vector<FlatVector> vectices(4);
	vectices[0] = { left, top };
	vectices[1] = { right, top };
	vectices[2] = { right, bottom };
	vectices[3] = { left, bottom };

	return vectices;
}

FlatVector FlatBody::GetLinearVelocity() const {
	return linearVelovity;
}

void FlatBody::SetLinearVelocity(const FlatVector& value) {
	linearVelovity = value;
}

void FlatBody::Move(const FlatVector& amount) {
	position += amount;
	b_TransformUpdateRequired = true;
	b_AabbUpdateRequired = true;
}

void FlatBody::MoveTo(const FlatVector& pos) {
	position = pos;
	b_TransformUpdateRequired = true;
	b_AabbUpdateRequired = true;
}

void FlatBody::Rotate(const float& amount) {
	rotation += amount;
	b_TransformUpdateRequired = true;
}

void FlatBody::Step(FlatVector& gravity, const int& iterations, float dt) {
	/*FlatVector acceleration = force / mass;
	linearVelovity += acceleration * dt;*/

	if (b_IsStatic) return;

	dt /= (float)iterations;

	linearVelovity += gravity * dt;

	position += linearVelovity * dt;
	rotation += rotaionVelocity * dt;

	force = { 0.0f, 0.0f };

	b_AabbUpdateRequired = true;
	b_TransformUpdateRequired = true;
}

void FlatBody::AddForce(FlatVector amount) {
	force = amount;
}

std::vector<FlatVector> FlatBody::GetTransformVertices() {
	if (b_TransformUpdateRequired) {
		FlatTransform transform = FlatTransform(position, rotation);

		for (int i = 0; i < vertices.size(); i++) {
			FlatVector v = vertices[i];
			transformVertices[i] = FlatVector::Transform(v, transform);
		}
	}

	return transformVertices;
}

std::optional<FlatBody> FlatBody::CreateCircleBody(float radius, FlatVector position, 
	float density, bool isStatic, float restitution) {
	float area = radius * radius * PI;
	if (area < FlatWorld::MIN_BODY_SIZE || area > FlatWorld::MAX_BODY_SIZE ||
		density < FlatWorld::MIN_DENSITY || density > FlatWorld::MAX_DENSITY)
		return std::nullopt;

	restitution = FlatMath::Clamp(restitution, 0.f, 1.f);
	float mass = area * density;

	return FlatBody(position, density, mass, restitution, area, isStatic, radius, 0.f, 0.f, ShapeType::Circle);
}

std::vector<int> FlatBody::CreateBoxTriangles() {
	std::vector<int> triangles(6);
	triangles[0] = 0;
	triangles[1] = 1;
	triangles[2] = 2;
	triangles[3] = 0;
	triangles[4] = 2;
	triangles[5] = 3;

	return triangles;
}

std::optional<FlatBody> FlatBody::CreateBoxBody(float width, float height, FlatVector position, float density,
	bool b_IsStatic, float restitution)
{
	float area = width * height;

	if (area < FlatWorld::MIN_BODY_SIZE || area > FlatWorld::MAX_BODY_SIZE ||
		density < FlatWorld::MIN_DENSITY || density > FlatWorld::MAX_DENSITY)
		return std::nullopt;

	restitution = FlatMath::Clamp(restitution, 0.0f, 1.0f);
	float mass = area * density;

	return FlatBody(position, density, mass, restitution, area,
		b_IsStatic, 0.0f, width, height, ShapeType::Box);
}

FlatAABB FlatBody::GetAABB() {
	if (b_AabbUpdateRequired) {
		float minX = FLT_MAX;
		float minY = FLT_MAX;
		float maxX = -FLT_MAX;
		float maxY = -FLT_MAX;
	
		if (shapeType == Box) {
			std::vector<FlatVector> vectices = GetTransformVertices();
			for (int i = 0; i < vectices.size(); i++) {
				FlatVector v = vectices[i];
				if (v.x < minX) minX = v.x;
				if (v.x > maxX) maxX = v.x;
				if (v.y < minY) minY = v.y;
				if (v.y > maxY) maxY = v.y;
			}
		}
		else if (shapeType == Circle) {
			minX = position.x - radius;
			minY = position.y - radius;
			maxX = position.x + radius;
			maxY = position.y + radius;
		}
		else {
			std::cerr << "Unkown shape type!\n";
		}
		aabb = std::make_unique<FlatAABB>(FlatAABB(minX, minY, maxX, maxY));
	}
	b_AabbUpdateRequired = false;
	return *aabb;
}

FlatVector FlatBody::GetPosition() const {
	return position;
}