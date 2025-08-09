#include "FlatBody.h"
#include "Def.h"
#include "FlatMath.h"
#include "FlatWorld.h"

FlatBody::FlatBody(const float& _density, const float& _mass, const float& _inertia, const float& _restitution, const float& _area,
	const bool& _b_IsStatic, const float& _radius, const float& _width, const float& _height, 
	const std::vector<FlatVector>& _vertices, const ShapeType& shape) :
	position(FlatVector()),
	density(_density),
	mass(_mass),
	invMass(!_b_IsStatic ? 1.0f / mass : 0.0f ),
	restitution(_restitution),
	area(_area),
	b_IsStatic(_b_IsStatic),
	radius(_radius),
	width(_width),
	height(_height),
	shapeType(shape),
	vertices(_vertices),
	inertia(_inertia),
	invInertia(!b_IsStatic ? 1.0f / inertia : 0.0f)
{
	force = FlatVector();

	if (shape == ShapeType::Box) {
		transformVertices.resize(vertices.size());
	}
	else {
		transformVertices.resize(0);
	}
	angle = 0.0f;
	angularVelocity = 0.0f;
	b_TransformUpdateRequired = true;
	b_AabbUpdateRequired = true;
}

FlatBody::FlatBody(const FlatBody& other) :
	position(other.position),
	density(other.density),
	mass(other.mass),
	invMass(other.invMass),
	restitution(other.restitution),
	area(other.area),
	b_IsStatic(other.b_IsStatic),
	radius(other.radius),
	width(other.width),
	height(other.height),
	shapeType(other.shapeType),
	vertices(other.vertices),
	inertia(other.inertia),
	invInertia(other.invInertia),
	force(other.force),
	transformVertices(other.transformVertices),
	angle(other.angle),
	angularVelocity(other.angularVelocity),
	b_TransformUpdateRequired(other.b_TransformUpdateRequired),
	b_AabbUpdateRequired(other.b_AabbUpdateRequired)
{}

FlatBody::FlatBody(FlatBody&& other) noexcept :
	position(std::move(other.position)),
	density(other.density),
	mass(other.mass),
	invMass(other.invMass),
	restitution(other.restitution),
	area(other.area),
	b_IsStatic(other.b_IsStatic),
	radius(other.radius),
	width(other.width),
	height(other.height),
	shapeType(other.shapeType),
	vertices(other.vertices),
	inertia(other.inertia),
	invInertia(other.invInertia),
	force(std::move(other.force)),
	transformVertices(std::move(other.transformVertices)),
	angle(other.angle),
	angularVelocity(other.angularVelocity),
	b_TransformUpdateRequired(other.b_TransformUpdateRequired),
	b_AabbUpdateRequired(other.b_AabbUpdateRequired)
{}

std::vector<FlatVector> FlatBody::CreateBoxVertices(const float& width, const float& height) {
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
	angle += amount;
	b_TransformUpdateRequired = true;
	b_AabbUpdateRequired = true;
}

void FlatBody::RotateTo(const float& ang) {
	angle = ang;
	b_TransformUpdateRequired = true;
	b_AabbUpdateRequired = true;
}

void FlatBody::Step(FlatVector& gravity, const int& iterations, float dt) {
	/*FlatVector acceleration = force / mass;
	linearVelovity += acceleration * dt;*/

	if (b_IsStatic) return;

	dt /= (float)iterations;

	linearVelovity += gravity * dt;

	position += linearVelovity * dt;
	angle += angularVelocity * dt;

	force = { 0.0f, 0.0f };

	b_AabbUpdateRequired = true;
	b_TransformUpdateRequired = true;
}

void FlatBody::AddForce(FlatVector amount) {
	force = amount;
}

std::vector<FlatVector> FlatBody::GetTransformVertices() {
	if (b_TransformUpdateRequired) {
		FlatTransform transform = FlatTransform(position, angle);

		for (int i = 0; i < vertices.size(); i++) {
			FlatVector v = vertices[i];
			transformVertices[i] = FlatVector::Transform(v, transform);
		}
	}

	return transformVertices;
}

bool FlatBody::CreateCircleBody(float radius, float density,bool b_IsStatic, float restitution, FlatBody*& body) {
	body = nullptr;

	float area = radius * radius * PI;

	if (area < FlatWorld::MIN_BODY_SIZE || area > FlatWorld::MAX_BODY_SIZE ||
		density < FlatWorld::MIN_DENSITY || density > FlatWorld::MAX_DENSITY) {
		return false;
	}

	restitution = FlatMath::Clamp(restitution, 0.0f, 1.0f);

	float mass = 0.0f;
	float inertia = 0.0f;

	if (!b_IsStatic) {
		mass = area * density;
		inertia = (1.0f / 2.0f) * mass * radius * radius;
	}

	body = new FlatBody(density, mass, inertia, restitution, area,
		b_IsStatic, radius, 0.0f, 0.0f, std::vector<FlatVector>{}, ShapeType::Circle);

	return  true;
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

bool FlatBody::CreateBoxBody(float width, float height, float density, bool b_IsStatic, float restitution, FlatBody*& body)
{
	float area = width * height;

	if (area < FlatWorld::MIN_BODY_SIZE || area > FlatWorld::MAX_BODY_SIZE ||
		density < FlatWorld::MIN_DENSITY || density > FlatWorld::MAX_DENSITY)
		return false;

	restitution = FlatMath::Clamp(restitution, 0.0f, 1.0f);
	float mass = 0.0f;
	float inertia = 0.0f;

	if (!b_IsStatic) {
		mass = area * density;
		inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
	}

	std::vector<FlatVector> vertices = CreateBoxVertices(width, height);
	body = new FlatBody(density, mass, inertia, restitution, area, b_IsStatic, 0.0f, width, height, vertices, ShapeType::Box);

	return true;
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
			__debugbreak();
		}
		aabb = std::make_unique<FlatAABB>(FlatAABB(minX, minY, maxX, maxY));
	}
	b_AabbUpdateRequired = false;
	return *aabb;
}

FlatVector FlatBody::GetPosition() const {
	return position;
}