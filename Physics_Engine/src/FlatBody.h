#pragma once

#include "FlatVector.h"
#include "FlatAABB.h"
#include <vector>
#include <memory>

class FlatWorld;

class FlatBody {
public:
	enum ShapeType {
		Circle = 0, 
		Box = 1
	};
	
	const ShapeType shapeType;
	const float mass;
	const float invMass;
	const float density;
	const float restitution;
	const float area;
	const float radius;
	const float width;
	const float height;
	const bool b_IsStatic;
	const float inertia;
	const float invInertia;

	const std::vector<FlatVector> vertices;
	float angle;
	float angularVelocity;

private:
	friend class FlatWorld;
	std::vector<FlatVector> transformVertices;
	std::unique_ptr<FlatAABB> aabb;
	
	bool b_TransformUpdateRequired;
	bool b_AabbUpdateRequired;

	FlatVector position;
	FlatVector linearVelovity;
	FlatVector force;

private:
	static std::vector<FlatVector> CreateBoxVertices(const float& width, const float& height);
	static std::vector<int> CreateBoxTriangles();

public:
	FlatBody(const float& _density, const float& _mass, const float& inertia, const float& _restitution, const float& _area,
		const bool& _b_IsStatic, const float& _radius, const float& _width, const float& _height, 
		const std::vector<FlatVector>& vertices, const ShapeType& shape);
	
	FlatBody(const FlatBody& other);
	FlatBody(FlatBody&& other) noexcept;

	void Move(const FlatVector& amount);
	void MoveTo(const FlatVector& pos);
	void Rotate(const float& amount);
	void RotateTo(const float& angle);
	void Step(FlatVector& gravity, const int& itertaions, float dt);
	void AddForce(FlatVector amount);

	FlatVector GetLinearVelocity() const;

	std::vector<FlatVector> GetTransformVertices();

	static bool CreateCircleBody(float radius, float density, bool b_IsStatic, float restitution, FlatBody*& body);

	static bool CreateBoxBody(float width, float height, float density, bool b_IsStatic,  float restitution, FlatBody*& body);

	FlatAABB GetAABB();

	FlatVector GetPosition() const;

protected:
	void SetLinearVelocity(const FlatVector& value);
};