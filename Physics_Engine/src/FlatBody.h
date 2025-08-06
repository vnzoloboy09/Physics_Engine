#pragma once

#include "FlatVector.h"
#include "FlatAABB.h"
#include <string>
#include <vector>
#include <optional>
#include <memory>

class FlatWorld;

class FlatBody {
public:
	enum ShapeType {
		Circle = 0, 
		Box = 1
	};
	
	const float mass;
	const float invMass;
	const float density;
	const float restitution;
	const float area;
	const float radius;
	const float width;
	const float height;

	const bool b_IsStatic;
	const ShapeType shapeType;

	std::vector<int> triangles;
	const std::vector<FlatVector> vertices;

private:
	friend class FlatWorld;
	std::vector<FlatVector> transformVertices;
	std::unique_ptr<FlatAABB> aabb;
	
	bool b_TransformUpdateRequired;
	bool b_AabbUpdateRequired;

	FlatVector position;
	FlatVector linearVelovity;
	FlatVector force;
	float rotation;
	float rotaionVelocity;

private:
	static std::vector<FlatVector> CreateBoxVertices(int width, int height);
	static std::vector<int> CreateBoxTriangles();

public:
	FlatBody(FlatVector _position, float _density, float _mass, float _restitution, float _area,
		bool _b_IsStatic, float _radius, float _width, float _height, ShapeType shape);

	void Move(const FlatVector& amount);
	void MoveTo(const FlatVector& pos);
	void Rotate(const float& amount);
	void Step(FlatVector& gravity, const int& itertaions, float dt);
	void AddForce(FlatVector amount);

	FlatVector GetLinearVelocity() const;

	std::vector<FlatVector> GetTransformVertices();

	static std::optional<FlatBody> CreateCircleBody(float radius, FlatVector position, float density, 
		bool isStatic, float restitution);

	static std::optional<FlatBody> CreateBoxBody( float width, float height, FlatVector position, float density,
		bool b_IsStatic, float restitution);

	FlatAABB GetAABB();

	FlatVector GetPosition() const;

protected:
	void SetLinearVelocity(const FlatVector& value);

};