#pragma once

#include "FlatVector.h"
#include "FlatAABB.h"
#include <string>
#include <vector>
#include <optional>

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
	FlatAABB* aabb;
	
	bool b_TransformUpdateRequired;
	bool b_AabbUpdateRequired;

	FlatVector position;
	FlatVector linearVelovity;
	FlatVector force;
	float rotation;
	float rotaionVelocity;

private:
	FlatBody(FlatVector _position, float _density, float _mass, float _restitution, float _area,
		bool _b_IsStatic, float _radius, float _width, float _height, ShapeType shape);

	static std::vector<FlatVector> CreateBoxVertices(int width, int height);
	static std::vector<int> CreateBoxTriangles();

public:
	void Move(FlatVector amount);
	void MoveTo(FlatVector pos);
	void Rotate(float amount);
	void Step(FlatVector gravity, int itertaions, float dt);
	void AddForce(FlatVector amount);

	FlatVector GetLinearVelocity() const;

	std::vector<FlatVector> GetTransformVertices();

	static bool CreateCircleBody(float radius, FlatVector position, float density,
		bool isStatic, float resitution, FlatBody*& body, std::string& errorMessage);

	std::optional<FlatBody> CreateCircleBody(float radius, FlatVector position, float density, 
		bool isStatic, float restitution);

	std::optional<FlatBody> CreateBoxBody( float width, float height, FlatVector position, float density,
		bool b_IsStatic, float restitution, std::string& errorMessage);

	static bool CreateBoxBody(float width, float height, FlatVector position, float density,
		bool isStatic, float resitution, FlatBody*& body, std::string& errorMessage);

	FlatAABB GetAABB();

	FlatVector GetPosition() const;

protected:
	void SetLinearVelocity(const FlatVector& value);

};