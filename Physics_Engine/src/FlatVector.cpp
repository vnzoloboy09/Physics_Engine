#include "FlatVector.h"

FlatVector::FlatVector() {
	x = 0.0f;
	y = 0.0f;
}

FlatVector::FlatVector(float x, float y) {
	this->x = x;
	this->y = y;
}

FlatVector& FlatVector::Add(const FlatVector& vec) {
	this->x += vec.x;
	this->y += vec.y;

	return *this;
}

FlatVector& FlatVector::Subtract(const FlatVector& vec) {
	this->x -= vec.x;
	this->y -= vec.y;

	return *this;
}

FlatVector& FlatVector::Multipliy(const FlatVector& vec) {
	this->x *= vec.x;
	this->y *= vec.y;

	return *this;
}

FlatVector& FlatVector::Divide(const FlatVector& vec) {
	this->x -= vec.x;
	this->y -= vec.y;

	return *this;
}

FlatVector operator +(const FlatVector& v1, const FlatVector& v2) {
	return FlatVector(v1.x + v2.x, v1.y + v2.y);
}

FlatVector operator -(const FlatVector& v1, const FlatVector& v2) {
	return FlatVector(v1.x - v2.x, v1.y - v2.y);
}

FlatVector operator *(const FlatVector& v1, const FlatVector& v2) {
	return FlatVector(v1.x * v2.x, v1.y * v2.y);
}

FlatVector operator /(const FlatVector& v1, const FlatVector& v2) {
	if (v2.x == 0 || v2.y == 0) {
		__debugbreak();
		return FlatVector();
	}
	return FlatVector(v1.x / v2.x, v1.y / v2.y);
}

FlatVector& FlatVector::operator +=(const FlatVector& vec) {
	return this->Add(vec);
}

FlatVector& FlatVector::operator -=(const FlatVector& vec) {
	return this->Subtract(vec);
}

FlatVector& FlatVector::operator *=(const FlatVector& vec) {
	return this->Multipliy(vec);
}

FlatVector& FlatVector::operator /=(const FlatVector& vec) {
	return this->Divide(vec);
}

FlatVector FlatVector::operator *(const float& scalar) {
	return FlatVector(this->x * scalar, this->y * scalar);
}

FlatVector FlatVector::operator *(const float& scalar) const {
	return FlatVector(this->x * scalar, this->y * scalar);
}

FlatVector FlatVector::operator /(const float& scalar) {
	if (scalar == 0) {
		__debugbreak();
		return *this;
	}
	return FlatVector(this->x / scalar, this->y / scalar);
}

FlatVector FlatVector::operator -() const {
	return FlatVector(-this->x, -this->y);
}

FlatVector FlatVector::operator -() {
	return FlatVector(-this->x, -this->y);
}

FlatVector operator *(float scalar, const FlatVector& v) {
	return { v.x * scalar, v.y * scalar };
}

bool FlatVector::Equals(const FlatVector& other) const {
	return this->x == other.x && this->y == other.y;
}

bool FlatVector::operator==(const FlatVector& other) const {
	return this->Equals(other);
}

FlatVector& FlatVector::Zero() {
	this->x = 0.0f;
	this->y = 0.0f;

	return *this;
}

FlatVector FlatVector::Transform(FlatVector v, FlatTransform transform) {
	return FlatVector(transform.cos * v.x - transform.sin * v.y + transform.positionX, 
		transform.sin * v.x + transform.cos * v.y + transform.positionY);
}