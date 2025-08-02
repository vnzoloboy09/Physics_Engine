#include "FlatAABB.h"

FlatAABB::FlatAABB() : min({ 0.0f,0.0f }), max({ 0.0f,0.0f }) {}

FlatAABB::FlatAABB(FlatVector _min, FlatVector _max) :
	min(_min), max(_max) {}

FlatAABB::FlatAABB(float minX, float minY, float maxX, float maxY) :
	min(FlatVector(minX, minY)), max(FlatVector(maxX, maxY)) { }