#include "FlatManifold.h"

FlatManifold::FlatManifold(FlatBody* bdA, FlatBody* bdB, FlatVector nor, float dep,
	FlatVector ct1, FlatVector ct2, int ctCount) :
	bodyA(bdA),
	bodyB(bdB),
	normal(nor),
	depth(dep),
	contact1(ct1),
	contact2(ct2),
	contactCount(ctCount) 
{}

FlatManifold::~FlatManifold() = default;