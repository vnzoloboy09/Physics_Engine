#include "FlatTransform.h"
#include "FlatVector.h"

FlatTransform::FlatTransform(FlatVector position, float angle) :
	positionX(position.x),
	positionY(position.y),
	sin(std::sin(angle)),
	cos(std::cos(angle)) {
}

FlatTransform::FlatTransform(float x, float y, float angle) :
	positionX(x),
	positionY(y),
	sin(std::sin(angle)),
	cos(std::cos(angle)) {
}

FlatTransform::FlatTransform() :
	positionX(0.0f),
	positionY(0.0f),
	sin(std::sin(0.0f)),
	cos(std::cos(0.0f)) {
}
