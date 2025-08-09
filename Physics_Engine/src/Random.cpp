#include "Random.h"

Random::Random() {}

Random& Random::Get() {
	static Random s_Instance;
	return s_Instance;
}

float Random::i_Float(const float& a, const float& b) {
	return ((float)rand()) / ((float)RAND_MAX) * (b - a) + a;
}

float Random::Float(const float& a, const float& b) {
	return Get().i_Float(a, b);
}

int Random::i_Int(const int& a, const int& b) {
	return (rand() % (b - a)) + a;
}

int Random::Int(const int& a, const int& b) {
	return Get().i_Int(a, b);
}