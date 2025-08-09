#pragma once

#include <random>

class Random {
public:
	Random(const Random&) = delete;

	static Random& Get();
	
	static float Float(const float& a, const float& b);
	static int Int(const int& a, const int& b);

private:
	Random();

	float i_Float(const float& a, const float& b);
	int i_Int(const int& a, const int& b);
};
