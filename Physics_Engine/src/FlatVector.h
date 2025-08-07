#pragma once

#include "FlatTransform.h"

class FlatVector {
public:
	float x;
	float y;

	FlatVector();

	FlatVector(float x, float y);

	FlatVector& Add(const FlatVector& vec);
	FlatVector& Subtract(const FlatVector& vec);
	FlatVector& Multipliy(const FlatVector& vec);
	FlatVector& Divide(const FlatVector& vec);

	friend FlatVector operator +(const FlatVector& v1, const FlatVector& v2);
	friend FlatVector operator -(const FlatVector& v1, const FlatVector& v2);
	friend FlatVector operator *(const FlatVector& v1, const FlatVector& v2);
	friend FlatVector operator /(const FlatVector& v1, const FlatVector& v2);

	FlatVector& operator +=(const FlatVector& vec);
	FlatVector& operator -=(const FlatVector& vec);
	FlatVector& operator *=(const FlatVector& vec);
	FlatVector& operator /=(const FlatVector& vec);

	FlatVector operator -();

	FlatVector operator *(const float& scalar);
	FlatVector operator /(const float& scalar);
    friend FlatVector operator *(float scalar, const FlatVector& vec);

	bool Equals(const FlatVector& other) const;
	bool operator==(const FlatVector& other) const;

	FlatVector& Zero();

	static FlatVector Transform(FlatVector v, FlatTransform transform);
};