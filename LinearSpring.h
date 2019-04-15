#pragma once

#include "chai3d.h"
using namespace chai3d;

struct FillingSphere;
struct LinearSpring {
	LinearSpring(FillingSphere *sphere1, FillingSphere *sphere2, double length, double stiffness = 1000) :
		sphere1(sphere1), sphere2(sphere2), natural_length(length), stiffness(stiffness) {}

	void computeForce();

	double stiffness;
	double natural_length;
	FillingSphere *sphere1, *sphere2;
};
