#pragma once

#include "chai3d.h"
using namespace chai3d;

struct FillingSphere;
struct LinearSpring {
	LinearSpring(FillingSphere *sphere1, FillingSphere *sphere2, double length, double damping = 1, double stiffness = 500) :
		sphere1(sphere1), sphere2(sphere2), natural_length(length), damping(damping), stiffness(stiffness) {}

	void computeForce();

	double damping;
	double stiffness;
	double natural_length;
	FillingSphere *sphere1, *sphere2;
};
