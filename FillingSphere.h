#pragma once

#include "chai3d.h"
using namespace chai3d;

struct FillingSphere {
	FillingSphere(double mass, cVector3d pos) :
		mass(mass), pos(pos), vel(0, 0, 0), force(0, 0, 0) {}

	void update(double dt);

	cVector3d pos;
	cVector3d vel;
	cVector3d force;
	double mass;
	bool isFixed = false;
};
