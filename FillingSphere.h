#pragma once

#include "chai3d.h"
using namespace chai3d;

struct FillingSphere {
	FillingSphere(double mass, cVector3d pos, cVector3d force = cVector3d(0, 0, 0)) :
		mass(mass), pos(pos), prev_pos(pos), user_force(0, 0, 0), env_force(force) {}

	void update(double dt);

	cVector3d pos, prev_pos;
	double prev_dt = -1.0;
	cVector3d force, user_force, env_force;
	double mass;
	bool isFixed = false;
};
