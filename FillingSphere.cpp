#include "chai3d.h"
#include "FillingSphere.h"

using namespace chai3d;

void FillingSphere::update(double dt) {
	if (isFixed)
	{
		return;
	}
	cVector3d accel = force / mass;
	vel += dt * accel;
	pos += dt * vel;
}