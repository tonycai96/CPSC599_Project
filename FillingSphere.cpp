#include "chai3d.h"
#include "FillingSphere.h"

using namespace chai3d;

void FillingSphere::update(double dt) {
	if (isFixed)
	{
		return;
	}
	for (int i = 0; i < 10; i++) {
		cVector3d a = force / mass;
		cVector3d tmp_pos = pos;
		cVector3d delta_pos = (pos - prev_pos) + a * dt * dt;
		pos = pos + 0.9 * delta_pos;
		prev_pos = tmp_pos;
	}
	prev_dt = dt;
}