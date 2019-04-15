#include "chai3d.h"
#include "FillingSphere.h"

using namespace chai3d;

void FillingSphere::update(double dt) {
	if (isFixed)
	{
		return;
	}
	cVector3d a = force / mass;
	cVector3d tmp_pos = pos;
	cVector3d delta_pos = (pos - prev_pos) * dt / prev_dt + a * (dt + prev_dt) / 2 * dt;
	pos = pos + 0.9 * delta_pos;
	prev_pos = tmp_pos;
	prev_dt = dt;
}