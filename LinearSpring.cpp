#include "chai3d.h"
#include "LinearSpring.h"
#include "FillingSphere.h"

using namespace chai3d;

void LinearSpring::computeForce() {
	cVector3d spring = sphere1->pos - sphere2->pos;
	cVector3d dir = spring;
	dir.normalize();
	cVector3d force2 = stiffness * (spring.length() - natural_length) * dir;
	sphere2->force += force2;
	cVector3d force1 = stiffness * (spring.length() - natural_length) * -dir;
	sphere1->force += force1;
}