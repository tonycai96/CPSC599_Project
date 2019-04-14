#pragma once
#include <vector>
#include "chai3d.h"
using namespace std;
using namespace chai3d;

class FillingSphere;
class LinearSpring;

class DeformableMesh {
public:
	DeformableMesh();
	void update(double dt);
	void applyForce(cVector3d force);

	cMesh* mesh;

private:
	static void updateMesh(cMesh* mesh, vector<LinearSpring*> springs);

	vector<LinearSpring*> m_springs;
	vector<FillingSphere*> m_spheres;
};