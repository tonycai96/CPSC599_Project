#pragma once
#include <vector>
#include "chai3d.h"
using namespace std;
using namespace chai3d;

class FillingSphere;
class LinearSpring;

class DeformableMesh {
public:
	DeformableMesh(double radius);
	void update(double dt);
	void applyForce(cVector3d force);

	cMesh* mesh;

private:
	static void DeformableMesh::updateMesh(
		cMesh* mesh,
		vector<vector<FillingSphere*>> surfaceSpheres,
		vector<vector<int>> verticesId);

	vector<LinearSpring*> m_springs;
	vector<FillingSphere*> m_spheres;
	vector<vector<FillingSphere*>> m_surfaceSpheres;
	vector<vector<int>> m_verticesId;
};