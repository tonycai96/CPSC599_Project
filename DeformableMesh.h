#pragma once
#include <vector>
#include "chai3d.h"
using namespace std;
using namespace chai3d;

class FillingSphere;
class LinearSpring;

class DeformableMesh {
public:
	static DeformableMesh* createSquareCloth(double length);
	// static DeformableMesh* createCircularCloth(double radius);
	void update(double dt);
	void applyForce(cCollisionEvent* evt, cVector3d force);

	cMesh* mesh;

private:
	static void DeformableMesh::updateMesh(
		cMesh* mesh,
		vector<FillingSphere*> surfaceSpheres,
		vector<int> verticesId);

	double computeMeshVolume();

	DeformableMesh() {}
	vector<LinearSpring*> m_springs;
	vector<FillingSphere*> m_spheres;
	vector<int> m_verticesId;
	vector<tuple<int, int, int>> m_triangles;
	tuple<int, int, int>* m_prev_collision;
};