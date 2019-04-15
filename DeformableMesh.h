#pragma once
#include <vector>
#include "chai3d.h"
using namespace std;
using namespace chai3d;

class FillingSphere;
class LinearSpring;
class MyProxyAlgorithm;

class DeformableMesh {
public:
	static DeformableMesh* createSquareCloth(double length);
	// static DeformableMesh* createCircularCloth(double radius);
	void update(double dt);
	void updateUserForce();
	cVector3d computeForce();

	cMesh* mesh;
	cToolCursor* m_tool;
	bool DeformableMesh::isPointInside(cVector3d pos);

private:
	static void DeformableMesh::updateMesh(
		cMesh* mesh,
		vector<FillingSphere*> surfaceSpheres,
		vector<int> verticesId);

	double computeMeshVolume(cVector3d origin = cVector3d(0, 0, 0));

	DeformableMesh() : m_triIndex(-1) {}
	vector<LinearSpring*> m_springs;
	vector<FillingSphere*> m_spheres;
	vector<int> m_verticesId;
	vector<tuple<int, int, int>> m_triangles;
	int m_triIndex;
};