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
	static DeformableMesh* createCircularCloth(double radius);
	// static DeformableMesh* createCircularCloth(double radius);
	void update(double dt);
	cVector3d computeForce();
	void updateDevicePos(cVector3d pos);
	void deactivate(double a_time);

	cMesh* m_mesh;
	double m_maxForce;
	double m_current_time = -1;
	double m_disable_time = -1;

private:
	static void DeformableMesh::updateMesh(
		cMesh* mesh,
		vector<FillingSphere*> surfaceSpheres,
		vector<int> verticesId);

	double computeMeshVolume(cVector3d origin = cVector3d(0, 0, 0));

	DeformableMesh() : m_triIndex(-1), m_maxForce(4) {}
	vector<LinearSpring*> m_springs;
	vector<FillingSphere*> m_spheres;
	vector<int> m_verticesId;
	vector<tuple<int, int, int>> m_triangles;
	int m_triIndex;
	cVector3d m_devicePos;
	vector<cVector3d> m_initial_mesh;

	double CURSOR_RADIUS = 0.02;
};