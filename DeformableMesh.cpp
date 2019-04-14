#include <vector>
#include "chai3d.h"
#include "DeformableMesh.h"
#include "LinearSpring.h"
#include "FillingSphere.h"

using namespace std;
using namespace chai3d;

DeformableMesh::DeformableMesh(double radius) {
	mesh = new cMesh();
	const int NUM_DIV = 16;
	m_surfaceSpheres.resize(NUM_DIV + 1, vector<FillingSphere*>(NUM_DIV));
	m_surfaceSpheres[0][0] = new FillingSphere(0, cVector3d(0, 0, radius / 1.5));
	for (int i = 1; i <= NUM_DIV; i++) {
		for (int j = 0; j < NUM_DIV; j++) {
			double r = (1.0 * i / NUM_DIV) * radius;
			double a = (1.0 * j / NUM_DIV) * 2 * M_PI;
			double x = r * cos(a), y = r * sin(a);
			double z = sqrt(radius * radius - r * r) / 1.5;
			m_surfaceSpheres[i][j] = new FillingSphere(0, cVector3d(x, y, z));
		}
	}

	m_verticesId.resize(NUM_DIV + 1, vector<int>(NUM_DIV));
	m_verticesId[0][0] = mesh->newVertex();
	for (int i = 1; i <= NUM_DIV; i++) {
		for (int j = 0; j < NUM_DIV; j++) {
			if (m_surfaceSpheres[i][j]) {
				int vertexId = mesh->newVertex();
				mesh->m_vertices->setColor(vertexId, cColorf(0, 0, 1));
				m_verticesId[i][j] = vertexId;
			}
		}
	}

	for (int i = 0; i < NUM_DIV; i++) {
		int v1 = m_verticesId[0][0], v2 = m_verticesId[1][i], v3 = m_verticesId[1][(i + 1) % NUM_DIV];
		mesh->newTriangle(v1, v2, v3);
	}
	for (int i = 1; i < m_surfaceSpheres.size() - 1; i++) {
		int n = m_surfaceSpheres[i].size();
		for (int j = 0; j < n; j++) {
			int v1 = m_verticesId[i][j], v2 = m_verticesId[i][(j + 1) % n];
			int v3 = m_verticesId[i + 1][j], v4 = m_verticesId[i + 1][(j + 1) % n];
			mesh->newTriangle(v1, v3, v2);
			mesh->newTriangle(v2, v3, v4);
		}
	}
	DeformableMesh::updateMesh(mesh, m_surfaceSpheres, m_verticesId);

	mesh->createAABBCollisionDetector(0.0);
	mesh->setStiffness(1000.0, true);
}

void DeformableMesh::update(double dt) {
	// TODO: update location of surface spheres
	DeformableMesh::updateMesh(mesh, m_surfaceSpheres, m_verticesId);
}

void DeformableMesh::applyForce(cVector3d force) {

}

// Assumes surfaceSpheres[0][0] -> 0, surfaceSpheres[1][0] -> 1, surfaceSpheres[1][1] -> 2, etc
void DeformableMesh::updateMesh(
	cMesh* mesh,
	vector<vector<FillingSphere*>> surfaceSpheres,
	vector<vector<int>> verticesId) {
	for (int i = 0; i < surfaceSpheres.size(); i++) {
		for (int j = 0; j < surfaceSpheres[i].size(); j++) {
			if (surfaceSpheres[i][j])
				mesh->m_vertices->setLocalPos(verticesId[i][j], surfaceSpheres[i][j]->pos);
		}
	}
	mesh->computeAllNormals();
}