#include <vector>
#include "chai3d.h"
#include "DeformableMesh.h"
#include "LinearSpring.h"
#include "FillingSphere.h"

using namespace std;

DeformableMesh::DeformableMesh() {
	mesh = new cMesh();
	DeformableMesh::updateMesh(mesh, m_springs);
}

void DeformableMesh::update(double dt) {
	DeformableMesh::updateMesh(mesh, m_springs);
}

void DeformableMesh::applyForce(cVector3d force) {

}

void DeformableMesh::updateMesh(cMesh* mesh, vector<LinearSpring*> springs) {

}