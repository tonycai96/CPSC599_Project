#include <vector>
#include <tuple>
#include "chai3d.h"
#include "DeformableMesh.h"
#include "LinearSpring.h"
#include "FillingSphere.h"

using namespace std;
using namespace chai3d;

DeformableMesh* DeformableMesh::createSquareCloth(double length) {
	DeformableMesh* ret = new DeformableMesh();
	ret->mesh = new cMesh();

	const int N = 16;
	vector<vector<FillingSphere*>> spheres(N + 1, vector<FillingSphere*>(N + 1));
	vector<vector<int>> v_ids(N + 1, vector<int>(N + 1));
	double x0 = -length / 2, y0 = -length / 2;
	for (int i = 0; i <= N; i++) {
		for (int j = 0; j <= N; j++) {
			double x = x0 + (1.0 * i / N) * length;
			double y = y0 = (1.0 * j / N) * length;
			spheres[i][j] = new FillingSphere(1.0, cVector3d(x, y, 0.01), cVector3d(0, 0, 1));
			int vertexId = ret->mesh->newVertex();
			ret->m_verticesId.push_back(vertexId);
			v_ids[i][j] = vertexId;
			ret->m_spheres.push_back(spheres[i][j]);
			ret->mesh->m_vertices->setColor(vertexId, cColorf(0, 0, 1));
		}
	}
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			int v1 = v_ids[i][j], v2 = v_ids[i][j + 1];
			int v3 = v_ids[i + 1][j], v4 = v_ids[i + 1][j + 1];
			ret->mesh->newTriangle(v1, v3, v2);
			ret->mesh->newTriangle(v2, v3, v4);
		}
	}
	DeformableMesh::updateMesh(ret->mesh, ret->m_spheres, ret->m_verticesId);
	ret->mesh->setStiffness(1000.0, true);
	ret->mesh->createAABBCollisionDetector(0.002);

	for (int i = 0; i < N; i++) {
		spheres[0][i]->isFixed = true;
		spheres[i][0]->isFixed = true;
		spheres[N][i]->isFixed = true;
		spheres[i][N]->isFixed = true;
	}
	for (int i = 0; i <= N; i++) {
		for (int j = 0; j <= N; j++) {
			// structural springs
			if (i + 1 <= N) {
				FillingSphere *s1 = spheres[i][j], *s2 = spheres[i + 1][j];
				ret->m_springs.push_back(
					new LinearSpring(s1, s2, (s1->pos - s2->pos).length()));
			}
			if (j + 1 <= N) {
				FillingSphere *s1 = spheres[i][j], *s2 = spheres[i][j + 1];
				ret->m_springs.push_back(
					new LinearSpring(s1, s2, (s1->pos - s2->pos).length()));
			}
			// shear springs
			if (i + 1 <= N && j + 1 <= N) {
				FillingSphere *s1 = spheres[i][j], *s2 = spheres[i + 1][j + 1];
				ret->m_springs.push_back(
					new LinearSpring(s1, s2, (s1->pos - s2->pos).length()));
			}
			if (i - 1 >= 0 && j + 1 <= N) {
				FillingSphere *s1 = spheres[i][j], *s2 = spheres[i - 1][j + 1];
				ret->m_springs.push_back(
					new LinearSpring(s1, s2, (s1->pos - s2->pos).length()));
			}
			// bending springs
			if (i + 2 <= N) {
				FillingSphere *s1 = spheres[i][j], *s2 = spheres[i + 2][j];
				ret->m_springs.push_back(
					new LinearSpring(s1, s2, (s1->pos - s2->pos).length()));
			}
			if (j + 2 <= N) {
				FillingSphere *s1 = spheres[i][j], *s2 = spheres[i][j + 2];
				ret->m_springs.push_back(
					new LinearSpring(s1, s2, (s1->pos - s2->pos).length()));
			}
		}
	}
	return ret;
}

//DeformableMesh::DeformableMesh(double radius) {
//	mesh = new cMesh();
//	const int NUM_DIV = 16;
//	m_spheres.resize(NUM_DIV + 1, vector<FillingSphere*>(NUM_DIV));
//	m_spheres[0][0] = new FillingSphere(0, cVector3d(0, 0, radius / 1.5));
//	for (int i = 1; i <= NUM_DIV; i++) {
//		for (int j = 0; j < NUM_DIV; j++) {
//			double r = (1.0 * i / NUM_DIV) * radius;
//			double a = (1.0 * j / NUM_DIV) * 2 * M_PI;
//			double x = r * cos(a), y = r * sin(a);
//			double z = sqrt(radius * radius - r * r) / 1.5;
//			m_spheres[i][j] = new FillingSphere(0, cVector3d(x, y, z));
//		}
//	}
//
//	m_verticesId.resize(NUM_DIV + 1, vector<int>(NUM_DIV));
//	m_verticesId[0][0] = mesh->newVertex();
//	for (int i = 1; i <= NUM_DIV; i++) {
//		for (int j = 0; j < NUM_DIV; j++) {
//			if (m_spheres[i][j]) {
//				int vertexId = mesh->newVertex();
//				mesh->m_vertices->setColor(vertexId, cColorf(0, 0, 1));
//				m_verticesId[i][j] = vertexId;
//			}
//		}
//	}
//
//	for (int i = 0; i < NUM_DIV; i++) {
//		int v1 = m_verticesId[0][0], v2 = m_verticesId[1][i], v3 = m_verticesId[1][(i + 1) % NUM_DIV];
//		mesh->newTriangle(v1, v2, v3);
//	}
//	for (int i = 1; i < m_spheres.size() - 1; i++) {
//		int n = m_spheres[i].size();
//		for (int j = 0; j < n; j++) {
//			int v1 = m_verticesId[i][j], v2 = m_verticesId[i][(j + 1) % n];
//			int v3 = m_verticesId[i + 1][j], v4 = m_verticesId[i + 1][(j + 1) % n];
//			mesh->newTriangle(v1, v3, v2);
//			mesh->newTriangle(v2, v3, v4);
//		}
//	}
//	DeformableMesh::updateMesh(mesh, m_spheres, m_verticesId);
//
//	mesh->createAABBCollisionDetector(0.0);
//	mesh->setStiffness(1000.0, true);
//}

bool DeformableMesh::isPointInside(cVector3d pos) {
	bool inside = false;
	for (int i = 0; i < mesh->getNumTriangles(); i++) {
		int vi0 = mesh->m_triangles->getVertexIndex0(i);
		int vi1 = mesh->m_triangles->getVertexIndex1(i);
		int vi2 = mesh->m_triangles->getVertexIndex2(i);

		cVector3d vertex0 = mesh->m_triangles->m_vertices->getLocalPos(vi0);
		cVector3d vertex1 = mesh->m_triangles->m_vertices->getLocalPos(vi1);
		cVector3d vertex2 = mesh->m_triangles->m_vertices->getLocalPos(vi2);

		double x = pos.x(), y = pos.y();
		cVector3d cPoint, cNormal;
		double u, v;
		bool intersect = cIntersectionSegmentTriangle(cVector3d(x, y, 0), cVector3d(x, y, 1),
			vertex0, vertex1, vertex2, true, true, cPoint, cNormal, u, v);
		if (intersect && pos.z() < cPoint.z()) {
			return true;
		}
	}
	return false;
}

void DeformableMesh::update(double dt) {
	// Check if it's necessary to update the proxy position
	cVector3d proxyPos = m_tool->m_hapticPoint->getGlobalPosProxy();
	cVector3d devicePos = m_tool->getDeviceGlobalPos();
	bool proxyInside = isPointInside(proxyPos);
	bool deviceInside = isPointInside(devicePos);
	if (proxyInside && deviceInside) {
		double best = 100;
		cVector3d closest;
		for (int i = 0; i < mesh->getNumTriangles(); i++) {
			int vi0 = mesh->m_triangles->getVertexIndex0(i);
			int vi1 = mesh->m_triangles->getVertexIndex1(i);
			int vi2 = mesh->m_triangles->getVertexIndex2(i);

			cVector3d vertex0 = mesh->m_triangles->m_vertices->getLocalPos(vi0);
			cVector3d vertex1 = mesh->m_triangles->m_vertices->getLocalPos(vi1);
			cVector3d vertex2 = mesh->m_triangles->m_vertices->getLocalPos(vi2);

			cVector3d pt = cProjectPointOnTriangle(proxyPos, vertex0, vertex1, vertex2);
			if ((pt - proxyPos).length() < best) {
				best = (pt - proxyPos).length();
				closest = pt;
			}
		}
	}
	updateUserForce();
	for (auto sphere : m_spheres) {
		sphere->force = sphere->env_force + sphere->user_force;
	}
	for (const auto spring : m_springs) {
		spring->computeForce();
	}
	for (int i = 0; i < m_spheres.size(); i++) {
		auto sphere = m_spheres[i];
		if (sphere->prev_dt > 0.0) {
			sphere->update(dt);
		} else {
			sphere->prev_dt = dt;
		}
	}
	DeformableMesh::updateMesh(mesh, m_spheres, m_verticesId);
}

cVector3d DeformableMesh::computeForce() {
	return cVector3d();
}

void DeformableMesh::updateUserForce() {
	/*if (force.length() < 1e-6 || !collision_evt) {
		if (m_prev_collision) {
			int vi0, vi1, vi2;
			tie(vi0, vi1, vi2) = *m_prev_collision;
			m_spheres[vi0]->user_force.zero();
			m_spheres[vi1]->user_force.zero();
			m_spheres[vi2]->user_force.zero();
		}
		m_prev_collision = NULL;
		return;
	}
	int vi0 = collision_evt->m_triangles->getVertexIndex0(collision_evt->m_index);
	int vi1 = collision_evt->m_triangles->getVertexIndex1(collision_evt->m_index);
	int vi2 = collision_evt->m_triangles->getVertexIndex2(collision_evt->m_index);

	cVector3d vertex0 = collision_evt->m_triangles->m_vertices->getLocalPos(vi0);
	cVector3d vertex1 = collision_evt->m_triangles->m_vertices->getLocalPos(vi1);
	cVector3d vertex2 = collision_evt->m_triangles->m_vertices->getLocalPos(vi2);

	int u = collision_evt->m_posV01;
	int v = collision_evt->m_posV02;
	int w = 1 - u - v;

	m_spheres[vi0]->user_force = w * force;
	m_spheres[vi1]->user_force += u * force;
	m_spheres[vi2]->user_force += v * force;

	m_prev_collision = &make_tuple(vi0, vi1, vi2);*/
}

void DeformableMesh::updateMesh(
	cMesh* mesh,
	vector<FillingSphere*> surfaceSpheres,
	vector<int> verticesId) {
	for (int i = 0; i < surfaceSpheres.size(); i++) {
		mesh->m_vertices->setLocalPos(verticesId[i], surfaceSpheres[i]->pos);
	}
	mesh->computeAllNormals();
	mesh->createAABBCollisionDetector(0.002);
}

double DeformableMesh::computeMeshVolume(cVector3d origin) {
	double ret = 0.0;
	for (int i = 0; i < mesh->getNumTriangles(); i++) {
		int vi0 = mesh->m_triangles->getVertexIndex0(i);
		int vi1 = mesh->m_triangles->getVertexIndex1(i);
		int vi2 = mesh->m_triangles->getVertexIndex2(i);
		cVector3d p1 = m_spheres[vi0]->pos - origin;
		cVector3d p2 = m_spheres[vi1]->pos - origin;
		cVector3d p3 = m_spheres[vi2]->pos - origin;
		double v321 = p3.x()*p2.y()*p1.z();
		double v231 = p2.x()*p3.y()*p1.z();
		double v312 = p3.x()*p1.y()*p2.z();
		double v132 = p1.x()*p3.y()*p2.z();
		double v213 = p2.x()*p1.y()*p3.z();
		double v123 = p1.x()*p2.y()*p3.z();
		ret += (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
	}
	return ret;
}