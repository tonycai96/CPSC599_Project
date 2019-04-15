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
	ret->m_mesh = new cMesh();

	const int N = 16;
	vector<vector<FillingSphere*>> spheres(N + 1, vector<FillingSphere*>(N + 1));
	vector<vector<int>> v_ids(N + 1, vector<int>(N + 1));
	double x0 = -length / 2, y0 = -length / 2;
	for (int i = 0; i <= N; i++) {
		for (int j = 0; j <= N; j++) {
			double x = x0 + (1.0 * i / N) * length;
			double y = y0 = (1.0 * j / N) * length;
			spheres[i][j] = new FillingSphere(1.0, cVector3d(x, y, 0.01), cVector3d(0, 0, 1));
			int vertexId = ret->m_mesh->newVertex();
			ret->m_verticesId.push_back(vertexId);
			v_ids[i][j] = vertexId;
			ret->m_spheres.push_back(spheres[i][j]);
			ret->m_mesh->m_vertices->setColor(vertexId, cColorf(0, 0, 1));
			ret->m_mesh->m_vertices->setTexCoord(vertexId, cVector3d((1.0 * i) / N, (1.0 * j) / N, 0));
		}
	}
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			int v1 = v_ids[i][j], v2 = v_ids[i][j + 1];
			int v3 = v_ids[i + 1][j], v4 = v_ids[i + 1][j + 1];
			ret->m_mesh->newTriangle(v1, v3, v2);
			ret->m_mesh->newTriangle(v2, v3, v4);
		}
	}
	DeformableMesh::updateMesh(ret->m_mesh, ret->m_spheres, ret->m_verticesId);
	ret->m_mesh->setStiffness(1000.0, true);
	ret->m_mesh->createAABBCollisionDetector(0.002);

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

	ret->m_mesh->m_texture = cTexture2d::create();
	ret->m_mesh->m_texture->loadFromFile("plastic_texture.jpg");
	ret->m_mesh->setUseTexture(true);
	ret->m_mesh->setUseTransparency(true, true);
	ret->m_mesh->setTransparencyLevel(0.75, true, true, true);
	// Compute initial displacement position
	for (int i = 0; i < 200; i++) {
		ret->update(0.001);
	}

	ret->m_initial_mesh.resize(ret->m_mesh->getNumVertices());
	for (int i = 0; i < ret->m_mesh->getNumTriangles(); i++) {
		int vi0 = ret->m_mesh->m_triangles->getVertexIndex0(i);
		int vi1 = ret->m_mesh->m_triangles->getVertexIndex1(i);
		int vi2 = ret->m_mesh->m_triangles->getVertexIndex2(i);

		ret->m_initial_mesh[vi0] = ret->m_mesh->m_triangles->m_vertices->getLocalPos(vi0);
		ret->m_initial_mesh[vi1] = ret->m_mesh->m_triangles->m_vertices->getLocalPos(vi1);
		ret->m_initial_mesh[vi2] = ret->m_mesh->m_triangles->m_vertices->getLocalPos(vi2);
	}
	
	return ret;
}

DeformableMesh* DeformableMesh::createCircularCloth(double radius) {
	DeformableMesh* ret = new DeformableMesh();
	ret->m_mesh = new cMesh();
	const int N = 16;
	vector<vector<FillingSphere*>> spheres(N, vector<FillingSphere*>(N));
	vector<vector<int>> v_ids(N, vector<int>(N));
	spheres[0][0] = new FillingSphere(0, cVector3d(0, 0, radius / 1.5));
	for (int i = 1; i <= N; i++) {
		for (int j = 0; j < N; j++) {
			double r = (1.0 * i / N) * radius;
			double a = (1.0 * j / N) * 2 * M_PI;
			double x = r * cos(a), y = r * sin(a);
			double z = sqrt(radius * radius - r * r) / 1.5;
			spheres[i][j] = new FillingSphere(0, cVector3d(x, y, z));
		}
	}

	v_ids.resize(N + 1, vector<int>(N));
	v_ids[0][0] = ret->m_mesh->newVertex();
	for (int i = 1; i <= N; i++) {
		for (int j = 0; j < N; j++) {
			if (spheres[i][j]) {
				int vertexId = ret->m_mesh->newVertex();
				ret->m_mesh->m_vertices->setColor(vertexId, cColorf(0, 0, 1));
				v_ids[i][j] = vertexId;
			}
		}
	}

	for (int i = 0; i < N; i++) {
		int v1 = v_ids[0][0], v2 = v_ids[1][i], v3 = v_ids[1][(i + 1) % N];
		ret->m_mesh->newTriangle(v1, v2, v3);
	}
	for (int i = 1; i < N - 1; i++) {
		for (int j = 0; j < N; j++) {
			int v1 = v_ids[i][j], v2 = v_ids[i][(j + 1) % N];
			int v3 = v_ids[i + 1][j], v4 = v_ids[i + 1][(j + 1) % N];
			ret->m_mesh->newTriangle(v1, v3, v2);
			ret->m_mesh->newTriangle(v2, v3, v4);
		}
	}
	DeformableMesh::updateMesh(ret->m_mesh, ret->m_spheres, ret->m_verticesId);
}

void DeformableMesh::updateDevicePos(cVector3d pos) {
	cVector3d contactPoint(100, 100, 100);
	double best = 100;
	int vvi0 = -1, vvi1 = -1, vvi2 = -1;
	for (int i = 0; i < m_mesh->getNumTriangles(); i++) {
		int vi0 = m_mesh->m_triangles->getVertexIndex0(i);
		int vi1 = m_mesh->m_triangles->getVertexIndex1(i);
		int vi2 = m_mesh->m_triangles->getVertexIndex2(i);

		cVector3d vertex0 = m_mesh->m_triangles->m_vertices->getLocalPos(vi0);
		cVector3d vertex1 = m_mesh->m_triangles->m_vertices->getLocalPos(vi1);
		cVector3d vertex2 = m_mesh->m_triangles->m_vertices->getLocalPos(vi2);

		cVector3d pt = cProjectPointOnTriangle(pos, vertex0, vertex1, vertex2);
		double dist = (pos - pt).length();
		if (dist < CURSOR_RADIUS && dist < (pos - contactPoint).length()) {
			contactPoint = pt;
			vvi0 = vi0, vvi1 = vi1, vvi2 = vi2;
			m_triIndex = i;
		}
	}
	const double stiffness = 20000.0;
	double depth = (contactPoint - pos).length();
	cVector3d dir = contactPoint - pos;
	dir.normalize();

	if (vvi0 != -1) {
		m_spheres[vvi0]->user_force = 1.0 / 3 * stiffness * (CURSOR_RADIUS - depth) * dir;
		m_spheres[vvi1]->user_force = 1.0 / 3 * stiffness * (CURSOR_RADIUS - depth) * dir;
		m_spheres[vvi2]->user_force = 1.0 / 3 * stiffness * (CURSOR_RADIUS - depth) * dir;
	}
	m_devicePos = pos;
}

void DeformableMesh::update(double dt) {
	// Check if it's necessary to update the proxy position
	double volume = 0.00005 + computeMeshVolume();
	double pressure = 40 / volume;
	for (int i = 0; i < m_mesh->getNumTriangles(); i++) {
		int vi0 = m_mesh->m_triangles->getVertexIndex0(i);
		int vi1 = m_mesh->m_triangles->getVertexIndex1(i);
		int vi2 = m_mesh->m_triangles->getVertexIndex2(i);

		cVector3d vertex0 = m_mesh->m_triangles->m_vertices->getLocalPos(vi0);
		cVector3d vertex1 = m_mesh->m_triangles->m_vertices->getLocalPos(vi1);
		cVector3d vertex2 = m_mesh->m_triangles->m_vertices->getLocalPos(vi2);

		cVector3d sNormal = cComputeSurfaceNormal(vertex0, vertex1, vertex2);
		sNormal.normalize();
		double area = 0.5 * cCross(vertex1 - vertex0, vertex2 - vertex0).length();

		m_spheres[vi0]->env_force = (1.0 / 3) * pressure * area * sNormal;
		m_spheres[vi1]->env_force = (1.0 / 3) * pressure * area * sNormal;
		m_spheres[vi2]->env_force = (1.0 / 3) * pressure * area * sNormal;
	}
	for (auto sphere : m_spheres) {
		sphere->force = sphere->env_force + sphere->user_force;
		sphere->user_force = cVector3d(0, 0, 0);
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
	DeformableMesh::updateMesh(m_mesh, m_spheres, m_verticesId);
}

cVector3d DeformableMesh::computeForce() {
	if (m_triIndex == -1) {
		return cVector3d(0, 0, 0);
	}
	int vi0 = m_mesh->m_triangles->getVertexIndex0(m_triIndex);
	int vi1 = m_mesh->m_triangles->getVertexIndex1(m_triIndex);
	int vi2 = m_mesh->m_triangles->getVertexIndex2(m_triIndex);

	cVector3d cur_vert0 = m_mesh->m_triangles->m_vertices->getLocalPos(vi0);
	cVector3d cur_vert1 = m_mesh->m_triangles->m_vertices->getLocalPos(vi1);
	cVector3d cur_vert2 = m_mesh->m_triangles->m_vertices->getLocalPos(vi2);

	cVector3d old_vert0 = m_initial_mesh[vi0];
	cVector3d old_vert1 = m_initial_mesh[vi1];
	cVector3d old_vert2 = m_initial_mesh[vi2];

	const double stiffness = 500;
	cVector3d force1 = stiffness * (old_vert0 - cur_vert0);
	cVector3d force2 = stiffness * (old_vert1 - cur_vert1);
	cVector3d force3 = stiffness * (old_vert2 - cur_vert2);

	cVector3d force = 1.0 / 3 * force1 + 1.0 / 3 * force2 + 1.0 / 3 * force3;
	if (m_disable_time > 0) {
		return cVector3d(0, 0, 0);
	}
	return force;
}

void DeformableMesh::deactivate(double a_time) {
	m_disable_time = a_time;
}

void DeformableMesh::updateMesh(
	cMesh* mesh,
	vector<FillingSphere*> surfaceSpheres,
	vector<int> verticesId) {
	for (int i = 0; i < surfaceSpheres.size(); i++) {
		mesh->m_vertices->setLocalPos(verticesId[i], surfaceSpheres[i]->pos);
	}
	mesh->computeAllNormals();
}

double DeformableMesh::computeMeshVolume(cVector3d origin) {
	double ret = 0.0;
	for (int i = 0; i < m_mesh->getNumTriangles(); i++) {
		int vi0 = m_mesh->m_triangles->getVertexIndex0(i);
		int vi1 = m_mesh->m_triangles->getVertexIndex1(i);
		int vi2 = m_mesh->m_triangles->getVertexIndex2(i);
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