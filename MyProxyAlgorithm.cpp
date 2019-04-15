//==============================================================================
/*
    CPSC 599.86 / 601.86 - Computer Haptics
    Winter 2018, University of Calgary

    This class extends the cAlgorithmFingerProxy class in CHAI3D that
    implements the god-object/finger-proxy haptic rendering algorithm.
    It allows us to modify or recompute the force that is ultimately sent
    to the haptic device.

    Your job for this assignment is to implement the updateForce() method
    in this class to support for two new effects: force shading and haptic
    textures. Methods for both are described in Ho et al. 1999.
*/
//==============================================================================

#include <vector>
#include <iostream>
#include "MyProxyAlgorithm.h"
#include "DeformableMesh.h"

using namespace chai3d;

MyProxyAlgorithm::MyProxyAlgorithm(std::vector<DeformableMesh*> bubbles) 
	: m_bubbles(bubbles) {}

//==============================================================================
/*!
    This method uses the information computed earlier in
    computeNextBestProxyPosition() to calculate the force to be rendered.
    It first calls cAlgorithmFingerProxy::updateForce() to compute the base
    force from contact geometry and the constrained proxy position. That
    force can then be modified or recomputed in this function.

    Your implementation of haptic texture mapping will likely end up in this
    function. When this function is called, collision detection has already
    been performed, and the proxy point has already been updated based on the
    constraints found. Your job is to compute a force with all that information
    available to you.

    Useful variables to read:
        m_deviceGlobalPos   - current position of haptic device
        m_proxyGlobalPos    - computed position of the constrained proxy
        m_numCollisionEvents- the number of surfaces constraining the proxy
        m_collisionRecorderConstraint0,1,2
                            - up to three cCollisionRecorder structures with
                              cCollisionEvents that contain very useful
                              information about each contact

    Variables that this function should set/reset:
        m_normalForce       - computed force applied in the normal direction
        m_tangentialForce   - computed force along the tangent of the surface
        m_lastGlobalForce   - this is what the operator ultimately feels!!!

        f = kx
        x = diff between device and proxy pos
        read k from map

*/
//==============================================================================

void MyProxyAlgorithm::updateForce()
{
    cAlgorithmFingerProxy::updateForce();

	if (m_numCollisionEvents > 0)
	{
		// std::cout << "FOO\n";
		cCollisionEvent* c0 = &m_collisionRecorderConstraint0.m_nearestCollision;
		for (auto bubble : m_bubbles) {
			if (c0->m_object == bubble->mesh) {
				bubble->applyForce(c0, m_lastGlobalForce);
			} else {
				bubble->applyForce(NULL, cVector3d(0, 0, 0));
			}
		}
	}
}
