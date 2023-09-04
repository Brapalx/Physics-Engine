#include "ForceGenerator.h"

Gravity::Gravity(const glm::vec3& grav)
{
	gravity = grav;
}


void Gravity::updateForce(RigidBody* body, float duration)
{
	// Check that we do not have infinite mass
	//if (!body->hasFiniteMass()) return;
	// Apply the mass-scaled force to the body
	//body->addForce(gravity * body->getMass());
}