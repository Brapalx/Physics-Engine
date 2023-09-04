#pragma once
#include <glm/glm.hpp>
#include "Rigidbody.h"

class ForceGenerator
{
	virtual void updateForce(RigidBody* body, float duration) {};
};

class Gravity : public ForceGenerator
{
	glm::vec3 gravity;
public:
	Gravity(const glm::vec3& grav);

	void updateForce(RigidBody* body, float duration);
};