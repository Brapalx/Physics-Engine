/* Start Header -----------------------------------------------------------------
File: Rigidbody.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/

#pragma once
#include "Component.h"

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

#include "glm\vec3.hpp"
#include "glm\vec4.hpp"
#include "glm\vec2.hpp"
#include <glm\gtc\quaternion.hpp>
#include <glm\mat3x3.hpp>
#include <glm\mat4x4.hpp>

class RigidBody : public Component
{
public:

	glm::vec3 nextPosition_ = glm::vec3();
	glm::vec3 currPosition_ = glm::vec3();
	glm::vec3 prevPosition_ = glm::vec3();

	glm::vec3 currLinearVelocity_ = glm::vec3();
	glm::vec3 prevLinearVelocity_ = glm::vec3();

	glm::vec3 currAngularVelocity_ = glm::vec3();
	glm::vec3 prevAngularVelocity_ = glm::vec3();

	glm::vec3 Force = glm::vec3();
	glm::vec3 Torque = glm::vec3();

	float mass_ = 2.0f;
	float inverseMass_ = 1.0f / mass_;

	bool gravityEnabled_ = true;
	float gravityMagnitude_ = -0.2f;

	RigidBody() : Component(Component::PHYSICS)
	{}

	static inline Component::ComponentType GetComponentID() { return (ComponentType::PHYSICS); }
	static inline const char* GetComponentName() { return ComponentTypeName[ComponentType::PHYSICS]; }

	inline float GetMass() { return mass_; }
	inline glm::vec3 GetCurrentPosition() { return currPosition_; }
	inline glm::vec3 GetVelocity() { return currLinearVelocity_; }
	inline void SetMass(float mass) { mass_ = mass; inverseMass_ = 1 / mass_; }
	inline void SetCurrentPosition(glm::vec3 position) { currPosition_ = position; }
	inline void SetNextPosition(glm::vec3 position) { nextPosition_ = position; }
	inline void ApplyForce(glm::vec3 newForce) { Force += newForce; }
	virtual void Initialize() override;

	void SyncPhysicsWithTransform();
	void UpdateTransform();
	void IntegrateEuler(float dt);
};