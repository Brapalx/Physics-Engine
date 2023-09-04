#include "RigidBody.h"
#include "Transform.h"
#include "Object.h"
#include "BoundingUtils.h"

void RigidBody::Initialize()
{
}

void RigidBody::SyncPhysicsWithTransform()
{
	Transform* transform = this->GetOwner()->GetTransform();
	currPosition_ = transform->Position;
}

void RigidBody::UpdateTransform()
{
	Transform* transform = this->GetOwner()->GetTransform();
	transform->Position = currPosition_;
}

void RigidBody::IntegrateEuler(float dt)
{
	AABB* collider = pOwner_->GetAABB();
	if (collider)
	{
		if (collider->eColliderType == AABB::STATIC)
			return;
	}

	// gravity
	/*if (gravityEnabled_)
		currLinearVelocity_ += glm::vec3(0, gravityMagnitude_, 0) * dt;*/

	if (gravityEnabled_)
		ApplyForce(glm::vec3(0, gravityMagnitude_, 0) * dt);


	prevLinearVelocity_ = currLinearVelocity_;
	// integrate force and linear velocity
	currLinearVelocity_ = currLinearVelocity_ + ((Force * inverseMass_) * dt);
	prevPosition_ = currPosition_;
	currPosition_ = prevPosition_ + currLinearVelocity_ * dt;

	prevAngularVelocity_ = currAngularVelocity_;
	// integrate torque and angular velocity
	glm::vec3 axis = currAngularVelocity_;
	float length = glm::length(currAngularVelocity_);
	float angle = length * dt; 
	// Prevents degenerate quaternions
	if (angle != 0.0f)
	{
		// Normalize axis
		axis = axis / length;
		glm::quat rotationDelta(std::cos(angle / 2.0f), axis * std::sin(angle / 2.0f));

		// Update rotation directly
		Transform& transform = *(pOwner_->GetTransform());
		transform.Rotation = rotationDelta * transform.Rotation;
	}
	UpdateTransform();

	std::cout << Force.y << std::endl;
}
