/* Start Header -----------------------------------------------------------------
File: PhysicsManager.cpp
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#include "PhysicsManager.h"
#include "Object.h"
#include "Transform.h"
#include "RigidBody.h"
#include "BoundingUtils.h"
#include "ContactConstraint.h"
#include "FrameRateController.h"

int PhysicsManager::IntegratorIterations = 1;
int ConstraintIterations = 20;

void PhysicsManager::Update()
{

	Simulation();

	DetectCollisions();

	for (int i = 0; i < ConstraintIterations; i++)
		SolveConstraints();

}

void PhysicsManager::DetectCollisions()
{
	// Do collision detection for each pair of colliders
	for (int i = 0; i < ColliderObjectsList.size(); ++i)
	{
		for (int j = 0; j < ColliderObjectsList.size(); ++j)
		{
			if (i == j)
				break;

			AABB* collider1 = ColliderObjectsList[i];
			AABB* collider2 = ColliderObjectsList[j];

			Transform* transform1 = collider1->pOwner_->GetTransform();
			Transform* transform2 = collider2->pOwner_->GetTransform();

			ContactData newContactData;

			glm::mat4 model1, model2;
			glm::mat4 translate = glm::translate(transform1->GetPosition());
			glm::mat4 rotate = glm::mat4_cast(transform1->GetRotation());
			glm::mat4 scale = glm::scale(transform1->GetScale());
			model1 = translate * rotate * scale;

			translate = glm::translate(transform2->GetPosition());
			rotate = glm::mat4_cast(transform2->GetRotation());
			scale = glm::scale(transform2->GetScale());
			model2 = translate * rotate * scale;

			collider1->localToWorldMatrix_ = model1;
			collider2->localToWorldMatrix_ = model2;

			if (GJKCollisionHandler(collider1, collider2, newContactData))
			{
				// check for constraint existing
				bool hasConstraint = false;
				ContactConstraint* contactConstraint = nullptr;

				for (Constraint* constraint : ConstraintObjectsList)
				{
					contactConstraint = dynamic_cast<ContactConstraint*>(constraint);
					if (contactConstraint)
					{
						if (contactConstraint->ColliderA == collider1 && contactConstraint->ColliderB == collider2)
						{
							hasConstraint = true;
							break;
						}
					}
				}

				if (hasConstraint == false)
				{
					// create new constraint
					ContactConstraint* newConstraint = new ContactConstraint(*collider1, *collider2);
					newConstraint->ConstraintData = newContactData;
					newConstraint->CalculateJacobian();

					// register it to be resolved later
					RegisterConstraintObject(newConstraint);

						// create mnanifold for new point
						ContactManifold * newManifold = new ContactManifold();
						newManifold->constraintID_ = newConstraint->constraintID;

						RegisterManifoldObject(newManifold);
						newManifold->Push(newContactData);

						collider1->isColliding_ = true;
						collider2->isColliding_ = true;
						
				}
				else
				{
					// add new point to manifold, if constraint exists
					ContactManifold* manifold = ManifoldObjectsList[contactConstraint->manifoldID];
					manifold->Push(newContactData);

					ContactConstraint * newConstraint = new ContactConstraint(*collider1, *collider2);
					newConstraint->ConstraintData = newContactData;
					newConstraint->CalculateJacobian();

					RegisterConstraintObject(newConstraint);
				}
			}
		}
	}
}


void PhysicsManager::SolveConstraints()
{
	if (ConstraintObjectsList.size() == 0)
		return;

	std::vector<Eigen::Matrix<float, 6, 1>> A;
	A.reserve(ColliderObjectsList.size());

	for (int i = 0; i < ColliderObjectsList.size(); ++i)
	{
		A.push_back(Eigen::Matrix<float, 6, 1>());
		A.back().setZero();
	}
	// calculate preliminary values of Catto_A - it is used to store already calculated impulses
	for (int i = 0; i < ConstraintObjectsList.size(); ++i)
	{
		Constraint* constraint = nullptr;
		constraint = ConstraintObjectsList[i];

		if (constraint)
		{
			// initial lambda
			constraint->ImpulseMagnitude = constraint->InitialLambda;

			if (constraint->ColliderA->eColliderType > 0)
			{
				A[constraint->ColliderA->colliderID_] += constraint->ColliderA->InverseMassMatrix * constraint->ColliderA->ContactJacobian.transpose() * constraint->ImpulseMagnitude;
			}
			else
			{
				A[constraint->ColliderB->colliderID_] += constraint->ColliderB->InverseMassMatrix * constraint->ColliderB->ContactJacobian.transpose() * constraint->ImpulseMagnitude;
			}
		}
	}
	// Gauss-Siedel solver
	for (int iterations = 0; iterations < ConstraintSolverIterations; ++iterations)
	{
		if (ConstraintObjectsList.size() == 0)
			return;

		int size = (int)ConstraintObjectsList.size();
		for (int i = 0; i < ConstraintObjectsList.size(); ++i)
		{
			Constraint* constraint = nullptr;
			constraint = ConstraintObjectsList[i];
			float deltaLambda = 0.0f;
			float deltaTime = g_FrameRateController->FixedDelta;

			if (constraint)
			{
				RigidBody& physicsA = *constraint->ColliderA->pOwner_->GetRigidBody();
				RigidBody& physicsB = *constraint->ColliderB->pOwner_->GetRigidBody();

				Eigen::Matrix<float, 12, 1> currentVelocityVector; 
				currentVelocityVector << physicsA.currLinearVelocity_.x, physicsA.currLinearVelocity_.y, physicsA.currLinearVelocity_.z,
					physicsA.currAngularVelocity_.x, physicsA.currAngularVelocity_.y, physicsA.currAngularVelocity_.z,
					physicsB.currLinearVelocity_.x, physicsB.currLinearVelocity_.y, physicsB.currLinearVelocity_.z,
					physicsB.currAngularVelocity_.x, physicsB.currAngularVelocity_.y, physicsB.currAngularVelocity_.z;

				Eigen::Matrix<float, 12, 1> externalForceVector;
				externalForceVector << physicsA.Force.x, physicsA.Force.y, physicsA.Force.z,
					physicsA.Torque.x, physicsA.Torque.y, physicsA.Torque.z,
					physicsB.Force.x, physicsB.Force.y, physicsB.Force.z,
					physicsB.Torque.x, physicsB.Torque.y, physicsB.Torque.z;

				deltaLambda = constraint->Solve(deltaTime, A, currentVelocityVector, externalForceVector);

				// remove constraint if resolved
				if (abs(deltaLambda) < 0.0000000001f)
				{
					std::cout << "Removed constraint\n";
					std::swap(ConstraintObjectsList[i], ConstraintObjectsList.back());
					ConstraintObjectsList.pop_back();

					physicsA.pOwner_->GetAABB()->isColliding_ = false;
					physicsB.pOwner_->GetAABB()->isColliding_ = false;

					break;
				}

				// calculate constraint forces
				Eigen::Matrix<float, 6, 1> constraintForceA = constraint->ColliderA->ContactJacobian.transpose() * deltaLambda * deltaTime;
				Eigen::Matrix<float, 6, 1> constraintForceB = constraint->ColliderB->ContactJacobian.transpose() * deltaLambda * deltaTime;

				glm::vec3 forceA(constraintForceA(0), constraintForceA(1), constraintForceA(2)), torqueA(constraintForceA(3), constraintForceA(4), constraintForceA(5));
				glm::vec3 forceB(constraintForceB(0), constraintForceB(1), constraintForceB(2)), torqueB(constraintForceB(3), constraintForceB(4), constraintForceB(5));

				// static object check
				if (constraint->ColliderA->eColliderType == AABB::STATIC)
				{
					forceA *= 0;
					torqueA *= 0;
				}
				else if (constraint->ColliderB->eColliderType == AABB::STATIC)
				{
					forceB *= 0;
					torqueB *= 0;
				}

				// store previous velocities
				physicsA.prevLinearVelocity_ = physicsA.currLinearVelocity_;
				physicsB.prevLinearVelocity_ = physicsB.currLinearVelocity_;

				physicsA.prevAngularVelocity_ = physicsA.currAngularVelocity_;
				physicsB.prevAngularVelocity_ = physicsB.currAngularVelocity_;

				// calkculate inverse mass matrices
				glm::mat3 inverseMassMatrixA = glm::mat3(1), inverseMassMatrixB = glm::mat3(1);
				inverseMassMatrixA *= physicsA.inverseMass_;
				inverseMassMatrixB *= physicsB.inverseMass_;

				// calculate inverse inertia tensors
				glm::mat3 inverseInertiaTensorA, inverseInertiaTensorB;
				inverseInertiaTensorA = glm::inverse(constraint->ColliderA->InertiaTensor);
				inverseInertiaTensorB = glm::inverse(constraint->ColliderB->InertiaTensor);

				glm::mat3 rotationMatrixA = glm::mat3_cast(physicsA.pOwner_->GetTransform()->Rotation);
				glm::mat3 rotationMatrixB = glm::mat3_cast(physicsB.pOwner_->GetTransform()->Rotation);
				inverseInertiaTensorA = rotationMatrixA * inverseInertiaTensorA * glm::transpose(rotationMatrixA);
				inverseInertiaTensorB = rotationMatrixB * inverseInertiaTensorB * glm::transpose(rotationMatrixB);

				physicsA.currLinearVelocity_ += inverseMassMatrixA * forceA;
				physicsA.currAngularVelocity_ += inverseInertiaTensorA * torqueA;

				physicsB.currLinearVelocity_ += inverseMassMatrixB * forceB;
				physicsB.currAngularVelocity_ += inverseInertiaTensorB * torqueB;


				physicsA.Force = glm::vec3(0);
				physicsB.Force = glm::vec3(0);
			}
		}
	}
}


void PhysicsManager::RegisterPhysicsObject(RigidBody* aNewPhysics)
{
	PhysicsObjectsList.push_back(aNewPhysics);
}

void PhysicsManager::RegisterColliderObject(AABB* aNewCollider)
{
	aNewCollider->colliderID_ = (int)ColliderObjectsList.size();
	ColliderObjectsList.push_back(aNewCollider);
}

void PhysicsManager::RegisterConstraintObject(Constraint* aNewConstraint)
{
	aNewConstraint->constraintID = (int)ConstraintObjectsList.size();
	ConstraintObjectsList.push_back(aNewConstraint);
}

void PhysicsManager::RegisterManifoldObject(ContactManifold* aNewManifold)
{
	aNewManifold->manifoldID_ = (int)ManifoldObjectsList.size();
	ManifoldObjectsList.push_back(aNewManifold);
}

void PhysicsManager::Simulation()
{
	RigidBody* body = nullptr;
	float deltatime = g_FrameRateController->FixedDelta;

	for (int i = 0; i < IntegratorIterations; ++i)
	{
		for (auto iterator = PhysicsObjectsList.begin(); iterator != PhysicsObjectsList.end(); ++iterator)
		{
			body = static_cast<RigidBody*>(*iterator);
			body->SyncPhysicsWithTransform();
			body->IntegrateEuler(deltatime);

		}
	}
}
