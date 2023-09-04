#include <algorithm>
#include "ContactConstraint.h"
//#include "PhysicsManager.h"
#include "BoundingUtils.h"
#include "Object.h"
#include "Transform.h"
#include "RigidBody.h"

void ContactConstraint::CalculateJacobian()
{
	glm::vec3& contactNormal = ConstraintData.collisionNormal_;

	glm::vec3 centerOfMassA = ColliderA->pOwner_->GetTransform()->GetPosition();
	glm::vec3 momentArmA = ConstraintData.worldContactA_ - centerOfMassA;

	glm::vec3 centerOfMassB = ColliderB->pOwner_->GetTransform()->GetPosition();
	glm::vec3 momentArmB = ConstraintData.worldContactB_ - centerOfMassB;

	glm::vec3 crossProductA = glm::cross(momentArmA, contactNormal);
	glm::vec3 crossProductB = glm::cross(momentArmB, contactNormal);

	ColliderA->ContactJacobian << -contactNormal.x, -contactNormal.y, -contactNormal.z,
		-crossProductA.x, -crossProductA.y, -crossProductA.z;

	ColliderB->ContactJacobian << contactNormal.x, contactNormal.y, contactNormal.z,
		crossProductB.x, crossProductB.y, crossProductB.z;

	if (ColliderA->eColliderType == AABB::STATIC)
	{
		ColliderA->ContactJacobian *= 0.0f;
	}
	else if (ColliderB->eColliderType == AABB::STATIC)
	{
		ColliderB->ContactJacobian *= 0.0f;
	}

	Jacobian << ColliderA->ContactJacobian, ColliderB->ContactJacobian;
	mInvJtranspose = inverseMassMatrix * Jacobian.transpose();
	effectMass = Jacobian * mInvJtranspose;
}

float ContactConstraint::Solve(float step, std::vector<Eigen::Matrix<float, 6, 1>>& A, Eigen::Matrix<float, 12, 1>& currVelocity, Eigen::Matrix<float, 12, 1>& Fext)
{
	float constraintError = 0.0f;
	CalculateInverseMassMatrices();

	// calculate constraint error from pos_ constraint equation Cn = (x2 +r2 −x1 −r1) * n1
	glm::vec3 centerOfMassA = ColliderA->pOwner_->GetTransform()->GetPosition();
	glm::vec3 momentArmA = ConstraintData.worldContactA_ - centerOfMassA;

	glm::vec3 centerOfMassB = ColliderB->pOwner_->GetTransform()->GetPosition();
	glm::vec3 momentArmB = ConstraintData.worldContactB_ - centerOfMassB;

	glm::vec3 pointA = centerOfMassA + momentArmA;
	glm::vec3 pointB = centerOfMassB + momentArmB;
	constraintError = glm::dot((pointB - pointA), ConstraintData.collisionNormal_);


	//float baumgarteScalar = 0.0035f;
	float baumgarteScalar = 0.1;
	float penetrationSlop = 0.0005f;
	float restitutionSlop = 0.1f;

	// lets objects to penetrate a bit before actually applying Baumgarte stabilization
	constraintError = std::max(constraintError - penetrationSlop, 0.0f);

	glm::vec3 linearVelocityA = ColliderA->pOwner_->GetRigidBody()->currLinearVelocity_;
	glm::vec3 linearVelocityB = ColliderB->pOwner_->GetRigidBody()->currLinearVelocity_;
	glm::vec3 angularVelocityA = ColliderA->pOwner_->GetRigidBody()->currAngularVelocity_;
	glm::vec3 angularVelocityB = ColliderB->pOwner_->GetRigidBody()->currAngularVelocity_;

	glm::vec3 relativeVelocityA = linearVelocityA + glm::cross(angularVelocityA, momentArmA);
	glm::vec3 relativeVelocityB = linearVelocityB + glm::cross(angularVelocityB, momentArmB);
	glm::vec3 relativeVelocity = relativeVelocityB - relativeVelocityA;

	float projection = glm::dot(relativeVelocity, ConstraintData.collisionNormal_);

	if (projection > 0.0f)
	{
		deltaLambda = 0.0f;
		return deltaLambda;
	}

	float restitutionA = ColliderA->restitution_;
	float restitutionB = ColliderB->restitution_;
	float restitution = (restitutionA + restitutionB) / 2.0f;

	float restitutionTerm = (projection * restitution);
	float baumgarteTerm = (-baumgarteScalar * constraintError) / step;
	float biasTerm = (baumgarteTerm + restitutionTerm) / step;

	// Equation 35 in 'Iterative Dynamics
	float currentVelocityTerm = (Jacobian * (currVelocity / step));
	float externalForceTerm = (Jacobian * (inverseMassMatrix * Fext));

	eta = biasTerm + currentVelocityTerm + externalForceTerm;

	// delta lambda i
	float bodyATerm = ColliderA->ContactJacobian * A[ColliderA->colliderID_];
	float bodyBTerm = ColliderB->ContactJacobian * A[ColliderB->colliderID_];

	deltaLambda = -(eta - bodyATerm - bodyBTerm);
	deltaLambda /= effectMass;

	float normalImpulseSumCopy = NormalImpulseSum;

	NormalImpulseSum += deltaLambda;

	// clamps nmormal impulse sum
	NormalImpulseSum = std::max(0.0f, std::min(NormalImpulseSum, FLT_MAX));

	deltaLambda = NormalImpulseSum - normalImpulseSumCopy;
	Eigen::Matrix<float, 6, 1> bodyA, bodyB;

	bodyA << mInvJtranspose(0), mInvJtranspose(1), mInvJtranspose(2),
		mInvJtranspose(3), mInvJtranspose(4), mInvJtranspose(5);

	bodyB << mInvJtranspose(6), mInvJtranspose(7), mInvJtranspose(8),
		mInvJtranspose(9), mInvJtranspose(10), mInvJtranspose(11);
	
	// update A
	A[ColliderA->colliderID_] += deltaLambda * bodyA;
	A[ColliderB->colliderID_] += deltaLambda * bodyB;

	return deltaLambda;
}
