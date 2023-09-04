#include "Object.h"
#include "Transform.h"
#include "RigidBody.h"
#include "BoundingUtils.h"
#include "Constraint.h"

float Constraint::InitialLambda = 0.0f;

void Constraint::CalculateInverseMassMatrices()
{
	RigidBody& physicsA = *(ColliderA->pOwner_->GetRigidBody());
	RigidBody& physicsB = *(ColliderB->pOwner_->GetRigidBody());

	Eigen::Matrix3f massMatrixA;
	massMatrixA.setIdentity();
	massMatrixA *= physicsA.mass_;

	Eigen::Matrix3f massMatrixB;
	massMatrixB.setIdentity();
	massMatrixB *= physicsB.mass_;

	Eigen::Matrix3f inertiaTensorA;
	Eigen::Matrix3f inertiaTensorB;
	Eigen::Matrix3f rotationMatrixA_Eigen, rotationMatrixB_Eigen;
	glm::mat3 rotationMatrixA_GLM = glm::mat3_cast(physicsA.pOwner_->GetTransform()->Rotation);
	glm::mat3 rotationMatrixB_GLM = glm::mat3_cast(physicsB.pOwner_->GetTransform()->Rotation);

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			inertiaTensorA(i, j) = ColliderA->InertiaTensor[i][j] * physicsA.mass_;
			inertiaTensorB(i, j) = ColliderB->InertiaTensor[i][j] * physicsB.mass_;
			rotationMatrixA_Eigen(i, j) = rotationMatrixA_GLM[i][j];
			rotationMatrixB_Eigen(i, j) = rotationMatrixB_GLM[i][j];
		}
	}

	Eigen::Matrix3f inverseInertiaTensorA = inertiaTensorA.inverse();
	Eigen::Matrix3f inverseInertiaTensorB = inertiaTensorB.inverse();
	Eigen::Matrix3f inverseMassMatrixA = massMatrixA.inverse();
	Eigen::Matrix3f inverseMassMatrixB = massMatrixB.inverse();

	inverseInertiaTensorA = rotationMatrixA_Eigen * inverseInertiaTensorA;
	inverseInertiaTensorB = rotationMatrixB_Eigen * inverseInertiaTensorB;

	inverseInertiaTensorA = inverseInertiaTensorA * rotationMatrixA_Eigen.transpose();
	inverseInertiaTensorB = inverseInertiaTensorB * rotationMatrixB_Eigen.transpose();

	inverseMassMatrix << inverseMassMatrixA, Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(),
		Eigen::Matrix3f::Zero(), inverseInertiaTensorA, Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(),
		Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), inverseMassMatrixB, Eigen::Matrix3f::Zero(),
		Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), inverseInertiaTensorB;

	ColliderA->InverseMassMatrix << inverseMassMatrixA, Eigen::Matrix3f::Zero(),
		Eigen::Matrix3f::Zero(), inverseInertiaTensorA;

	ColliderB->InverseMassMatrix << inverseMassMatrixB, Eigen::Matrix3f::Zero(),
		Eigen::Matrix3f::Zero(), inverseInertiaTensorB;
}
