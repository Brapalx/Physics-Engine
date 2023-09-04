/* Start Header -----------------------------------------------------------------
File: Constraint.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once

#include "Libraries/Eigen/Dense"
#include <vector>

class AABB;
class Constraint
{
public:
	AABB* ColliderA;
	AABB* ColliderB;

	Eigen::Matrix<float, 1, 12> Jacobian;
	Eigen::Matrix<float, 12, 1> mInvJtranspose;
	Eigen::Matrix<float, 12, 12> inverseMassMatrix;

	float ImpulseMagnitude = 0.0f;
	static float InitialLambda;
	float eta = 0.0f;
	float deltaLambda = 0.0f;
	int constraintID = 0;
	float effectMass = 0.0f;

	Constraint(AABB& aColliderA, AABB& aColliderB) :
		ColliderA(&aColliderA),
		ColliderB(&aColliderB)
	{
		CalculateInverseMassMatrices();
	}

	virtual float Solve(float aTimestep, std::vector<Eigen::Matrix<float, 6, 1>>& aCatto_A, Eigen::Matrix<float, 12, 1>& aCurrentVelocityVector, Eigen::Matrix<float, 12, 1>& aExternalForceVector) = 0;
	virtual void CalculateJacobian() = 0;

	void CalculateInverseMassMatrices();
};