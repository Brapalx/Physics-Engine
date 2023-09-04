#pragma once
#include "Libraries/Eigen/Dense"
#include "Constraint.h"
#include "BoundingUtils.h"

// to prevent objects from collision
class ContactConstraint : public Constraint
{

public:
	ContactData ConstraintData;
	int manifoldID = 0;

	float NormalImpulseSum = 0.0f;
	float TangentImpulseSum1 = 0.0f;
	float TangentImpulseSum2 = 0.0f;

	ContactConstraint(AABB& bb1, AABB& bb2) : Constraint(bb1, bb2)
	{}
	virtual void CalculateJacobian() override;
	virtual float Solve(float step, std::vector<Eigen::Matrix<float, 6, 1>>& A, Eigen::Matrix<float, 12, 1>& currVelocity, Eigen::Matrix<float, 12, 1>& Fext) override;
};