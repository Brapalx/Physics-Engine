/* Start Header -----------------------------------------------------------------
File: Constraint.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#include "Object.h"
#include "BoundingUtils.h"


class FramerateController;
class InputManager;
class RigidBody;
class AABB;
class Constraint;
class Engine;

class PhysicsManager
{
public:
	static int IntegratorIterations;
	const static int ConstraintSolverIterations = 10;

	std::vector<RigidBody*> PhysicsObjectsList;
	std::vector<AABB*> ColliderObjectsList;
	std::vector<Constraint*> ConstraintObjectsList;
	std::vector<ContactManifold*> ManifoldObjectsList;

	PhysicsManager() {};
	~PhysicsManager() {};

	void RegisterPhysicsObject(RigidBody* aNewPhysics);
	void RegisterColliderObject(AABB* aNewCollider);
	void RegisterConstraintObject(Constraint* aNewConstraint);
	void RegisterManifoldObject(ContactManifold* aNewManifold);

	void Update();
	void Simulation();
	void DetectCollisions();
	void SolveConstraints();

	Simplex currSimplex;
};

extern PhysicsManager* g_PhysicsManager;