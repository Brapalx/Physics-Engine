/* Start Header -----------------------------------------------------------------
File: BoundingUtils.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once

#include <vector>
#include <array>
#include <string>
#include <glm/glm.hpp>

#include "Libraries/Eigen/Dense"

class Object;

// aabb struct
struct AABB
{
	enum ColliderType
	{
		STATIC,
		DYNAMIC,
		KINEMATIC,
		ColliderTypeCount
	};

	bool canCollide_;
	bool isColliding_ = false;
	std::vector<glm::vec3> points;
	int colliderID_ = 0;
	ColliderType eColliderType = ColliderType::DYNAMIC;
	Eigen::Matrix<float, 1, 6> ContactJacobian;
	glm::mat3 InertiaTensor = glm::mat3(1);
	Eigen::Matrix<float, 6, 6> InverseMassMatrix;
	float restitution_ = 0.2;
	glm::mat4 localToWorldMatrix_;

	Object* pOwner_;
	glm::vec3 center_;
	glm::vec3 half_extents_;
	glm::vec3 size_;
   
	float sphereRadius_;

	glm::vec3 FindFarthestPointInDirection(glm::vec3 aDirection);

	glm::vec3 FindFurthestPoint(glm::vec3 direction) const
	{
		glm::vec3 maxPoint;
		float   maxDistance = -FLT_MAX;

		for (const glm::vec3& vertex : points) 
		{
			float distance = glm::dot(vertex, direction);
			if (distance > maxDistance) 
			{
				maxDistance = distance;
				maxPoint = vertex;
			}
		}
		return maxPoint;
	}
};

struct BSphere
{
	glm::vec3 center_;
	float radius_;
	
	std::vector<glm::vec3> positions_;
	std::vector<unsigned int> indices_;

	glm::vec3 FindFurthestPoint(glm::vec3 direction) const
	{
		glm::vec3 maxPoint;
		float   maxDistance = -FLT_MAX;

		for (const glm::vec3& vertex : positions_)
		{
			float distance = glm::dot(vertex, direction);
			if (distance > maxDistance)
			{
				maxDistance = distance;
				maxPoint = vertex;
			}
		}

		return maxPoint;
	}
};

struct OBB
{
	glm::vec3 center_;
	glm::vec3 axis_;
	glm::vec3 half_extents_;
};

class Object;

struct Node
{
	enum nType {NODE, LEAF};

	Object* obj_;
	nType type_;
	Node* left_;
	Node* right_;
	AABB bb_;
	BSphere bs_;
	int level_;
	bool isMergeable_;
	int leftInd_;
	int rightInd_;
	std::string name_;
};

typedef Node* NodePtr;

struct Point
{
	glm::vec3 pos_;
	Point(glm::vec3 p) : pos_(p)
	{}
};

struct Line
{
	Point A;
	Point B;
	Line(glm::vec3 a, glm::vec3 b) : A(a), B(b)
	{}
};

inline Point ClosestPointOnLineFromTargetPoint(Line& aLine, Point& aTargetPoint, float& u, float& v)
{
	glm::vec3 lineSegment = aLine.B.pos_ - aLine.A.pos_;

	glm::vec3 normalized = glm::normalize(lineSegment);
	v = glm::dot(-aLine.A.pos_, normalized) / glm::length(lineSegment);
	u = glm::dot(aLine.B.pos_, normalized) / glm::length(lineSegment);
	glm::vec3 closestPoint;
	if (u <= 0.0f)
	{
		closestPoint = aLine.B.pos_;
	}
	else if (v <= 0.0f)
	{
		closestPoint = aLine.A.pos_;
	}
	else
	{
		closestPoint = u * aLine.A.pos_ + v * aLine.B.pos_;
	}

	return Point(closestPoint);
}

struct SupportPoint
{
	glm::vec3 MHVertex_;
	glm::vec3 worldPointA_;
	glm::vec3 worldPointB_;
	glm::vec3 localPointA_;
	glm::vec3 localPointB_;

	bool operator == (const SupportPoint& ref) { return MHVertex_ == ref.MHVertex_; }
};

struct Simplex
{
	SupportPoint points[4];
	int Size = 0;

	SupportPoint& a;
	SupportPoint& b;
	SupportPoint& c;
	SupportPoint& d;

	inline void Clear() { Size = 0; }
	Simplex() :
		a(points[0]),
		b(points[1]),
		c(points[2]),
		d(points[3])
	{}

	inline void Set(SupportPoint a, SupportPoint b, SupportPoint c, SupportPoint d) { Size = 4, this->a = a; this->b = b; this->c = c; this->d = d; }
	inline void Set(SupportPoint a, SupportPoint b, SupportPoint c) { Size = 3, this->a = a; this->b = b; this->c = c; }
	inline void Set(SupportPoint a, SupportPoint b) { Size = 2, this->a = a; this->b = b; }
	inline void Set(SupportPoint a) { Size = 1, this->a = a; }

	inline void Push(SupportPoint aNewVertex)
	{
		Size = std::min(Size + 1, 4);

		for (int i = Size - 1; i > 0; i--)
			points[i] = points[i - 1];

		points[0] = aNewVertex;
	}
};


struct LineLoop
{
	LineLoop() {}
	std::vector<glm::vec3> LineLoopVertices;

	inline void AddVertex(glm::vec3& aPosition)
	{
		if (aPosition == glm::vec3(0, 0, 0))
			return;

		LineLoopVertices.push_back(aPosition);
	}

};

struct PolytopeFace
{
	SupportPoint supportPoints_[3];
	glm::vec3 faceNormal_;

	PolytopeFace(const SupportPoint& supportA, const SupportPoint& supportB, const SupportPoint& supportC)
	{
		supportPoints_[0] = supportA;
		supportPoints_[1] = supportB;
		supportPoints_[2] = supportC;
		glm::vec3 edge1 = supportB.MHVertex_ - supportA.MHVertex_;
		glm::vec3 edge2 = supportC.MHVertex_ - supportA.MHVertex_;
		faceNormal_ = glm::cross(edge1, edge2);
	}
};

struct PolytopeEdge
{
	SupportPoint supportPoints_[2];

	PolytopeEdge(const SupportPoint& supportA, const SupportPoint& supportB)
	{
		supportPoints_[0] = supportA;
		supportPoints_[1] = supportB;
	}
};


struct ContactData
{
	glm::vec3 worldContactA_;
	glm::vec3 worldContactB_;
	glm::vec3 localContactA_;
	glm::vec3 localContactB_;
	glm::vec3 collisionNormal_;

	float penetrationDepth_;
};

struct ContactManifold
{
	int manifoldID_ = 0;
	int constraintID_ = 0;
	ContactData points_[4];
	int Size = 0;

	ContactData& a;
	ContactData& b;
	ContactData& c;
	ContactData& d;

	inline void Clear() { Size = 0; }
	ContactManifold() :
		a(points_[0]),
		b(points_[1]),
		c(points_[2]),
		d(points_[3])
	{}

	inline void Set(ContactData a, ContactData b, ContactData c, ContactData d) { Size = 4, this->a = a; this->b = b; this->c = c; this->d = d; }
	inline void Set(ContactData a, ContactData b, ContactData c) { Size = 3, this->a = a; this->b = b; this->c = c; }
	inline void Set(ContactData a, ContactData b) { Size = 2, this->a = a; this->b = b; }
	inline void Set(ContactData a) { Size = 1, this->a = a; }

	inline void Push(ContactData aNewContact)
	{
		Size = std::min(Size + 1, 4);

		for (int i = Size - 1; i > 0; i--)
			points_[i] = points_[i - 1];

		points_[0] = aNewContact;
	}
};

inline bool IsValid(float x)
{
	int ix = *reinterpret_cast<int*>(&x);
	return (ix & 0x7f800000) != 0x7f800000;
}

// function declarations
AABB* CalculateAABB(const std::vector<glm::vec3>& points);
AABB* CalculateAABB(const std::vector<glm::vec3>& points, const glm::vec3& scale);
BSphere CalculateBSphere(const std::vector<glm::vec3>& points);
BSphere CalculateRitterSphere(const std::vector<glm::vec3>& points);
BSphere CalculateLarssonSphere(const std::vector<glm::vec3>& points);
BSphere RecPCASphere(const std::vector<glm::vec3>& points);

std::vector<Node> BottomUpBVTree(std::vector<Object*>& objects, int numObjects);
std::vector<Node> BottomUpBVTree_BSphere(std::vector<Object*>& objects, int numObjects);
std::vector<Node> TopDownBVTree(std::vector<Object> objects, bool pointSizeCutoffMode);
std::vector<Node> TopDownBVTree_BS(std::vector<Object> objects, bool pointSizeCutoffMode);
bool DetectCollision_BroadPhase(Object* S, std::vector<Node>& nodes, std::vector<std::vector<glm::vec3>>& simplices);

bool GJKCollisionHandler(AABB* aCollider1, AABB* aCollider2, ContactData& aContactData);
bool EPAContactDetection(Simplex& aSimplex, AABB* aCollider1, AABB* aCollider2, ContactData& aContactData);
bool ExtrapolateContactInformation(PolytopeFace* aClosestFace, ContactData& aContactData, glm::mat4& aLocalToWorldMatrixA, glm::mat4& aLocalToWorldMatrixB);
bool CheckIfSimplexContainsOrigin(Simplex& aSimplex, glm::vec3& aSearchDirection);