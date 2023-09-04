/* Start Header -----------------------------------------------------------------
File: BoundingUtils.cpp
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#include "BoundingUtils.h"
#include <algorithm> 
#include "Object.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>
#include "glm\vec3.hpp"
#include "glm\vec4.hpp"
#include "glm\vec2.hpp"
#include <glm\gtc\quaternion.hpp>
#include <glm\mat3x3.hpp>
#include <glm\mat4x4.hpp>
#include "PhysicsManager.h"


void AddEdge(std::list<PolytopeEdge>& edgeList, const SupportPoint& a, const SupportPoint& b)
{
	for (auto iterator = edgeList.begin(); iterator != edgeList.end(); ++iterator)
	{
		if (iterator->supportPoints_[0] == b && iterator->supportPoints_[1] == a)
		{
			// erase if encoutered same edge 
			edgeList.erase(iterator);
			return;
		}
	}
	edgeList.emplace_back(a, b);
}

void BarycentricProjection(glm::vec3 point, glm::vec3 a, glm::vec3 b, glm::vec3 c, float& u, float& v, float& w)
{
	glm::vec3 v0 = b - a, v1 = c - a, v2 = point - a;
	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

// calculates bounding box
AABB* CalculateAABB(const std::vector<glm::vec3>& points)
{
	float minX = std::numeric_limits<float>::max(), minY = std::numeric_limits<float>::max(), minZ = std::numeric_limits<float>::max();
	float maxX = 0, maxY = 0, maxZ = 0;

	// gets mins and maxes from point set
	for (const glm::vec3& p : points)
	{
		if (p.x < minX) minX = p.x;
		if (p.y < minY) minY = p.y;
		if (p.z < minZ) minZ = p.z;
		if (p.x > maxX) maxX = p.x;
		if (p.y > maxY) maxY = p.y;
		if (p.z > maxZ) maxZ = p.z;
	}

	// calculates bounding box from mins and maxes
	AABB* newAABB= new AABB();
	newAABB->center_ = glm::vec3((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
	newAABB->half_extents_ = glm::vec3(maxX - newAABB->center_.x, maxY - newAABB->center_.y, maxZ - newAABB->center_.z);
	newAABB->size_ = glm::vec3(maxX - minX, maxY - minY, maxZ - minZ);


	// gets a radius as well
	float maxLength = 0.0f;
	glm::vec3 p2;
	for (const glm::vec3& p : points)
	{
		float d = glm::distance(p, newAABB->center_);
		if (d > maxLength)
		{
			maxLength = d;
			p2 = p;
		}
	}

	newAABB->sphereRadius_ = maxLength;
	newAABB->points = points;

	return newAABB;
}

AABB* CalculateAABB(const std::vector<glm::vec3>& points, const glm::vec3& scale)
{
	float minX = std::numeric_limits<float>::max(), minY = std::numeric_limits<float>::max(), minZ = std::numeric_limits<float>::max();
	float maxX = 0, maxY = 0, maxZ = 0;

	std::vector<glm::vec3> newPoints;

	for (const glm::vec3& point : points)
		newPoints.push_back(glm::vec3(point.x * scale.x, point.y * scale.y, point.z * scale.z));

	// gets mins and maxes from point set
	for (const glm::vec3& p : newPoints)
	{
		if (p.x < minX) minX = p.x;
		if (p.y < minY) minY = p.y;
		if (p.z < minZ) minZ = p.z;
		if (p.x > maxX) maxX = p.x;
		if (p.y > maxY) maxY = p.y;
		if (p.z > maxZ) maxZ = p.z;
	}

	// calculates bounding box from mins and maxes
	AABB* newAABB = new AABB();
	newAABB->center_ = glm::vec3((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
	newAABB->half_extents_ = glm::vec3(maxX - newAABB->center_.x, maxY - newAABB->center_.y, maxZ - newAABB->center_.z);
	newAABB->size_ = glm::vec3(maxX - minX, maxY - minY, maxZ - minZ);


	// gets a radius as well
	float maxLength = 0.0f;
	glm::vec3 p2;
	for (const glm::vec3& p : newPoints)
	{
		float d = glm::distance(p, newAABB->center_);
		if (d > maxLength)
		{
			maxLength = d;
			p2 = p;
		}
	}

	newAABB->sphereRadius_ = maxLength;
	newAABB->points = newPoints;
	return newAABB;
}

AABB* CalculateAABB(Object* obj)
{
	float minX = std::numeric_limits<float>::max(), minY = std::numeric_limits<float>::max(), minZ = std::numeric_limits<float>::max();
	float maxX = 0, maxY = 0, maxZ = 0;

	std::vector<glm::vec3> newPoints;

	for (const glm::vec3& point : obj->GetMesh()->GetModel().positions)
	{

		
		glm::vec4 center = obj->GetTransform()->GetModel() * glm::vec4(obj->GetAABB()->center_, 1.0f);
		glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(center)) * glm::mat4_cast(obj->GetTransform()->Rotation) * glm::scale(glm::mat4(1), obj->GetAABB()->size_);


		glm::vec4 newP = obj->GetTransform()->GetModel() * glm::vec4(point, 1.0f);
		//newPoints.push_back(glm::vec3(newP.x, newP.y, newP.z));
		newPoints.push_back(point);
	}

	// gets mins and maxes from point set
	for (const glm::vec3& p : newPoints)
	{
		if (p.x < minX) minX = p.x;
		if (p.y < minY) minY = p.y;
		if (p.z < minZ) minZ = p.z;
		if (p.x > maxX) maxX = p.x;
		if (p.y > maxY) maxY = p.y;
		if (p.z > maxZ) maxZ = p.z;
	}

	// calculates bounding box from mins and maxes
	AABB* newAABB = new AABB();
	newAABB->center_ = glm::vec3((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
	newAABB->half_extents_ = glm::vec3(maxX - newAABB->center_.x, maxY - newAABB->center_.y, maxZ - newAABB->center_.z);
	newAABB->size_ = glm::vec3(maxX - minX, maxY - minY, maxZ - minZ);


	// gets a radius as well
	float maxLength = 0.0f;
	glm::vec3 p2;
	for (const glm::vec3& p : newPoints)
	{
		float d = glm::distance(p, newAABB->center_);
		if (d > maxLength)
		{
			maxLength = d;
			p2 = p;
		}
	}

	newAABB->sphereRadius_ = maxLength;
	newAABB->points = newPoints;
	return newAABB;
}

// point inside sphere check
bool isInsideSphere(const glm::vec3& center, const float radius, const glm::vec3& point)
{
	return glm::distance(point, center) <= radius;
}


// find most separated points on a bounding box
void MostSeparatedPointsOnAABB(int &min, int &max, std::vector<glm::vec3> pt)
{
	// First find most extreme points along principal axes
	int minx = 0, maxx = 0, miny = 0, maxy = 0, minz = 0, maxz = 0;
	for (unsigned i = 1; i < pt.size(); i++) 
	{
		if (pt[i].x < pt[minx].x) minx = i;
		if (pt[i].x > pt[maxx].x) maxx = i;
		if (pt[i].y < pt[miny].y) miny = i;
		if (pt[i].y > pt[maxy].y) maxy = i;
		if (pt[i].z < pt[minz].z) minz = i;
		if (pt[i].z > pt[maxz].z) maxz = i;
	}

	// Compute the squared distances for the three pairs of points
	float dist2x = glm::dot(pt[maxx] - pt[minx], pt[maxx] - pt[minx]);
	float dist2y = glm::dot(pt[maxy] - pt[miny], pt[maxy] - pt[miny]);
	float dist2z = glm::dot(pt[maxz] - pt[minz], pt[maxz] - pt[minz]);
	// Pick the pair (min,max) of points most distant
	min = minx;
	max = maxx;

	if (dist2y > dist2x && dist2y > dist2z) {
		max = maxy;
		min = miny;
	}
	if (dist2z > dist2x && dist2z > dist2y) {
		max = maxz;
		min = minz;
	}
}

// creates a sphere from two distant points
BSphere SphereFromDistantPoints(const std::vector<glm::vec3>& points)
{
	int min, max;
	MostSeparatedPointsOnAABB(min, max, points);

	BSphere bs;
	bs.center_ = (points[min] + points[max]) * 0.5f;
	bs.radius_ = glm::dot(points[max] - bs.center_, points[max] - bs.center_);
	bs.radius_ = sqrt(bs.radius_);

	return bs;
}

// grows a sphere to fit a point
void GrowSphere(BSphere& bs, const glm::vec3& point)
{
	glm::vec3 d = point - bs.center_;
	float dist = glm::dot(d, d);

	if (dist > bs.radius_ * bs.radius_)
	{
		float dist2 = sqrt(dist);
		float newRadius = (bs.radius_ + dist2) * 0.5f;
		float k = (newRadius - bs.radius_) / dist2;
		bs.radius_ = newRadius;
		bs.center_ += d * k;
	}
}

// calculates a ritter sphere
BSphere CalculateRitterSphere(const std::vector<glm::vec3>& points)
{
	BSphere sp = SphereFromDistantPoints(points);
	
	for (const glm::vec3& p : points)
		GrowSphere(sp, p);

	// calculates vertices for drawing
	float rings = 10;
	float sectors = 10;
	float const R = 1.0f / (float)(rings - 1);
	float const S = 1.0f / (float)(sectors - 1);
	int r, s;

	float radius = sp.radius_;

	sp.positions_.resize((unsigned)(rings * sectors * 3));
	std::vector<glm::vec3>::iterator v = sp.positions_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		float const y = sinf((float)(-M_PI_2 + M_PI * r * R));
		float const x = cosf((float)(2 * M_PI * s * S)) * sinf((float)(M_PI * r * R));
		float const z = sinf((float)(2 * M_PI * s * S)) * sinf((float)(M_PI * r * R));


		*v++ = glm::vec3(x * radius, y * radius, z * radius);
	}

	sp.indices_.resize(rings * sectors * 4);
	std::vector<unsigned int>::iterator i = sp.indices_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		*i++ = r * sectors + s;
		*i++ = r * sectors + (s + 1);
		*i++ = (r + 1) * sectors + (s + 1);
		*i++ = (r + 1) * sectors + s;
	}

	return sp;
}

// Returns indices imin and imax into pt[] array of the least and most distant points along the direction dir
void ExtremePointsAlongDirection(glm::vec3 dir, const std::vector<glm::vec3>& points, int *imin, int *imax)
{
	float minproj = FLT_MAX, maxproj = -FLT_MAX;
	for (int i = 0; i < points.size(); i++) {
		// Project vector from origin to point onto direction vector
		float proj = glm::dot(points[i], dir);
		// Keep track of least distant point along direction vector
		if (proj < minproj) {
			minproj = proj;
			*imin = i;
		}
		// Keep track of most distant point along direction vector
		if (proj > maxproj) {
			maxproj = proj;
			*imax = i;
		}
	}
}

// calculates a bounding sphere from a bounding box
BSphere CalculateBSphere(const std::vector<glm::vec3>& points)
{
	float rings = 10;
	float sectors = 10;
	float const R = 1.0f / (float)(rings - 1);
	float const S = 1.0f / (float)(sectors - 1);
	int r, s;

	BSphere sp;

	AABB* bb = CalculateAABB(points);
	float radius = bb->sphereRadius_;

	// calculates vertices for drawing
	sp.positions_.resize(rings * sectors * 3);
	std::vector<glm::vec3>::iterator v = sp.positions_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		float const y = sin(-M_PI_2 + M_PI * r * R);
		float const x = cos(2 * M_PI * s * S) * sin(M_PI * r * R);
		float const z = sin(2 * M_PI * s * S) * sin(M_PI * r * R);


		*v++ = glm::vec3(x * radius, y * radius, z * radius);
	}

	sp.indices_.resize(rings * sectors * 4);
	std::vector<unsigned int>::iterator i = sp.indices_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		*i++ = r * sectors + s;
		*i++ = r * sectors + (s + 1);
		*i++ = (r + 1) * sectors + (s + 1);
		*i++ = (r + 1) * sectors + s;
	}

	delete bb;

	return sp;

}

// calculates a sphere using the EPOS-6 method
BSphere CalculateLarssonSphere(const std::vector<glm::vec3>& points)
{
	// normals to use fot the EPOS-6 method
	std::vector<glm::vec3> normals =
	{
		glm::vec3(1, 1, 0), glm::vec3(1, -1, 0), glm::vec3(1, 0, 1),
		glm::vec3(1, 0, -1), glm::vec3(0, 1, 1), glm::vec3(0, 1, -1)
	};

	float maxDist = 0.0f;
	int imin, imax;

	// finds most separate points in all normals
	for (const glm::vec3& normal : normals)
	{
		int i, j;
		ExtremePointsAlongDirection(normal, points, &i, &j);

		float distance = glm::distance(points[i], points[j]);

		if (distance > maxDist)
		{
			maxDist = distance;
			imin = i;
			imax = j;
		}
	}

	// uses the furthest pair to create a sphere, then grows it
	BSphere bs;
	bs.center_ = (points[imin] + points[imax]) * 0.5f;
	bs.radius_ = glm::dot(points[imax] - bs.center_, points[imax] - bs.center_);
	bs.radius_ = sqrt(bs.radius_);

	for (const glm::vec3& point : points)
	{
		GrowSphere(bs, point);
	}

	// calculating vertices for drawing
	float rings = 10;
	float sectors = 10;
	float const R = 1.0f / (float)(rings - 1);
	float const S = 1.0f / (float)(sectors - 1);
	int r, s;

	float radius = bs.radius_;

	bs.positions_.resize((unsigned)(rings * sectors * 3));
	std::vector<glm::vec3>::iterator v = bs.positions_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		float const y = sinf((float)(-M_PI_2 + M_PI * r * R));
		float const x = cosf((float)(2 * M_PI * s * S)) * sinf((float)(M_PI * r * R));
		float const z = sinf((float)(2 * M_PI * s * S)) * sinf((float)(M_PI * r * R));


		*v++ = glm::vec3(x * radius, y * radius, z * radius);
	}

	bs.indices_.resize(rings * sectors * 4);
	std::vector<unsigned int>::iterator i = bs.indices_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		*i++ = r * sectors + s;
		*i++ = r * sectors + (s + 1);
		*i++ = (r + 1) * sectors + (s + 1);
		*i++ = (r + 1) * sectors + s;
	}

	return bs;
}

// covariancematrix helper function for pca method
void CovarianceMatrix(glm::mat3& cov, const std::vector<glm::vec3>& points)
{
	float oon = 1.0f / (float)points.size();
	glm::vec3 c(0.0f, 0.0f, 0.0f);

	float e00, e11, e22, e01, e02, e12;
	// Compute the center of mass (centroid) of the points
	for (int i = 0; i < points.size(); i++)
		c += points[i];

	c *= oon;
	// Compute covariance elements
	e00 = e11 = e22 = e01 = e02 = e12 = 0.0f;
	for (int i = 0; i < points.size(); i++) 
	{
		// Translate points so center of mass is at origin
		glm::vec3 p = points[i] - c;
		// Compute covariance of translated points
		e00 += p.x * p.x;
		e11 += p.y * p.y;
		e22 += p.z * p.z;
		e01 += p.x * p.y;
		e02 += p.x * p.z;
		e12 += p.y * p.z;
	}
	// Fill in the covariance matrix elements
	cov[0][0] = e00 * oon;
	cov[1][1] = e11 * oon;
	cov[2][2] = e22 * oon;
	cov[0][1] = cov[1][0] = e01 * oon;
	cov[0][2] = cov[2][0] = e02 * oon;
	cov[1][2] = cov[2][1] = e12 * oon;
}

// helper function for pca method
void SymSchur2(glm::mat3& a, int p, int q, float &c, float &s)
{
	if (abs(a[p][q]) > 0.0001f) {
		float r = (a[q][q] - a[p][p]) / (2.0f * a[p][q]);
		float t;
		if (r >= 0.0f)
			t = 1.0f / (r + sqrt(1.0f + r * r));
		else
			t = -1.0f / (-r + sqrt(1.0f + r * r));
		c = 1.0f / sqrt(1.0f + t * t);
		s = t * c;
	}
	else {
		c = 1.0f;
		s = 0.0f;
	}
}

// Jacobi helper function for pca method
void Jacobi(glm::mat3& a, glm::mat3& v)
{
	int i, j, n, p, q;
    float prevoff, c, s;
	glm::mat3 J, b, t;
	// Initialize v to identify matrix
	for (i = 0; i < 3; i++) {
		v[i][0] = v[i][1] = v[i][2] = 0.0f;
		v[i][i] = 1.0f;
	}
	// Repeat for some maximum number of iterations
	const int MAX_ITERATIONS = 50;
	for (n = 0; n < MAX_ITERATIONS; n++) {
		// Find largest off-diagonal absolute element a[p][q]
		p = 0; q = 1;
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				if (i == j) continue;
				if (abs(a[i][j]) > abs(a[p][q])) {
					p = i;
					q = j;
				}
			}
		}
		// Compute the Jacobi rotation matrix J(p, q, theta)
		// (This code can be optimized for the three different cases of rotation)
		SymSchur2(a, p, q, c, s);
		for (i = 0; i < 3; i++) {
			J[i][0] = J[i][1] = J[i][2] = 0.0f;
			J[i][i] = 1.0f;
		}
		J[p][p] = c; J[p][q] = s;
		J[q][p] = -s; J[q][q] = c;
		// Cumulate rotations into what will contain the eigenvectors
		v = v * J;
		// Make ’a’ more diagonal, until just eigenvalues remain on diagonal
		a = (glm::transpose(J) * a) * J;
		// Compute "norm" of off-diagonal elements
		float off = 0.0f;
		
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				if (i == j) continue;
				off += a[i][j] * a[i][j];
			}
		}
		/* off = sqrt(off); not needed for norm comparison */
		// Stop when norm no longer decreasing
		if (n > 2 && off >= prevoff)
			return;
		prevoff = off;
	}
}

// calculates a sphere using the pca method
void PCASphere(BSphere& eigSphere, const std::vector<glm::vec3>& points)
{
	glm::mat3 m, v;
	// Compute the covariance matrix m
	CovarianceMatrix(m, points);
	// Decompose it into eigenvectors (in v) and eigenvalues (in m)
	Jacobi(m, v);
	// Find the component with largest magnitude eigenvalue (largest spread)
	glm::vec3 e;
	int maxc = 0;
	float maxf, maxe = abs(m[0][0]);
	if ((maxf = abs(m[1][1])) > maxe) maxc = 1, maxe = maxf;
	if ((maxf = abs(m[2][2])) > maxe) maxc = 2, maxe = maxf;
	e[0] = v[0][maxc];
	e[1] = v[1][maxc];
	e[2] = v[2][maxc];

	// Find the most extreme points along direction e
	int imin, imax;
	ExtremePointsAlongDirection(e, points, &imin, &imax);
	glm::vec3 minpt = points[imin];
	glm::vec3 maxpt = points[imax];
	float dist = sqrt(glm::dot(maxpt - minpt, maxpt - minpt));
	eigSphere.radius_ = dist * 0.5f;
	eigSphere.center_ = (minpt + maxpt) * 0.5f;
}

// kickstarter function for pca sphere
BSphere RecPCASphere(const std::vector<glm::vec3>& points)
{
	BSphere bs;
	// Start with sphere from maximum spread
	PCASphere(bs, points);
	// Grow sphere to include all points
	for (int i = 0; i < points.size(); i++)
		GrowSphere(bs, points[i]);

	// calculate vertices for drawing
	float rings = 10;
	float sectors = 10;
	float const R = 1.0f / (float)(rings - 1);
	float const S = 1.0f / (float)(sectors - 1);
	int r, s;

	float radius = bs.radius_;

	bs.positions_.resize(rings * sectors * 3);
	std::vector<glm::vec3>::iterator v = bs.positions_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		float const y = sin(-M_PI_2 + M_PI * r * R);
		float const x = cos(2 * M_PI * s * S) * sin(M_PI * r * R);
		float const z = sin(2 * M_PI * s * S) * sin(M_PI * r * R);


		*v++ = glm::vec3(x * radius, y * radius, z * radius);
	}

	bs.indices_.resize(rings * sectors * 4);
	std::vector<unsigned int>::iterator i = bs.indices_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		*i++ = r * sectors + s;
		*i++ = r * sectors + (s + 1);
		*i++ = (r + 1) * sectors + (s + 1);
		*i++ = (r + 1) * sectors + s;
	}

	return bs;
}

// merges two bounding boxes
AABB ComputeBoundingVolume_BB(const AABB& bb1, const AABB& bb2)
{
	glm::vec3 min1 = bb1.center_ - bb1.half_extents_;
	glm::vec3 max1 = bb1.center_ + bb1.half_extents_;
	glm::vec3 min2 = bb2.center_ - bb2.half_extents_;
	glm::vec3 max2 = bb2.center_ + bb2.half_extents_;

	float minX = (min1.x < min2.x) ? min1.x : min2.x;
	float minY = (min1.y < min2.y) ? min1.y : min2.y;
	float minZ = (min1.z < min2.z) ? min1.z : min2.z;

	float maxX = (max1.x > max2.x) ? max1.x : max2.x;
	float maxY = (max1.y > max2.y) ? max1.y : max2.y;
	float maxZ = (max1.z > max2.z) ? max1.z : max2.z;

	AABB newAABB;

	newAABB.center_ = glm::vec3((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
	newAABB.half_extents_ = glm::vec3(maxX - newAABB.center_.x, maxY - newAABB.center_.y, maxZ - newAABB.center_.z);
	newAABB.size_ = glm::vec3(maxX - minX, maxY - minY, maxZ - minZ);

	return newAABB;
}

// finds smallest pair of boxes to merge
void FindNodesToMerge_BB(const std::vector<Node>& nodes, int* i1, int* i2)
{
	float minArea = std::numeric_limits<float>::max();

	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = 0; j < nodes.size(); ++j)
		{
			if ((i != j) && (nodes[i].isMergeable_ && nodes[j].isMergeable_))
			{
				AABB bb = ComputeBoundingVolume_BB(nodes[i].bb_, nodes[j].bb_);

				float area = (bb.half_extents_.x * 2.0f) *  (bb.half_extents_.y * 2.0f) *  (bb.half_extents_.z * 2.0f);

				if (area < minArea)
				{
					minArea = area;
					*i1 = i;
					*i2 = j;
				}
		    }
		}
	}
}

// check if the tree is done building
bool TreeConstructionDone(const std::vector<Node>& nodes)
{
	bool foundMergeable = false;

	for (const Node& node : nodes)
	{
		if (node.isMergeable_)
		{
			if (foundMergeable) return false;
			else                foundMergeable = true;
		}
	}

	if (foundMergeable)
		return true;
}

// cleans up the levels before the tree is done
void SetupLevels(Node& root, std::vector<Node>& nodes, int level)
{
	root.level_ = level;
	if (root.left_)
		SetupLevels(nodes[root.leftInd_],nodes, level + 1);

	if (root.right_)
		SetupLevels(nodes[root.rightInd_], nodes, level + 1);
}

// creates a volume hierarchy tree, bottom-up method (AABB)
std::vector<Node> BottomUpBVTree(std::vector<Object*>& objects, int numObjects)
{
	assert(numObjects != 0);
	int i, j;

	std::vector<Node> nodes;

	// Form the leaf nodes for the given input objects
	for (int i = 0; i < numObjects; ++i) 
	{
		Node newNode;
		newNode.type_ = Node::LEAF;
		newNode.obj_ = objects[i];
		newNode.name_ = objects[i]->GetName();
		//newNode.obj_->SetName(objects[i].GetName());
		//newNode.bb_ = *CalculateAABB(&objects[i]);
		newNode.bb_ = *objects[i]->GetAABB();
		newNode.bb_.center_ = objects[i]->GetTransform()->GetPosition();
		newNode.level_ = 7;
		newNode.isMergeable_ = true;
		newNode.left_ = nullptr;
		newNode.right_ = nullptr;
		newNode.leftInd_ = -1;
		newNode.rightInd_ = -1;
		nodes.push_back(newNode);
	}


	while (!TreeConstructionDone(nodes)) {
		// Find indices of the two "nearest" nodes, based on some criterion
		FindNodesToMerge_BB(nodes, &i, &j);
		// Group nodes i and j together under a new internal node
		Node newNode;
		newNode.type_ = Node::NODE;
		newNode.name_ = "branch";
		newNode.left_ = &nodes[i];
		newNode.right_ = &nodes[j];
		newNode.level_ = (nodes[i].level_ < nodes[j].level_) ? nodes[i].level_ - 1 : nodes[j].level_ - 1;
		newNode.isMergeable_ = true;
		newNode.leftInd_ = i;
		newNode.rightInd_ = j;
		nodes[i].isMergeable_ = false;
		nodes[j].isMergeable_ = false;
		// Compute a bounding volume for the two nodes
		newNode.bb_ = ComputeBoundingVolume_BB(nodes[i].bb_, nodes[j].bb_);
		nodes.push_back(newNode);
	}

	// normalizing levels
	SetupLevels(nodes[nodes.size() - 1], nodes, 0);

	return nodes;
}

// merging two bounding spheres
BSphere ComputeBoundingVolume_BS(const BSphere& bs1, const BSphere& bs2)
{
	float radiisum = bs1.radius_ + bs2.radius_;
	float distance = glm::distance(bs1.center_, bs2.center_);

	BSphere sp;

	if (distance + bs1.radius_ < bs2.radius_)
	{
		return bs2;
	}
	else if (distance + bs2.radius_ < bs1.radius_)
	{
		return bs1;
	}

	sp.radius_ = (bs1.radius_ + distance + bs2.radius_) / 2.0f;
	sp.center_ = bs1.center_ + (bs2.center_ - bs1.center_) * (sp.radius_ - bs1.radius_) / glm::distance(bs2.center_, bs1.center_);
	
	//calculates vertices for new sphere
	float rings = 10;
	float sectors = 10;
	float const R = 1.0f / (float)(rings - 1);
	float const S = 1.0f / (float)(sectors - 1);
	int r, s;

	float radius = sp.radius_;

	sp.positions_.resize((unsigned)(rings * sectors * 3));
	std::vector<glm::vec3>::iterator v = sp.positions_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		float const y = sinf((float)(-M_PI_2 + M_PI * r * R));
		float const x = cosf((float)(2 * M_PI * s * S)) * sinf((float)(M_PI * r * R));
		float const z = sinf((float)(2 * M_PI * s * S)) * sinf((float)(M_PI * r * R));


		*v++ = glm::vec3(x * radius, y * radius, z * radius);
	}

	sp.indices_.resize(rings * sectors * 4);
	std::vector<unsigned int>::iterator i = sp.indices_.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		*i++ = r * sectors + s;
		*i++ = r * sectors + (s + 1);
		*i++ = (r + 1) * sectors + (s + 1);
		*i++ = (r + 1) * sectors + s;
	}

	return sp;
}

// finds smallest pair of bounding spheres to merge
void FindNodesToMerge_BS(const std::vector<Node>& nodes, int* i1, int* i2)
{
	float minVolume = std::numeric_limits<float>::max();

	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = 0; j < nodes.size(); ++j)
		{
			if ((i != j) && (nodes[i].isMergeable_ && nodes[j].isMergeable_))
			{
				BSphere bs = ComputeBoundingVolume_BS(nodes[i].bs_, nodes[j].bs_);

				float volume = (4.0f * M_PI * bs.radius_ * bs.radius_ * bs.radius_) / 3.0f;

				if (volume < minVolume)
				{
					minVolume = volume;
					*i1 = i;
					*i2 = j;
				}
			}
		}
	}
}

// bottom up bvh, sphere version
std::vector<Node> BottomUpBVTree_BSphere(std::vector<Object*>& objects, int numObjects)
{
	assert(numObjects != 0);
	int i, j;
	std::vector<Node> nodes;

	// Form the leaf nodes for the given input objects
	for (int i = 0; i < numObjects; ++i)
	{
		Node newNode;
		newNode.type_ = Node::LEAF;
		newNode.obj_ = objects[i];
		newNode.bs_ = objects[i]->GetRitterSphere();
		newNode.bs_.center_ = newNode.bs_.center_ + objects[i]->GetTransform()->GetPosition();
		newNode.level_ = 7;
		newNode.isMergeable_ = true;
		newNode.left_ = nullptr;
		newNode.right_ = nullptr;
		newNode.leftInd_ = -1;
		newNode.rightInd_ = -1;
		nodes.push_back(newNode);
	}


	while (!TreeConstructionDone(nodes)) {
		// Find indices of the two "nearest" nodes, based on some criterion
		FindNodesToMerge_BB(nodes, &i, &j);
		// Group nodes i and j together under a new internal node
		Node newNode;
		newNode.type_ = Node::NODE;
		newNode.left_ = &nodes[i];
		newNode.right_ = &nodes[j];
		newNode.level_ = (nodes[i].level_ < nodes[j].level_) ? nodes[i].level_ - 1 : nodes[j].level_ - 1;
		newNode.isMergeable_ = true;
		newNode.leftInd_ = i;
		newNode.rightInd_ = j;
		nodes[i].isMergeable_ = false;
		nodes[j].isMergeable_ = false;
		// Compute a bounding volume for the two nodes
		newNode.bs_ = ComputeBoundingVolume_BS(nodes[i].bs_, nodes[j].bs_);
		nodes.push_back(newNode);
	}

	// normalizing levels
	SetupLevels(nodes[nodes.size() - 1], nodes, 0);

	// gets rid of duplicates
	for (int i = 0; i < nodes.size(); ++i)
	{
		int currLevel_ = nodes[i].level_;

		for (int j = 0; j < nodes.size(); ++j)
		{
			if (nodes[j].bs_.center_ == nodes[i].bs_.center_ && nodes[j].bs_.radius_ == nodes[i].bs_.radius_)
			{
				if (nodes[j].level_ > nodes[i].level_)
					nodes[j].level_ = -1;
			}
		}
	}

	return nodes;
}

// compare axis functors
struct CompareX
{
	bool operator() (const glm::vec3& p1, const glm::vec3& p2)
	{
		return p1.x < p2.x;
	}
};

struct CompareY
{
	bool operator() (const glm::vec3& p1, const glm::vec3& p2)
	{
		return p1.y < p2.y;
	}
};

struct CompareZ
{
	bool operator() (const glm::vec3& p1, const glm::vec3& p2)
	{
		return p1.z < p2.z;
	}
};

bool AABB_intersection(AABB bb1, AABB bb2)
{
	if (abs(bb1.center_.x - bb2.center_.x) > (bb1.half_extents_.x + bb2.half_extents_.x)) return false;
	if (abs(bb1.center_.y - bb2.center_.y) > (bb1.half_extents_.y + bb2.half_extents_.y)) return false;
	if (abs(bb1.center_.z - bb2.center_.z) > (bb1.half_extents_.z + bb2.half_extents_.z)) return false;

	// we have an overlap
	return true;
}

bool SameSide(glm::vec3 p1, glm::vec3 p2, glm::vec3 a, glm::vec3 b)
{
	glm::vec3 cp1 = glm::cross(b - a, p1 - a);
		glm::vec3 cp2 = glm::cross(b - a, p2 - a);
		if (glm::dot(cp1, cp2) >= 0)
			return true;
		else 
			return false;
}

bool PointInTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c)
{
	if (SameSide(p, a, b, c) && SameSide(p, b, a, c) && SameSide(p, c, a, b))
		return true;
	else
		return false;
}

glm::vec3 Support(const AABB& bbA, const AABB& bbB, glm::vec3 direction)
{
	return bbA.FindFurthestPoint(direction) - bbB.FindFurthestPoint(-direction);
}

bool SameDirection(const glm::vec3& direction, const glm::vec3& ao)
{
	return glm::dot(direction, ao) > 0;
}


int sign(float x)
{
	return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

glm::vec3 AABB::FindFarthestPointInDirection(glm::vec3 aDirection)
{
	glm::vec3 extentInDirection(sign(aDirection.x) * half_extents_.x, sign(aDirection.y) * half_extents_.y, sign(aDirection.z) * half_extents_.z);
	glm::vec3 result = extentInDirection;
	return result;
}

SupportPoint Support(AABB* bb1, AABB* bb2, glm::vec3 direction, glm::mat4& model1, glm::mat4& model2)
{
	SupportPoint newSupportPoint;
	direction = glm::normalize(direction);

	// get points in opposite direcitons
	newSupportPoint.localPointA_ = bb1->FindFarthestPointInDirection(direction);
	newSupportPoint.localPointB_ = bb2->FindFarthestPointInDirection(-direction);
	newSupportPoint.worldPointA_ = glm::vec3(model1 * glm::vec4(newSupportPoint.localPointA_, 1));
	newSupportPoint.worldPointB_ = glm::vec3(model2 * glm::vec4(newSupportPoint.localPointB_, 1));

	// minkowski difference
	newSupportPoint.MHVertex_ = newSupportPoint.worldPointA_ - newSupportPoint.worldPointB_;
	return newSupportPoint;
}

glm::vec3 TripleCrossProduct(glm::vec3 A, glm::vec3 B, glm::vec3 C) {
	return (B * glm::dot(C, A)) - (A * glm::dot(C, B));
}

// calculates new direction for search
bool CheckIfSimplexContainsOrigin(Simplex& simplex, glm::vec3& searchDir)
{
	// Line
	if (simplex.Size == 2)
	{
		glm::vec3 newPointToOldPoint = simplex.b.MHVertex_ - simplex.a.MHVertex_;
		glm::vec3 newPointToOrigin = -simplex.a.MHVertex_;

		float u = 0.0f, v = 0.0f;
		Line lineSegment(simplex.a.MHVertex_, simplex.b.MHVertex_);
		Point origin(glm::vec3(0));
		Point closestPoint = ClosestPointOnLineFromTargetPoint(lineSegment, origin, u, v);

		if (v <= 0.0f)
		{
			simplex.Set(simplex.a);
			searchDir = -closestPoint.pos_;
			return false;
		}
		else if (u <= 0.0f)
		{
			simplex.Set(simplex.b);
			searchDir = -closestPoint.pos_;
			return false;
		}
		else
		{
			searchDir = -closestPoint.pos_;
			return false;
		}

	}
	// Triangle
	else if (simplex.Size == 3)
	{
		glm::vec3 newPointToOrigin = -simplex.a.MHVertex_;
		glm::vec3 edge1 = simplex.b.MHVertex_ - simplex.a.MHVertex_;
		glm::vec3 edge2 = simplex.c.MHVertex_ - simplex.a.MHVertex_;
		glm::vec3 triangleNormal = glm::cross(edge1, edge2);
		glm::vec3 edge1Normal = glm::cross(edge1, triangleNormal);
		glm::vec3 edge2Normal = glm::cross(triangleNormal, edge2);

		if (glm::dot(edge2Normal, newPointToOrigin) > 0.0f)
		{
			if (glm::dot(edge2, newPointToOrigin) > 0.0f)
			{
				searchDir = TripleCrossProduct(edge2, newPointToOrigin, edge2);
				simplex.Clear();
				simplex.Set(simplex.a, simplex.c);
				return false;
			}
			// If closer to the new simplex point 
			else
			{
				if (glm::dot(edge1, newPointToOrigin) > 0.0f)
				{
					searchDir = TripleCrossProduct(edge1, newPointToOrigin, edge1);
					simplex.Clear();
					simplex.Set(simplex.a, simplex.b);
					return false;
				}
				else 
				{
					searchDir = newPointToOrigin;
					simplex.Clear();
					simplex.Set(simplex.a);
					return false;
				}
			}
		}
		else
		{
			if (glm::dot(edge1Normal, newPointToOrigin) > 0.0f)
			{
				if (glm::dot(edge1, newPointToOrigin) > 0.0f)
				{
					searchDir = TripleCrossProduct(edge1, newPointToOrigin, edge1);
					simplex.Clear();
					simplex.Set(simplex.a, simplex.b);
					return false;
				}
				else
				{
					searchDir = newPointToOrigin;
					simplex.Clear();
					simplex.Set(simplex.a);
					return false;
				}
			}
			else
			{
				// Check if it is above the triangle
				if (glm::dot(triangleNormal, newPointToOrigin) > 0.0f)
				{
					searchDir = triangleNormal;
					// No need to change the simplex, return as [A, B, C]
					return false;
				}
				else // Has to be below the triangle, all 4 other possible regions checked
				{
					searchDir = -triangleNormal;
					// Return simplex as [A, C, B]
					simplex.Set(simplex.a, simplex.c, simplex.b);
					return false;
				}
			}
		}
	}
	else if (simplex.Size == 4) //tetrahedron
	{
		glm::vec3 edge1 = simplex.b.MHVertex_ - simplex.a.MHVertex_;
		glm::vec3 edge2 = simplex.c.MHVertex_ - simplex.a.MHVertex_;
		glm::vec3 edge3 = simplex.d.MHVertex_ - simplex.a.MHVertex_;

		glm::vec3 face1Normal = glm::cross(edge1, edge2);
		glm::vec3 face2Normal = glm::cross(edge2, edge3);
		glm::vec3 face3Normal = glm::cross(edge3, edge1);

		glm::vec3 newPointToOrigin = -simplex.a.MHVertex_;

		if (glm::dot(face1Normal, newPointToOrigin) > 0.0f)
		{
			// first face
			goto tag;
		}

		if (glm::dot(face2Normal, newPointToOrigin) > 0.0f)
		{
			// second face
			simplex.Clear();
			simplex.Set(simplex.a, simplex.c, simplex.d);
			goto tag;
		}

		if (glm::dot(face3Normal, newPointToOrigin) > 0.0f)
		{
			// third face
			simplex.Clear();
			simplex.Set(simplex.a, simplex.d, simplex.b);
			goto tag;
		}

		// simplex must contain origin
		return true;
	tag:

		glm::vec3 edge1Normal = glm::cross(edge1, face1Normal);
		if (glm::dot(edge1Normal, newPointToOrigin) > 0.0f)
		{
			searchDir = TripleCrossProduct(edge1, newPointToOrigin, edge1);
			simplex.Clear();
			simplex.Set(simplex.a, simplex.b);
			return false;
		}

		glm::vec3 edge2Normal = glm::cross(face1Normal, edge2);
		if (glm::dot(edge2Normal, newPointToOrigin) > 0.0f)
		{
			searchDir = TripleCrossProduct(edge2, newPointToOrigin, edge2);
			simplex.Clear();
			simplex.Set(simplex.a, simplex.c);
			return false;
		}

		searchDir = face1Normal;
		simplex.Clear();
		simplex.Set(simplex.a, simplex.b, simplex.c);
		return false;
	}
	return false;
}

bool ExtrapolateContactInformation(PolytopeFace* closestFace, ContactData& contactData, glm::mat4& aLocalToWorldMatrixA, glm::mat4& aLocalToWorldMatrixB)
{
	const float distanceFromOrigin = glm::dot(closestFace->faceNormal_, closestFace->supportPoints_[0].MHVertex_);

	// calculate the barycentric coordinates of the closest triangle with respect to
	// the projection of the origin onto the triangle
	float bary_u, bary_v, bary_w;

	BarycentricProjection(closestFace->faceNormal_ * distanceFromOrigin, closestFace->supportPoints_[0].MHVertex_, closestFace->supportPoints_[1].MHVertex_, closestFace->supportPoints_[2].MHVertex_, bary_u, bary_v, bary_w);

	// if any of the barycentric coefficients have a magnitude greater than 1 or lesser than and equal to 0, then the origin is not within the triangular prism described by 'triangle'
	// thus, there is no collision here
	if (fabs(bary_u) > 1.0f || fabs(bary_v) > 1.0f || fabs(bary_w) > 1.0f)
		return false;
	if (!IsValid(bary_u) || !IsValid(bary_v) || !IsValid(bary_w))
		return false;

	glm::vec3 supportLocal1 = closestFace->supportPoints_[0].localPointA_;
	glm::vec3 supportLocal2 = closestFace->supportPoints_[1].localPointA_;
	glm::vec3 supportLocal3 = closestFace->supportPoints_[2].localPointA_;

	contactData.localContactA_ = (bary_u * supportLocal1) + (bary_v * supportLocal2) + (bary_w * supportLocal3);
	contactData.worldContactA_ = aLocalToWorldMatrixA * glm::vec4(contactData.localContactA_, 1);

	supportLocal1 = closestFace->supportPoints_[0].localPointB_;
	supportLocal2 = closestFace->supportPoints_[1].localPointB_;
	supportLocal3 = closestFace->supportPoints_[2].localPointB_;

	contactData.localContactB_ = (bary_u * supportLocal1) + (bary_v * supportLocal2) + (bary_w * supportLocal3);
	contactData.worldContactB_ = aLocalToWorldMatrixB * glm::vec4(contactData.localContactB_, 1);

	contactData.collisionNormal_ = glm::normalize(closestFace->faceNormal_);
	contactData.penetrationDepth_ = distanceFromOrigin;

	return true;
}

bool EPAContactDetection(Simplex& simplex, AABB* bb1, AABB* bb2, ContactData& contactData)
{
	const float exitThreshold = 0.0001f;
	const unsigned iterationLimit = 50;
	unsigned iterationCount = 0;

	std::list<PolytopeFace> polytopeFaces;
	std::list<PolytopeEdge> polytopeEdges;

	// add all faces of simplex to the polytope
	polytopeFaces.emplace_back(simplex.a, simplex.b, simplex.c);
	polytopeFaces.emplace_back(simplex.a, simplex.c, simplex.d);
	polytopeFaces.emplace_back(simplex.a, simplex.d, simplex.b);
	polytopeFaces.emplace_back(simplex.b, simplex.d, simplex.c);

	while (true)
	{
		if (iterationCount++ >= iterationLimit)
			return false;

		// find the closest face to origin 
		float minimumDistance = std::numeric_limits<float>::max();
		std::list<PolytopeFace>::iterator closestFace = polytopeFaces.begin();
		for (auto iterator = polytopeFaces.begin(); iterator != polytopeFaces.end(); ++iterator)
		{
			float distance = glm::dot(iterator->faceNormal_, iterator->supportPoints_[0].MHVertex_);
			if (distance < minimumDistance)
			{
				minimumDistance = distance;
				closestFace = iterator;
			}
		}

		SupportPoint newPolytopePoint = Support(bb1, bb2, closestFace->faceNormal_, bb1->localToWorldMatrix_, bb2->localToWorldMatrix_);

		if (glm::dot(closestFace->faceNormal_, newPolytopePoint.MHVertex_) - minimumDistance < exitThreshold)
		{
			LineLoop closestFaceObjectA;
			closestFaceObjectA.AddVertex(closestFace->supportPoints_[0].worldPointA_);
			closestFaceObjectA.AddVertex(closestFace->supportPoints_[1].worldPointA_);
			closestFaceObjectA.AddVertex(closestFace->supportPoints_[2].worldPointA_);

			LineLoop closestFaceObjectB;
			closestFaceObjectB.AddVertex(closestFace->supportPoints_[0].worldPointB_);
			closestFaceObjectB.AddVertex(closestFace->supportPoints_[1].worldPointB_);
			closestFaceObjectB.AddVertex(closestFace->supportPoints_[2].worldPointB_);

			LineLoop closestPolytopeFace;
			closestPolytopeFace.AddVertex(closestFace->supportPoints_[0].MHVertex_);
			closestPolytopeFace.AddVertex(closestFace->supportPoints_[1].MHVertex_);
			closestPolytopeFace.AddVertex(closestFace->supportPoints_[2].MHVertex_);

			return ExtrapolateContactInformation(&(*closestFace), contactData, bb1->localToWorldMatrix_, bb2->localToWorldMatrix_);
		}

		// check for seen faces
		for (auto iterator = polytopeFaces.begin(); iterator != polytopeFaces.end();)
		{
			// seen face is support points is in the same halfspace
			glm::vec3 planeVector = newPolytopePoint.MHVertex_ - iterator->supportPoints_[0].MHVertex_;

			if (glm::dot(iterator->faceNormal_, planeVector) > 0.0f)
			{
				// adds outside seen edges
				AddEdge(polytopeEdges, iterator->supportPoints_[0], iterator->supportPoints_[1]);
				AddEdge(polytopeEdges, iterator->supportPoints_[1], iterator->supportPoints_[2]);
				AddEdge(polytopeEdges, iterator->supportPoints_[2], iterator->supportPoints_[0]);

				iterator = polytopeFaces.erase(iterator);
				continue;
			}
			++iterator;
		}

		for (auto iterator = polytopeEdges.begin(); iterator != polytopeEdges.end(); ++iterator)
			polytopeFaces.emplace_back(newPolytopePoint, iterator->supportPoints_[0], iterator->supportPoints_[1]);

		polytopeEdges.clear();
	}
}



bool GJKCollisionHandler(AABB* bb1, AABB* bb2, ContactData& contactData)
{


	Simplex simplex;
	// random search vector
	glm::vec3 searchDirection = glm::vec3(1, 1, 1);
	SupportPoint newSupportPoint = Support(bb1, bb2, searchDirection, bb1->localToWorldMatrix_, bb2->localToWorldMatrix_);

	if (glm::dot(searchDirection, newSupportPoint.MHVertex_) >= glm::length(newSupportPoint.MHVertex_) * 0.8f)
	{
		searchDirection = glm::vec3(0, 1, 0);
		newSupportPoint = Support(bb1, bb2, searchDirection, bb1->localToWorldMatrix_, bb2->localToWorldMatrix_);
	}
	simplex.Push(newSupportPoint);

	searchDirection *= -1.0f;

	const unsigned iterationLimit = 75;
	unsigned iterationCount = 0;

	while (true)
	{
		if (iterationCount++ >= iterationLimit)
			return false;

		if (glm::length(searchDirection) <= 0.0001f)
			return false;

		SupportPoint newSupportPoint = Support(bb1, bb2, searchDirection, bb1->localToWorldMatrix_, bb2->localToWorldMatrix_);
		simplex.Push(newSupportPoint);

		// minkowski difference check for not colliding
		if (glm::dot(newSupportPoint.MHVertex_, searchDirection) < 0.0f)
		{
			return false;
		}
		else
		{
			// If the new point IS past the origin, check if the simplex contains the origin, 
			// If it doesn't modify search direction to point towards to origin
			if (CheckIfSimplexContainsOrigin(simplex, searchDirection))
			{
				g_PhysicsManager->currSimplex.Set(simplex.a, simplex.b, simplex.c, simplex.d);
				
				// pass simplex to EPA
				return EPAContactDetection(simplex, bb1, bb2, contactData);
			}
		}
	}
}


void printVec(glm::vec3 a)
{
	std::cout << a.x << ", " << a.y << ", " << a.z << std::endl;
}


bool DetectCollision_MidPhase(Object* S, Node* treeNode, std::vector<Node>& nodes, std::vector<std::vector<glm::vec3>>& simplices)
{
	// if treeNode == LEAF, then perform GJK
	if (treeNode->type_ == Node::LEAF)
	{
		// b. use BBox of treeNode geometry – approximate solution
		AABB spherebox = *S->GetAABB();
		spherebox.center_ = S->GetTransform()->GetPosition();

     	//auto [colBool, colPoints] = GJK(spherebox, treeNode->bb_, simplices);

		ContactData cd;

		// Calculate the model matrices and store in contact data for future use
		glm::mat4 model1, model2;
		glm::mat4 translate = glm::translate(S->GetTransform()->GetPosition());
		glm::mat4 rotate = glm::mat4_cast(S->GetTransform()->GetRotation());
		glm::mat4 scale = glm::scale(S->GetTransform()->GetScale());
		model1 = translate * rotate * scale;

		translate = glm::translate(treeNode->bb_.pOwner_->GetTransform()->GetPosition());
		rotate = glm::mat4_cast(treeNode->bb_.pOwner_->GetTransform()->GetRotation());
		scale = glm::scale(treeNode->bb_.pOwner_->GetTransform()->GetScale());
		model2 = translate * rotate * scale;

		spherebox.localToWorldMatrix_ = model1;
		treeNode->bb_.localToWorldMatrix_ = model2;
		
		
		if (GJKCollisionHandler(&spherebox, &treeNode->bb_, cd))
		{
			std::cout << "Collided With: " << treeNode->name_ << std::endl;
			std::cout << "ContactPoint1 (World): ";
			printVec(cd.worldContactA_);
			std::cout << "ContactPoint2 (World): ";
			printVec(cd.worldContactB_);
			std::cout << "Normal: ";
			printVec(cd.collisionNormal_);
			std::cout << "Penetration Depth: " << cd.penetrationDepth_ << std::endl;
			return true;
		}

		//// checks for gjk collision
		//if (colBool)
		//{
		//	std::cout << "Collided With: " << treeNode->name_ << std::endl;
		//	std::cout << "collisionNormal_: " << colPoints.collisionNormal_.x << ", " << colPoints.collisionNormal_.y << ", " << colPoints.collisionNormal_.z << std::endl;
		//	std::cout << "Penetration Depth: " << colPoints.penetrationDepth_ << std::endl;
		//	return true;
		//}
	}

	AABB nodeBox = treeNode->bb_;
	AABB sphereBox = *S->GetAABB();
	sphereBox.center_ = S->GetTransform()->GetPosition();

	// preliminary aabb check
	if (!AABB_intersection(nodeBox, sphereBox))
		return false;

	// calls midphase function again on its children, if it has any

	if (treeNode->leftInd_ != -1)
	{
		if (DetectCollision_MidPhase(S, &nodes[treeNode->leftInd_], nodes, simplices))
			return true;
	}

	if (treeNode->rightInd_ != -1)
	{
		if (DetectCollision_MidPhase(S, &nodes[treeNode->rightInd_], nodes, simplices))
			return true;
	}

	// no intersection with the scene
	return false;
}

bool DetectCollision_BroadPhase(Object* S, std::vector<Node>& nodes, std::vector<std::vector<glm::vec3>>& simplices)
{
	// The broad phase part of collision detection
	AABB worldBox;

	for (auto node : nodes)
	{
		if (node.level_ == 0)
		{
			worldBox = node.bb_;
			break;
		}
	}

	AABB sphereBox = *S->GetAABB();
	sphereBox.center_ = S->GetTransform()->GetPosition();
	
	// preliminary check
	if (!AABB_intersection(worldBox, sphereBox))
		return false;

	for (auto node : nodes)
	{
		// checks if node is root
		if (node.level_ == 0)
		{
			// calls the midphase function on its children
			if (node.leftInd_ != -1)
			{
				if (DetectCollision_MidPhase(S, &nodes[node.leftInd_], nodes, simplices))
					return true;
			}
			
			if (node.rightInd_ != -1)
			{
				if (DetectCollision_MidPhase(S, &nodes[node.rightInd_], nodes, simplices))
					return true;
			}
				
			break;
		}
	}

	// no intersection with the scene
	return false;
}