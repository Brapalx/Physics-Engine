/* Start Header -----------------------------------------------------------------
File: Object.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#include "Object.h"
#include "PhysicsManager.h"

Object::Object(std::string name, Mesh* mesh, Shader* shader, Texture* text, Texture* sText, Camera* cam, glm::vec3 pos, glm::vec3 scale, bool physics)
{
	name_ = name;
	mesh_ = mesh;
	shader_ = shader;
	cam_ = cam;
	transform_ = new Transform();
	transform_->SetPosition(pos);
	transform_->SetScale(scale);
	transform_->SetOwner(this);
	initPos_ = pos;

	dTexture_ = text;
	sTexture_ = sText;
	textureMode_ = 0;
	UVMode_ = 0;
	TEMode_ = 0;
	emmisiveL_ = glm::vec3(0.0f);
	ambMat_ = glm::vec3(0.0f);

	bb_ = CalculateAABB(mesh_->GetModel().positions, scale);
	bb_->pOwner_ = this;

	float width = bb_->half_extents_.x * 2 * scale.x;
	float height = bb_->half_extents_.y * 2 * scale.y;
	float depth = bb_->half_extents_.z * 2 * scale.z;

	// inertia tensor (cube)
	bb_->InertiaTensor[0][0] = (1.0f / 12) * (powf(depth, 2) + powf(height, 2));
	bb_->InertiaTensor[0][1] = 0;
	bb_->InertiaTensor[0][2] = 0;

	bb_->InertiaTensor[1][0] = 0;
	bb_->InertiaTensor[1][1] = (1.0f / 12) * (powf(width, 2) + powf(height, 2));
	bb_->InertiaTensor[1][2] = 0;

	bb_->InertiaTensor[2][0] = 0;
	bb_->InertiaTensor[2][1] = 0;
	bb_->InertiaTensor[2][2] = (1.0f / 12) * (powf(depth, 2) + powf(width, 2));

	g_PhysicsManager->RegisterColliderObject(bb_);

	bs_ = CalculateBSphere(mesh_->GetModel().positions);
	ritter_ = CalculateRitterSphere(mesh_->GetModel().positions);
	larssonSphere_ = CalculateLarssonSphere(mesh_->GetModel().positions);
	pcaSphere_ = RecPCASphere(mesh_->GetModel().positions);


	if (physics)
	{
		rb_ = new RigidBody();
		rb_->SetOwner(this);
		g_PhysicsManager->RegisterPhysicsObject(rb_);
	}
}

void Object::SetPlatform()
{
	rb_->gravityEnabled_ = false;
	bb_->eColliderType = AABB::STATIC;
}

void Object::Draw(std::unordered_map<int, Light> lights, LightGlobalData lData, bool vertNorm, bool faceNorm)
{

	// binds shader and both textures
	shader_->Bind();
	dTexture_->Bind();
	shader_->SetTexture(0);
	sTexture_->Bind();
	shader_->SetTexture(1);
	shader_->Update(transform_, *cam_, lights, lData);
	shader_->UpdateObjectMats(ambMat_, emmisiveL_, textureMode_, UVMode_, TEMode_);
	mesh_->Draw();

	if (vertNorm) mesh_->DrawVecNorm();
	if (faceNorm) mesh_->DrawFaceNorm();
}

void Object::DrawDeferred()
{
	std::unordered_map<int, Light> a;
	LightGlobalData lg{};

	dTexture_->Bind();
	shader_->SetTexture(0);
	sTexture_->Bind();
	shader_->SetTexture(1);
	shader_->Update(transform_, *cam_, a, lg);
	shader_->UpdateObjectMats(ambMat_, emmisiveL_, textureMode_, UVMode_, TEMode_);
	mesh_->Draw();
}

void Object::DrawDebug(bool vertNorm, bool faceNorm, bool velocity)
{
	shader_->Bind();
	std::unordered_map<int, Light> a;
	LightGlobalData lg{};
	shader_->Update(transform_, *cam_, a, lg);
	if (vertNorm) mesh_->DrawVecNorm();
	if (faceNorm) mesh_->DrawFaceNorm();

	if (!velocity)
		return;

	std::vector<glm::vec3> points;
	float length = 1.0f;

	points.push_back(bb_->center_);
	points.push_back(bb_->center_ + glm::normalize(rb_->currLinearVelocity_) * length);

	//std::cout << rb_->currLinearVelocity_.y << std::endl;

	GLuint velObject;
	GLuint velBuffer[1];

	// orbit lines VAO
	glGenVertexArrays(1, &velObject);
	glBindVertexArray(velObject);

	glGenBuffers(1, velBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, velBuffer[0]);
	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(points[0]), &points[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);

	glLineWidth(100);
	glEnable(GL_LINE_SMOOTH);


	glBindVertexArray(velObject);
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glBindVertexArray(0);
	glDeleteVertexArrays(1, &velObject);
	glDeleteBuffers(1, &velBuffer[0]);
	glDisable(GL_LINE_SMOOTH);
	glLineWidth(1);


}