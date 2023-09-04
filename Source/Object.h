/* Start Header -----------------------------------------------------------------
File: Object.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#include "Transform.h"
#include "Mesh.h"
#include "Camera.h"
#include "Shader.h"
#include <string>
#include <unordered_map>
#include "Light.h"
#include "Texture.h"
#include "BoundingUtils.h"
#include "RigidBody.h"


class Object
{
  std::string name_;
  Transform* transform_;
  glm::vec3 lightColor_;
  Mesh* mesh_;
  std::string meshName_;
  Shader* shader_;
  Camera* cam_;
  Texture* dTexture_;
  Texture* sTexture_;
  glm::vec3 emmisiveL_;
  glm::vec3 ambMat_;
  int textureMode_;
  int UVMode_;
  int TEMode_;
  AABB* bb_;
  BSphere bs_;
  BSphere ritter_;
  BSphere pcaSphere_;
  BSphere larssonSphere_;
  RigidBody* rb_;
 
  

  public:
	  Object(std::string name, Mesh* mesh, Shader* shader, Texture* text, Texture* sText, Camera* cam, glm::vec3 pos, glm::vec3 scale, bool physics = false);
  Object() {};

  inline std::string GetName() const { return name_; }
  inline Transform* GetTransform() { return transform_; }
  inline std::string GetMeshName() {return mesh_->GetName();}
  inline Mesh* GetMesh() {return mesh_;}
  inline std::string GetShaderName() { return shader_->GetName(); }
  inline glm::vec3 GetEmissive() {return emmisiveL_;}
  inline glm::vec3 GetAmbMat() { return ambMat_;}
  inline int GetTexMode() { return textureMode_;}
  inline int GetUVMode() { return UVMode_;}
  inline int GetTEMode() { return TEMode_;}
  inline AABB* GetAABB() { return bb_;}
  inline BSphere& GetBSphere() { return bs_;}
  inline BSphere& GetRitterSphere() { return ritter_;}
  inline BSphere& GetLarssonSphere() { return larssonSphere_;}
  inline BSphere& GetPCASphere() { return pcaSphere_;}
  inline RigidBody* GetRigidBody() { return rb_; }

  inline void SetName(std::string n) { name_ = n; };
  inline void SetMesh(Mesh* mesh){ mesh_ = mesh; }
  inline void SetShader(Shader* s) { shader_ = s; }
  inline void SetLightColor(const glm::vec3& c) { transform_->SetLightC(c); }
  inline void SetEmissive(const glm::vec3& e) { emmisiveL_ = e; }
  inline void SetAmbMat(const glm::vec3& a) { ambMat_ = a; }
  inline void SetTexMode(const unsigned int& m) { textureMode_ = m; }
  inline void SetUVMode(const int& u) { UVMode_ = u; }
  inline void SetTEMode(const int& e) { TEMode_ = e; }
  void SetPlatform();

  glm::vec3 initPos_;

  void Draw(std::unordered_map<int, Light> lights, LightGlobalData lData, bool vertNorm = false, bool faceNorm = false);
  void DrawDeferred();
  void DrawDebug(bool vertNorm = false, bool faceNorm = false, bool velocity = false);


};