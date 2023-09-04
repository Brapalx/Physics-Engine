/* Start Header -----------------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name: Shader.h
Purpose: Declares the shader class and its functions
Language: C++, MSVC
Platform: MSVC15, Windows 10
Project: brady.m_CS350_1
Author: Brady Menendez, brady.m, 180000316
Creation date: 9/29/18
End Header --------------------------------------------------------------------*/

#pragma once
#include <string>
#include <GL/glew.h>
#include "Transform.h"
#include "Camera.h"
#include <vector>
#include <unordered_map>
#include "Light.h"

class Shader
{
  Shader(const Shader& other);
  enum {TRANSFORM_U, CAM_TRANSFORM_U, W_TRANSFORM_U, CAM_POS_U, NUM_UNIFORMS};

  GLuint program_;
  static const unsigned int NUM_SHADERS = 2;
  GLuint shaders_[NUM_SHADERS];
  GLuint uniforms_[NUM_UNIFORMS];
  std::string name_;

  public:

  Shader();
  Shader(const std::string& filename, const std::string& name);
  std::string GetName() { return name_;}
  void Bind();
  void SetTexture(unsigned int unit);
  void setInt(const std::string &name, int value);
  void setTransform(const glm::mat4& mat);
  void UpdateObjectMats(const glm::vec3& amb, const glm::vec3& emm, int mode, int uvmode, int temode);
  void Update(Transform* transform, const Camera& cam, std::unordered_map<int, Light> lights, LightGlobalData lData);
  GLuint GetProgram() { return program_; };
  virtual ~Shader();
};
