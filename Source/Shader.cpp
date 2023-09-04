/* Start Header -----------------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name: Shader.cpp
Purpose: Defines the shader class and its functions
Language: C++, MSVC
Platform: MSVC15, Windows 10
Project: brady.m_CS350_1
Author: Brady Menendez, brady.m, 180000316
Creation date: 9/29/18
End Header --------------------------------------------------------------------*/

#pragma once
#include "Shader.h"
#include <fstream>
#include <iostream>
#include <vector>

// function declarations
static void CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string& errorMessage);
static std::string LoadShader(const std::string& fileName);
static GLuint CreateShader(const std::string& text, GLenum shaderType);

Shader::Shader()
{
}

// loads shaders from files
Shader::Shader(const std::string& filename, const std::string& name)
{
  name_ = name;
  program_ = glCreateProgram();
  shaders_[0] = CreateShader(LoadShader(filename + ".vert"), GL_VERTEX_SHADER);
  shaders_[1] = CreateShader(LoadShader(filename + ".frag") , GL_FRAGMENT_SHADER);

  for (unsigned int i = 0; i < NUM_SHADERS; i++)
    glAttachShader(program_, shaders_[i]);

  glLinkProgram(program_);
  CheckShaderError(program_, GL_LINK_STATUS, true, "ERROR: PROGRAM LINKING FAILED");

  glValidateProgram(program_);
  CheckShaderError(program_, GL_VALIDATE_STATUS, true, "ERROR: INVALID PROGRAM");

  uniforms_[TRANSFORM_U] = glGetUniformLocation(program_, "transform");
  uniforms_[CAM_TRANSFORM_U] = glGetUniformLocation(program_, "camTransform");
  uniforms_[W_TRANSFORM_U] = glGetUniformLocation(program_, "worldTransform");
  uniforms_[CAM_POS_U] = glGetUniformLocation(program_, "camPos");
}

void Shader::Update(Transform* transform, const Camera& cam, std::unordered_map<int, Light> lights, LightGlobalData lData)
{

  glm::mat4 model = cam.GetViewProjection() * transform->GetModel();
  glUniformMatrix4fv(uniforms_[TRANSFORM_U], 1, GL_FALSE, &model[0][0]);

  glm::mat4 camModel = cam.GetViewProjection();
  glUniformMatrix4fv(uniforms_[CAM_TRANSFORM_U], 1, GL_FALSE, &camModel[0][0]);

  glm::mat4 tModel = transform->GetModel();
  glUniformMatrix4fv(uniforms_[W_TRANSFORM_U], 1, GL_FALSE, &tModel[0][0]);

  glm::vec3 a = cam.GetPos();

  float pos[4];
  pos[0] = a.x;
  pos[1] = a.y;
  pos[2] = a.z;
  pos[3] = 1.0f;

  // setting many uniforms
  glUniform4f(uniforms_[CAM_POS_U], a.x, a.y, a.z, 1.0f);
  glUniform1i(glGetUniformLocation(program_, "lightCount"), lights.size());
  glUniform1f(glGetUniformLocation(program_, "c1"), lData.c1_);
  glUniform1f(glGetUniformLocation(program_, "c2"), lData.c2_);
  glUniform1f(glGetUniformLocation(program_, "c3"), lData.c3_);
  glUniform3f(glGetUniformLocation(program_, "fog"), lData.fog_[0], lData.fog_[1], lData.fog_[2]);
  glUniform3f(glGetUniformLocation(program_, "ambient"), lData.ambient_[0], lData.ambient_[1], lData.ambient_[2]);

  // sends all light data to the shader 
  if(!lights.empty())
  { 
    int counter = 0;
    for (auto light : lights)
    { 
      std::string builder;

      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].pos";
      glUniform3f(glGetUniformLocation(program_, builder.c_str()), light.second.position_.x, light.second.position_.y, light.second.position_.z);

      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].ambColor";
      glUniform3f(glGetUniformLocation(program_, builder.c_str()), light.second.ambientColor_.x, light.second.ambientColor_.y, light.second.ambientColor_.z);

      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].diffColor";
      glUniform3f(glGetUniformLocation(program_, builder.c_str()), light.second.diffuseColor_.x, light.second.diffuseColor_.y, light.second.diffuseColor_.z);

      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].specColor";
      glUniform3f(glGetUniformLocation(program_, builder.c_str()), light.second.specularColor_.x, light.second.specularColor_.y, light.second.specularColor_.z);
 
      builder = "lights[";
      builder += std::to_string(counter);
	  builder += "].inAngle";
      glUniform1f(glGetUniformLocation(program_, builder.c_str()), light.second.innerAngle_);

      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].outAngle";
      glUniform1f(glGetUniformLocation(program_, builder.c_str()), light.second.outerAngle_);

      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].falloff";
      glUniform1f(glGetUniformLocation(program_, builder.c_str()), light.second.spotFalloff_);
 
      builder = "lights[";
      builder += std::to_string(counter);
      builder += "].type";
      glUniform1i(glGetUniformLocation(program_, builder.c_str()), light.second.type_);

      counter++;
    }
  }

  if (this->name_ == "Light")
  {
    glUniform3f(glGetUniformLocation(program_, "lightC"), transform->GetLightC().x, transform->GetLightC().y, transform->GetLightC().z);
  }
}

void Shader::Bind()
{
  for (unsigned int i = 0; i < NUM_SHADERS; i++)
  { 
    glDetachShader(program_, shaders_[i]);
    glDeleteShader(shaders_[i]);
  }
  glUseProgram(program_);
}

void Shader::UpdateObjectMats(const glm::vec3& amb, const glm::vec3& emm, int mode, int uvmode, int temode)
{
  glUniform3f(glGetUniformLocation(program_, "Ka"), amb.x, amb.y, amb.z);
  glUniform3f(glGetUniformLocation(program_, "I_emmisive"), emm.x, emm.y, emm.z);
  glUniform1i(glGetUniformLocation(program_, "texMode"), mode);
}

void Shader::SetTexture(unsigned int unit)
{
  std::string builder = "tex";
  builder.push_back(std::to_string(unit)[0]);
  glUniform1i(glGetUniformLocation(program_, builder.c_str()), unit);
}

Shader::~Shader()
{
  glDeleteProgram(program_);
}

// helper function
static std::string LoadShader(const std::string& fileName)
{
  std::ifstream file;
  file.open(fileName.c_str());
  std::string output, line;

  if (file.is_open())
  {
    while (file.good())
    {
      std::getline(file,line);
      output.append(line + "\n");
    }
  }
  else
  {
    std::cerr << "UNABLE TO LOAD SHADER: " << fileName << std::endl;
  }

  return output;
}

// checks for shader errors
static void CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string& errorMessage)
{
  GLint success = 0;
  GLchar error[1024] = { 0 };

  if (isProgram)
    glGetProgramiv(shader, flag, &success);
  else
    glGetShaderiv(shader, flag, &success);

  if (success == GL_FALSE)
  {
    if (isProgram)
      glGetProgramInfoLog(shader, sizeof(error), NULL, error);
    else
      glGetShaderInfoLog(shader, sizeof(error), NULL, error);

    std::cerr << errorMessage << ": '" << error << "'" << std::endl;
  }
}

// helper function
static GLuint CreateShader(const std::string& text, GLenum shaderType)
{
  GLuint shader = glCreateShader(shaderType);

  if (shader == 0)
    std::cerr << "ERROR: SHADER CREATION FAILED" << std::endl;

  const GLchar* shaderSourceLines[1];
  GLint shaderSourceLengths[1];

  shaderSourceLines[0] = text.c_str();
  shaderSourceLengths[0] = text.length();

  glShaderSource(shader, 1, shaderSourceLines, shaderSourceLengths);
  glCompileShader(shader);

  CheckShaderError(shader, GL_COMPILE_STATUS, false, "ERROR: SHADER COMPILATION FAILED");
  return shader;
}

void Shader::setInt(const std::string &name, int value)
{
	glUniform1i(glGetUniformLocation(program_, name.c_str()), value);
}

void Shader::setTransform(const glm::mat4& mat)
{
	GLuint loc = glGetUniformLocation(program_, "transform");
	glUniformMatrix4fv(loc, 1, GL_FALSE, &mat[0][0]);
}

