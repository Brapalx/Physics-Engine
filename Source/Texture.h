/* Start Header -----------------------------------------------------------------
File: Texture.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#include <GL/glew.h>
#include <string>
#include <vector>
#include <glm/glm.hpp>

class Texture
{
  GLuint texture_;
  std::vector<glm::vec3> data_;
  float width_;
  float height_;
  unsigned int unit_;
  Texture(const Texture& other){}
  void operator=(const Texture& other){}

  public:

  Texture(const std::string& filename, unsigned int unit);
  void Bind();
  virtual ~Texture();
  std::vector<glm::vec3> LoadPPM(const std::string& filename);
};