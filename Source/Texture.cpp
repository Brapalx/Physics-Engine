/* Start Header -----------------------------------------------------------------
File: Texture.cpp
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#include "Texture.h"
#include <sstream>
#include <fstream>
#include <glm/glm.hpp>

Texture::Texture(const std::string& filename, unsigned int unit)
{
  // loads the file
  data_ = LoadPPM(filename);
  unit_ = unit;

  // generates the texture
  glGenTextures(1, &texture_);
  glActiveTexture(GL_TEXTURE0 + unit_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_FLOAT, data_.data());
}

void Texture::Bind()
{
  glActiveTexture(GL_TEXTURE0 + unit_);
  glBindTexture(GL_TEXTURE_2D, texture_);
}

Texture::~Texture()
{
  glDeleteTextures(1, &texture_);
}

std::vector<glm::vec3> Texture::LoadPPM(const std::string& filename)
{
  std::ifstream infile(filename);
  std::string line;

  // skip some lines
  std::getline(infile, line);
  std::getline(infile, line);

  // get image dimensions
  std::getline(infile, line);
  width_ = std::stof(line.substr(0, line.find_first_of(' ')));
  height_ = std::stof(line.substr(line.find_first_of(' ') + 1, line.find_last_not_of(' ')));

  std::getline(infile, line);
  std::vector<glm::vec3> data;

  // get RGB values
  while (std::getline(infile, line))
  {
	float r, g, b;
      r =std::stof(line.substr(0, line.find_last_not_of(' ') + 1));

	  std::getline(infile, line);
	  g = std::stof(line.substr(0, line.find_last_not_of(' ') + 1));

	  std::getline(infile, line);
	  b = std::stof(line.substr(0, line.find_last_not_of(' ') + 1));

      data.push_back(glm::vec3(r / 255.0f, g / 255.0f, b/255.0f));
  }


  return data;
}