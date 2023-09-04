/* Start Header -----------------------------------------------------------------
File: ObjectLoader.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <string>

struct Model
{
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> vecNormals;
  std::vector<glm::vec3> vecColors;
  std::vector<glm::vec3> vertexNormals;
  std::vector<glm::vec3> faceNormals;
  std::vector<unsigned int> indices;
  void CalculateVectorNormals();
  Model() : positions(0), vecNormals(0), indices(0) {}
  Model(const std::string& OBJfilename, glm::vec3 color = glm::vec3(1,0,0));
};

struct OBJ
{
  std::vector<int> indices;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  OBJ(const std::string& fileName);
  Model OBJToModel();
};

