/* Start Header -----------------------------------------------------------------
File: ObjectLoader.cpp
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/

#include "objLoader.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <map>
#include <sstream>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

enum FACE {POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z};

// helper function
std::vector<std::string> SplitString(const std::string &s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> tokens;

  while (getline(ss, item, delim))
    tokens.push_back(item);

  return tokens;
}

// vector predicates
bool vecXcompare(glm::vec3 a, glm::vec3 b)
{
	return (a.x < b.x);
}

bool vecYcompare(glm::vec3 a, glm::vec3 b)
{
	return (a.y < b.y);
}

bool vecZcompare(glm::vec3 a, glm::vec3 b)
{
	return (a.z < b.z);
}

// reads in the obj file
OBJ::OBJ(const std::string& fileName)
{

  std::ifstream file;
  file.open(fileName.c_str());

  std::string line;
  if (file.is_open())
  {
    while (file.good())
    {
      getline(file, line);

      if (line.length() < 2) continue;

      if(line[0] == 'v' && (line[1] == ' ' || line[1] == '\t'))
      { 
        std::vector<std::string> tokens = SplitString(line, ' ');
        this->vertices.push_back(glm::vec3(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3])));
      }
      else if (line[0] == 'f')
      {
        std::vector<std::string> tokens = SplitString(line, ' ');

        this->indices.push_back(std::stoi(tokens[1]));
        this->indices.push_back(std::stoi(tokens[2]));
        this->indices.push_back(std::stoi(tokens[3]));
        
        if ((int)tokens.size() > 4)
        {
          this->indices.push_back(std::stoi(tokens[1]));
          this->indices.push_back(std::stoi(tokens[3]));
          this->indices.push_back(std::stoi(tokens[4]));
        }
      }
    }

	// scaling vertices to -1,1
	float minX = (*std::min_element(this->vertices.begin(), this->vertices.end(), vecXcompare)).x;
	float maxX = (*std::max_element(this->vertices.begin(), this->vertices.end(), vecXcompare)).x;
	float minY = (*std::min_element(this->vertices.begin(), this->vertices.end(), vecYcompare)).y;
	float maxY = (*std::max_element(this->vertices.begin(), this->vertices.end(), vecYcompare)).y;
	float minZ = (*std::min_element(this->vertices.begin(), this->vertices.end(), vecZcompare)).z;
	float maxZ = (*std::max_element(this->vertices.begin(), this->vertices.end(), vecZcompare)).z;

	glm::vec3 min(minX, minY, minZ);
	glm::vec3 max(maxX, maxY, maxZ);
	glm::vec3 scale(max - min);
	float sle = glm::length(scale);
	
	glm::mat4 sc = glm::scale(glm::vec3(1.0f/sle, 1.0f/sle, 1.0/sle));
	glm::mat4 tr = glm::translate(-(min + (scale / 2.0f)));

	for (unsigned int i = 0; i < this->vertices.size(); i++)
		this->vertices[i] = sc * tr * glm::vec4(this->vertices[i],1.0f);
  }
  else
  {
    std::cerr << "UNABLE TO LOAD OBJ FILE: " << fileName << std::endl;
  }
}

glm::vec3 VectorAverage(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
{
  float x = (v1.x + v2.x + v3.x) / 3.0f;
  float y = (v1.y + v2.y + v3.y) / 3.0f;
  float z = (v1.z + v2.z + v3.z) / 3.0f;

  return glm::vec3(x,y,z);
}

bool ParallelPred(glm::vec3 v1, glm::vec3 v2)
{
  return glm::dot(v1, v2) == 1;
}

void Model::CalculateVectorNormals()
{
  for (unsigned int i = 0; i < indices.size(); i += 3)
  {
    int i0 = indices[i];
    int i1 = indices[i + 1];
    int i2 = indices[i + 2];

    glm::vec3 v1 = positions[i1] - positions[i0];
    glm::vec3 v2 = positions[i2] - positions[i0];

    glm::vec3 normal = glm::normalize(glm::cross(v1, v2));

    vecNormals[i0] += normal;
    vecNormals[i1] += normal;
    vecNormals[i2] += normal;

    glm::vec3 posAverage = VectorAverage(positions[i0], positions[i1], positions[i2]);
    faceNormals.push_back(posAverage);
    faceNormals.push_back(posAverage + glm::normalize(VectorAverage(vecNormals[i0], vecNormals[i1], vecNormals[i2])) * 0.08f);
  }

  for (unsigned int i = 0; i < positions.size(); i++)
    vecNormals[i] = glm::normalize(vecNormals[i]);

  for (unsigned int i = 0; i < positions.size(); i++)
  {
    vertexNormals.push_back(positions[i]);
    vertexNormals.push_back(positions[i] + vecNormals[i] * 0.08f);
  }
}


Model OBJ::OBJToModel()
{
  Model normalModel;

  unsigned int numIndices = indices.size();

  std::vector<int> indexLookup(indices);

  std::sort(indexLookup.begin(), indexLookup.end());

  std::map<int, unsigned int> normalModelIndexMap;

  for (unsigned int i = 0; i < numIndices; i++)
  {
    int currentIndex = indices[i];

    glm::vec3 currentPosition = vertices[currentIndex - 1];
    glm::vec3 currentNormal;

    currentNormal = glm::vec3(0, 0, 0);

    unsigned int normalModelIndex;

    //Create model to properly generate normals on
    std::map<int, unsigned int>::iterator it = normalModelIndexMap.find(currentIndex);
    if (it == normalModelIndexMap.end())
    {
      normalModelIndex = normalModel.positions.size();
      normalModelIndexMap.insert(std::pair<int, unsigned int>(currentIndex, normalModelIndex));
      normalModel.positions.push_back(currentPosition);
      normalModel.vecNormals.push_back(currentNormal);
    }
    else
      normalModelIndex = it->second;

    normalModel.indices.push_back(normalModelIndex);
  }

  normalModel.CalculateVectorNormals();
  return normalModel;
};

Model::Model(const std::string& OBJfilename, glm::vec3 color)
{
  *this = (OBJ(OBJfilename).OBJToModel());

  for (unsigned int i = 0; i < positions.size(); i++)
   vecColors.push_back(color);
}

