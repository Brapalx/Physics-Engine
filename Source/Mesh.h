/* Start Header -----------------------------------------------------------------
File: Mesh.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/

#pragma once
#include <GL/glew.h>
#include <string>
#include "OBJLoader.h"

class Vertex
{
	glm::vec3 pos_;
  glm::vec3 normal_;
  glm::vec2 texCoord_;
public:
	Vertex(const glm::vec3& pos, const glm::vec2& texCoord = glm::vec2(0,0),const glm::vec3& normal = glm::vec3(0,0,0)) : pos_(pos), normal_(normal), texCoord_(texCoord) { }
  glm::vec3 GetPos() { return pos_; }
  glm::vec3 GetNorm() { return normal_; }
};


class Mesh
{
  enum { POSITION_VB, INDEX_VB, NORM_VB, NUM_BUFFERS};
  GLuint vertexArrayObject_;
  GLuint vertexArrayBuffers_[NUM_BUFFERS];
  GLuint vertexNormalObject_;
  GLuint vertexNormalBuffer_[1];
  GLuint faceNormalObject_;
  GLuint faceNormalBuffer_[1];
  
  unsigned int drawCount_;
  unsigned int vertNormCount_;
  unsigned int faceNormCount_;
  Model model_;
  std::string name_;
  void Init(const Model& model);
  

  public:
    Mesh();
    Mesh(Vertex* verts, unsigned int numVerts, unsigned int* indices, unsigned int numIndices);
    Mesh(const std::string& filename, const std::string& name, glm::vec3 color = glm::vec3(1, 0, 0));
    void Draw();
    void DrawVecNorm();
    void DrawFaceNorm();
    inline std::string GetName() {return name_;}
	inline Model& GetModel() { return model_; }
    virtual ~Mesh();


};
