/* Start Header -----------------------------------------------------------------
File: Mesh.cpp
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#include "Mesh.h"

Mesh::Mesh()
{
}

// non default constructor
Mesh::Mesh(Vertex* verts, unsigned int numVerts, unsigned int* indices, unsigned int numIndices)
{
  Model model;

  for (unsigned int i = 0; i < numVerts; i++)
  { 
    model.positions.push_back(verts[i].GetPos());
    model.vecNormals.push_back(verts[i].GetNorm());
  }

  for (unsigned int i = 0; i < numIndices; i++)
    model.indices.push_back(indices[i]);

  Init(model);
}

Mesh::Mesh(const std::string& filename, const std::string& name, glm::vec3 color)
{
  name_ = name;
  Model model(filename, color);
  Init(model);
}

// mesh initializing
void Mesh::Init(const Model& model)
{
  model_ = model;
  drawCount_ = model.indices.size();
  vertNormCount_ = model.vertexNormals.size();
  faceNormCount_ = model.faceNormals.size();

  glGenVertexArrays(1, &vertexArrayObject_);
  glBindVertexArray(vertexArrayObject_);

  glGenBuffers(NUM_BUFFERS, vertexArrayBuffers_);
  glBindBuffer(GL_ARRAY_BUFFER, vertexArrayBuffers_[POSITION_VB]);
  glBufferData(GL_ARRAY_BUFFER, model.positions.size() * sizeof(model.positions[0]), &model.positions[0], GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glBindBuffer(GL_ARRAY_BUFFER, vertexArrayBuffers_[NORM_VB]);
  glBufferData(GL_ARRAY_BUFFER, model.vecNormals.size() * sizeof(model.vecNormals[0]), &model.vecNormals[0], GL_STATIC_DRAW);

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertexArrayBuffers_[INDEX_VB]);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, model.indices.size() * sizeof(model.indices[0]), &model.indices[0], GL_STATIC_DRAW);

  glBindVertexArray(0);

  // vec normal VAO
  glGenVertexArrays(1, &vertexNormalObject_);
  glBindVertexArray(vertexNormalObject_);

  glGenBuffers(1, vertexNormalBuffer_);
  glBindBuffer(GL_ARRAY_BUFFER, vertexNormalBuffer_[0]);
  glBufferData(GL_ARRAY_BUFFER, model.vertexNormals.size() * sizeof(model.vertexNormals[0]), &model.vertexNormals[0], GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glBindVertexArray(0);

  // face normal VAO
  glGenVertexArrays(1, &faceNormalObject_);
  glBindVertexArray(faceNormalObject_);

  glGenBuffers(1, faceNormalBuffer_);
  glBindBuffer(GL_ARRAY_BUFFER, faceNormalBuffer_[0]);
  glBufferData(GL_ARRAY_BUFFER, model.faceNormals.size() * sizeof(model.faceNormals[0]), &model.faceNormals[0], GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glBindVertexArray(0);
}

Mesh::~Mesh()
{
  glDeleteVertexArrays(1, &vertexArrayObject_);
  glDeleteVertexArrays(1, &vertexNormalObject_);
  glDeleteVertexArrays(1, &faceNormalObject_);
}

void Mesh::Draw()
{
  glBindVertexArray(vertexArrayObject_);
  glDrawElements(GL_TRIANGLES, drawCount_, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}

void Mesh::DrawVecNorm()
{
  glBindVertexArray(vertexNormalObject_);
  glDrawArrays(GL_LINES, 0, vertNormCount_);
  glBindVertexArray(0);
}

void Mesh::DrawFaceNorm()
{
  glBindVertexArray(faceNormalObject_);
  glDrawArrays(GL_LINES, 0, faceNormCount_);
  glBindVertexArray(0);
}

