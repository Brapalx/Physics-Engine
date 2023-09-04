/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : orbitShader.vert
Purpose : basic shader for the orbit
Project : brady.m_CS300_2
Author : Brady Menendez, brady.m, 180000316
Creation date : 10/18/18
End Header--------------------------------------------------------------------*/
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 color;

out vec3 normal0;
out vec3 fragPos;
out vec3 rasterColor;
uniform mat4 transform;

void main()
{
  gl_Position = transform * vec4(position.x, position.y, position.z, 1.0);
  normal0 = (transform * vec4(normal.x, normal.y, normal.z, 0.0)).xyz;
  fragPos = vec3(transform * vec4(position,1.0));
  rasterColor = color;
}
