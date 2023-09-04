/* Start Header-----------------------------------------------------------------
Copyright(C) 2019 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : orbitShader.vert
Purpose : basic shader for debug drawing
Project : brady.m_CS350_2
Author : Brady Menendez, brady.m, 180000316
Creation date : 2/20/19
End Header--------------------------------------------------------------------*/
#version 330 core

layout (location = 0) in vec3 position;

uniform mat4 transform;

void main()
{
  gl_Position = transform * vec4(position.x, position.y, position.z, 1.0);
}
