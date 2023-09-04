/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : lightingPassShader.vert
Purpose : Lighting Pass
Project : brady.m_CS350_1
Author : Brady Menendez, brady.m, 180000316
Creation date : 01/23/19
End Header--------------------------------------------------------------------*/
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 aTexCoords;

out vec2 TexCoords;
 
void main()
{
  TexCoords = aTexCoords;
  gl_Position = vec4(position, 1.0);
}