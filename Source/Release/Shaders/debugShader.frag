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

out vec4 color;

uniform vec3 col = vec3(1.0f, 0.0f, 0.0f);

void main()
{
  color = vec4(col,1.0);
}
