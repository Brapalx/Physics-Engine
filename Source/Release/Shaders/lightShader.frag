/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : lightShader.frag
Purpose : LightShader
Project : brady.m_CS300_2
Author : Brady Menendez, brady.m, 180000316
Creation date : 10/1/18
End Header--------------------------------------------------------------------*/
#version 330 core

in vec3 normal0;
in vec3 fragPos;
in vec3 rasterColor;
out vec4 color;

uniform vec3 lightC;

void main()
{
  float ambientStrength = 0.9;
  vec3 ambient = ambientStrength * lightC;
 
  
  vec3 result = ambient;
  color = vec4(result,1.0);
}
