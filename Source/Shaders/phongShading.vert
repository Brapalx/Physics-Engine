/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : phongShading.vert
Purpose : Phong Shading
Project : brady.m_CS300_2
Author : Brady Menendez, brady.m, 180000316
Creation date : 10/15/18
End Header--------------------------------------------------------------------*/
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform int texMode;

// Interpolating vertex attributes over the rasterizer
out VS_OUT
{
    vec3 rasterColor;
	vec3 specColor;
	vec4 normal0;
	vec4 fragPos;
	vec3 rawPos;
	vec3 rawNorm;
} vs_out;

uniform mat4 transform;
uniform mat4 camTransform;
uniform vec4 camPos;
uniform mat4 worldTransform;

void main()
{
  vs_out.fragPos = worldTransform *  vec4( position, 1.0f );
  vs_out.normal0 = transform * vec4( normal, 0.0f );  
  gl_Position = transform * vec4( position, 1.0f );
  
  vs_out.rawPos = position;
  vs_out.rawNorm = normal;
}
