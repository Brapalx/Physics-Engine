/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : geometryPassShader.vert
Purpose : G-Pass
Project : brady.m_CS350_1
Author : Brady Menendez, brady.m, 180000316
Creation date : 01/23/19
End Header--------------------------------------------------------------------*/
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec4 Normal;
out vec3 rawPos;
out vec3 rawNorm;

uniform mat4 transform;
uniform mat4 camTransform;
uniform vec4 camPos;
uniform mat4 worldTransform;

void main()
{
    vec4 worldPos = transform * vec4(aPos, 1.0);
    FragPos = worldPos.xyz; 
    
    Normal = transform * vec4( aNormal, 0.0f );  

    gl_Position = transform * vec4(aPos, 1.0);
	
	rawPos = aPos;
	rawNorm = aNormal;
}