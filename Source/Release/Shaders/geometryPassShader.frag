/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : geometryPassShader.frag
Purpose : G-Pass
Project : brady.m_CS350_1
Author : Brady Menendez, brady.m, 180000316
Creation date : 01/23/19
End Header--------------------------------------------------------------------*/
#version 330 core
layout (location = 0) out vec3 gPosition;
layout (location = 1) out vec3 gNormal;
layout (location = 2) out vec3 gTex;
layout (location = 3) out vec3 gSpec;

in vec3 FragPos;
in vec4 Normal;
in vec3 rawPos;
in vec3 rawNorm;

uniform sampler2D tex0;
uniform sampler2D tex1;

uniform int texMode;

void main()
{
  gPosition = FragPos;
  gNormal = normalize(vec3(Normal));
  
  vec3 position = rawPos;
  vec3 normal = normalize(rawNorm);
  
  if (texMode == 0) // cylindrical
  {
	vec2 texC = vec2(atan(position.z / position.x), position.y);
	gTex = texture(tex0, texC).rgb;
    gSpec = texture(tex1, texC).rgb;
  }
  else if (texMode == 1) // spherical
  {
	vec2 texC = vec2(atan(position.z / position.x), acos(position.y / (sqrt(position.x * position.x + position.y * position.y + position.z * position.z))));
	gTex = texture(tex0, texC).rgb;
    gSpec = texture(tex1, texC).rgb;
  }
  else if (texMode == 2) // planar
  {
    float aX = abs(position.x);
	float aY = abs(position.y);
	float aZ = abs(position.z);
    int faceCase = 0;
	  
	if (aX >= aY && aX >= aZ)
	{
	  if (position.x >= 0.0f) faceCase = 0;
	  else               faceCase = 1;
	}
	else if (aY >= aX && aY >= aZ)
	{
	  if (position.y >= 0.0f) faceCase = 2;
	  else               faceCase = 3;
	}
	else if (aZ >= aX && aZ >= aY)
	{
	  if (position.z >= 0.0f) faceCase = 4;
	  else                    faceCase = 5;
	}

	float u, v;

	switch (faceCase)
	{
	  case 0:
	  u = -position.z / aX;
	  v = position.y / aX;
	  break;
		
	  case 1:
	  u = position.z / aX;
	  v = position.y / aX;
	  break;

	  case 2:
	  u = position.x / aY;
	  v = -position.z / aY;
	  break;

	  case 3:
	  u = position.x / aY;
	  v = position.z / aY;
	  break;

	  case 4:
	  u = position.x / aZ;
	  v = position.y / aZ;
	  break;

	  case 5:
	  u = -position.x / aZ;
	  v = position.y / aZ;
	  break;
	}

	u = (u + 1.0f) / 2.0f;
	v = (v + 1.0f) / 2.0f;
	gTex = texture(tex0, vec2(u,v)).rgb;
    gSpec = texture(tex1, vec2(u,v)).rgb; 
  } 
}