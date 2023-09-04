/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : phongShading.frag
Purpose : Phong Shading
Project : brady.m_CS300_2
Author : Brady Menendez, brady.m, 180000316
Creation date : 10/15/18
End Header--------------------------------------------------------------------*/
#version 330 core

in VS_OUT
{
    vec3 rasterColor;
	vec3 specColor;
	vec4 normal0;
	vec4 fragPos;
	vec3 rawPos;
	vec3 rawNorm;
} fs_in;

struct Light
{
  vec3 pos;
  vec3 ambColor;
  vec3 diffColor;
  vec3 specColor;
  float inAngle;
  float outAngle;
  float falloff;
  int type;
};

uniform Light lights[16];
uniform int lightCount;

uniform mat4 camTransform;
uniform vec4 camPos;

uniform sampler2D tex0;
uniform sampler2D tex1;

uniform float c1;
uniform float c2;
uniform float c3;
uniform vec3 fog;
uniform vec3 ambient;
uniform vec3 I_emmisive;

uniform int texMode;

uniform vec3 Ka;
vec3 Kd;
vec3 Ks = fs_in.specColor;
float ns = 200.5f;

float zNear = 1.0f;
float zFar = 30.0f;

out vec3 color;

void main()
{ 
  vec3 position = fs_in.rawPos;
  vec3 normal = normalize(fs_in.rawNorm);
  
  if (texMode == 0) // cylindrical
  {
	vec2 texC = vec2(atan(position.z / position.x), position.y);
	Kd = texture(tex0, texC).rgb;
    Ks = texture(tex1, texC).rgb;
  }
  else if (texMode == 1) // spherical
  {
	vec2 texC = vec2(atan(position.z / position.x), acos(position.y / (sqrt(position.x * position.x + position.y * position.y + position.z * position.z))));
	Kd = texture(tex0, texC).rgb;
    Ks = texture(tex1, texC).rgb;
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
	Kd = texture(tex0, vec2(u,v)).rgb;
    Ks = texture(tex1, vec2(u,v)).rgb; 
  }  
  
  vec3 finalColor = vec3(0.0f);
 
  vec4 norm = normalize(fs_in.normal0);
 
  for(int i = 0; i < lightCount; i++)
  {
    vec4 lightPos = camTransform * vec4(lights[i].pos,1.0f);
   
    // Ambient
    vec3 I_ambient = (ambient + lights[i].ambColor) * Ka;
   
    if (lights[i].type == 0) // point lights
    {
      // Diffuse
      vec4 L = normalize(lightPos - fs_in.fragPos);
      float N_dot_L = dot(norm, L );
      vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

      // Specular 
      vec4 R = 2 * N_dot_L * norm - L;
      vec4 V = normalize(-fs_in.fragPos);
  
      vec3 I_specular = Ks * lights[i].specColor * pow(max(dot(R, V),0),ns);

      // Attenutation terms
      float att = min(1/(c1 + c2 * length(L) + c3 * length(L) * length(L)), 1.0f);
      vec3 I_local = I_emmisive + att * (I_ambient + I_diffuse + I_specular);
  
      float s = (zFar - length(V))/(zFar-zNear);

      // Final color
      finalColor += s * I_local + (1-s) * fog;
    }
    else if (lights[i].type == 1) // directional lights
    {
	  vec4 center = camTransform * vec4(0.0f,0.0f,0.0f,1.0f);
	  vec4 lightDir = center - lightPos;
	
      // Diffuse
      vec4 L = normalize(-lightDir);
      float N_dot_L = dot(norm, L );
      vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

      // Specular 
      vec4 R = 2 * N_dot_L * norm - L;
      vec4 V = normalize(-fs_in.fragPos);
  
      vec3 I_specular = Ks * lights[i].specColor * pow(max(dot(R, V),0),ns);

      vec3 I_local = I_emmisive + (I_ambient + I_diffuse + I_specular);

      // Final color
      finalColor += I_local;
    }
    else if (lights[i].type == 2) // spotlights
    {
	  vec4 center = camTransform * vec4(0.0f,0.0f,0.0f,1.0f);
      vec4 lightDir = normalize(center - lightPos);
	  vec4 rayDir = normalize(lightPos - fs_in.fragPos);	
	  float angle = acos(dot(-rayDir,lightDir));
	
	  if (angle > lights[i].inAngle && angle <= lights[i].outAngle) // if in between in and out
	  {
	    // Diffuse
        vec4 L = normalize(lightPos - fs_in.fragPos);
        float N_dot_L = dot(norm, L );
        vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

        // Specular
        vec4 R = 2 * N_dot_L * norm - L;
        vec4 V = normalize(-fs_in.fragPos);
 
        vec3 I_specular = Ks * lights[i].specColor * pow(max(dot(R, V),0),ns);

        // Attenutation terms
        float att = min(1/(c1 + c2 * length(L) + c3 * length(L) * length(L)), 1.0f);
        vec3 I_local = I_emmisive + att * (I_ambient + I_diffuse + I_specular);
  
        float s = (zFar - length(V))/(zFar-zNear);
	    vec3 result = s * I_local + (1-s) * fog;
	  
	    float mult =  pow((lights[i].outAngle - angle)/(lights[i].outAngle - lights[i].inAngle), lights[i].falloff);

        // Final color
        finalColor += mult * result;
	
	  }
	  else if (angle > lights[i].outAngle) // if outside
	  {
	    finalColor += I_ambient;
	  }
	  else // if inside
	  {
        // Diffuse
        vec4 L = normalize(lightPos - fs_in.fragPos);
        float N_dot_L = dot(norm, L );
        vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

        // Specular
        vec4 R = 2 * N_dot_L * norm - L;
        vec4 V = normalize(-fs_in.fragPos);
        vec3 I_specular = Ks * lights[i].specColor * pow(max(dot(R, V),0),ns);

        // Attenutation terms
        float att = min(1/(c1 + c2 * length(L) + c3 * length(L) * length(L)), 1.0f);
        vec3 I_local = I_emmisive + att * (I_ambient + I_diffuse + I_specular);
  
        float s = (zFar - length(V))/(zFar-zNear);

        // Final color
        finalColor += s * I_local + (1-s) * fog;
	  }
    }
  }
  color = finalColor;
}
