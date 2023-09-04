/* Start Header-----------------------------------------------------------------
Copyright(C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name : lightingPassShader.frag
Purpose : Lighting Pass
Project : brady.m_CS350_1
Author : Brady Menendez, brady.m, 180000316
Creation date : 01/23/19
End Header--------------------------------------------------------------------*/
#version 330 core

in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D gTex;
uniform sampler2D gSpec;

uniform int renderTargetMode;

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
uniform mat4 worldTransform;
uniform mat4 transform;

uniform float c1;
uniform float c2;
uniform float c3;
uniform vec3 fog;
uniform vec3 ambient;
uniform vec3 I_emmisive;

uniform int texMode;

uniform vec3 Ka;
float ns = 200.5f;

float zNear = 1.0f;
float zFar = 30.0f;

out vec3 color;

void main()
{ 
  if (renderTargetMode == 1)
  {
    color = texture(gPosition, TexCoords).rgb;
  }
  else if (renderTargetMode == 2)
  {
    color = texture(gNormal, TexCoords).rgb;
  }
  else if (renderTargetMode == 3)
  {
    color = texture(gTex, TexCoords).rgb;
  }
  else if (renderTargetMode == 4)
  {
    color = texture(gSpec, TexCoords).rgb;
  }
  else
  {
  vec3 FragPos = texture(gPosition, TexCoords).rgb;
  vec4 norm = vec4(texture(gNormal, TexCoords).rgb,0.0f);
  vec3 Kd = texture(gTex, TexCoords).rgb;
  vec3 Ks = texture(gSpec, TexCoords).rgb;
  
  color = FragPos;
  
  vec3 finalColor = vec3(0.0f);
 
 
  for(int i = 0; i < lightCount; i++)
  {
    vec4 lightPos = camTransform * vec4(lights[i].pos,1.0f);
   
    // Ambient
    vec3 I_ambient = (ambient + lights[i].ambColor) * Ka;
   
    if (lights[i].type == 0) // point lights
    {
      // Diffuse
      vec4 L = normalize(lightPos - vec4(FragPos,1.0f));
      float N_dot_L = dot(norm, L );
      vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

      // Specular 
      vec4 R = 2 * N_dot_L * norm - L;
      vec4 V = normalize(vec4(-FragPos,1.0f));
  
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
      vec4 V = normalize(vec4(-FragPos,1.0f));
  
      vec3 I_specular = Ks * lights[i].specColor * pow(max(dot(R, V),0),ns);

      vec3 I_local = I_emmisive + (I_ambient + I_diffuse + I_specular);

      // Final color
      finalColor += I_local;
    }
    else if (lights[i].type == 2) // spotlights
    {
	  vec4 center = camTransform * vec4(0.0f,0.0f,0.0f,1.0f);
      vec4 lightDir = normalize(center - lightPos);
	  vec4 rayDir = normalize(lightPos - vec4(FragPos,1.0f));	
	  float angle = acos(dot(-rayDir,lightDir));
	
	  if (angle > lights[i].inAngle && angle <= lights[i].outAngle) // if in between in and out
	  {
	    // Diffuse
        vec4 L = normalize(lightPos - vec4(FragPos,1.0f));
        float N_dot_L = dot(norm, L );
        vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

        // Specular
        vec4 R = 2 * N_dot_L * norm - L;
        vec4 V = normalize(vec4(-FragPos,1.0f));
 
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
        vec4 L = normalize(lightPos - vec4(FragPos,1.0f));
        float N_dot_L = dot(norm, L );
        vec3 I_diffuse = Kd * lights[i].diffColor * max(N_dot_L,0);

        // Specular
        vec4 R = 2 * N_dot_L * norm - L;
        vec4 V = normalize(vec4(-FragPos,1.0f));
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
}
