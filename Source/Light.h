/* Start Header -----------------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name: Light.h
Purpose: Declares the light structure and some light functions.
Project: brady.m_CS350_1
Author: Brady Menendez, brady.m, 180000316
Creation date: 10/16/10
End Header --------------------------------------------------------------------*/
#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

// light struct, has all individual light variables
struct Light
{
  enum LIGHT_TYPE
  {
    POINT, DIRECTIONAL, SPOTLIGHT
  };
  
  LIGHT_TYPE type_;
  std::string parent_;
  glm::vec3 ambientColor_;
  glm::vec3 diffuseColor_;
  glm::vec3 specularColor_;
  glm::vec3 position_;
  float innerAngle_;
  float outerAngle_;
  float spotFalloff_;
  bool isEnabled_;

  // returns a point light
  static Light MakePointLight(std::string par, glm::vec3 pos, glm::vec3 col)
  {
    Light pL;
    pL.parent_ = par;
    pL.type_ = Light::POINT;
    pL.ambientColor_ = glm::vec3(0.0f);
    pL.diffuseColor_ = col;
    pL.specularColor_ = col;
    pL.position_ = pos;
	pL.isEnabled_ = false;

    return pL;
  }

  // returns a directional light
  static Light MakeDirectionalLight(std::string par, glm::vec3 pos, glm::vec3 col)
  {
    Light pL;
    pL.parent_ = par;
    pL.type_ = Light::DIRECTIONAL;
    pL.ambientColor_ = glm::vec3(0.0f);
    pL.diffuseColor_ = col;
    pL.specularColor_ = col;
    pL.position_ = pos;
	pL.isEnabled_ = false;

    return pL;
  }

  // returns a spot light
  static Light MakeSpotLight(std::string par, glm::vec3 pos, glm::vec3 col, float inAngle, float outAngle, float falloff)
  {
    Light pL;
    pL.parent_ = par;
    pL.type_ = Light::SPOTLIGHT;
    pL.ambientColor_ = glm::vec3(0.0f);
    pL.diffuseColor_ = col;
    pL.specularColor_ = col;
    pL.position_ = pos;
    pL.innerAngle_ = glm::radians(inAngle);
    pL.outerAngle_ = glm::radians(outAngle);
    pL.spotFalloff_ = falloff;
	pL.isEnabled_ = false;
    return pL;
  }
};

// global light data struct
struct LightGlobalData
{
  float c1_, c2_, c3_;
  float fog_[3];
  float ambient_[3];
};


