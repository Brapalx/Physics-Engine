/* Start Header -----------------------------------------------------------------
File: Transform.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

#include "glm\vec3.hpp"
#include "glm\vec4.hpp"
#include "glm\vec2.hpp"
#include <glm\gtc\quaternion.hpp>
#include <glm\mat3x3.hpp>
#include <glm\mat4x4.hpp>
#include "Component.h"

class Transform : public Component
{
	/*----------MEMBER VARIABLES----------*/
public:
	glm::vec3 Position;
	glm::quat Rotation;
	glm::vec3 Scale;

    glm::vec3 lightColor_;
	glm::mat4 m_;

	/*----------MEMBER FUNCTIONS----------*/
public:
	Transform() : Component(ComponentType::TRANSFORM),
		Position(glm::vec3(0)),
		Rotation(glm::quat()),
		Scale(glm::vec3(1))
	{}
	Transform(Transform const& CopyTransform) : Component(ComponentType::TRANSFORM)
	{
		Position = CopyTransform.Position;
		Rotation = CopyTransform.Rotation;
		Scale = CopyTransform.Scale;
	}
	virtual ~Transform() {};

	static inline Component::ComponentType GetComponentID() { return (ComponentType::TRANSFORM); }
	static inline const char* GetComponentName() { return ComponentTypeName[ComponentType::TRANSFORM]; }

	inline glm::vec3 GetPosition() { return Position; }
	inline glm::quat GetRotation() { return Rotation; }
	inline glm::vec3 GetScale() { return Scale; }

	inline void SetPosition(glm::vec3 newPosition) { Position = newPosition; }
	inline void SetRotation(glm::quat newRotation) { Rotation = newRotation; }
	inline void SetScale(glm::vec3 newScale) { Scale = newScale; }
	// Rotates this transform using the provided quaternion
	inline void Rotate(glm::quat aQuat) {
		Rotation = Rotation * aQuat;
	}

	inline glm::vec3 GetLightC() const { return lightColor_; }
	inline void SetLightC(const glm::vec3& c) { lightColor_ = c; }

	inline glm::mat4 GetModel()
	{
		glm::mat4 translate = glm::translate(Position);
		glm::mat4 rotate = glm::mat4_cast(Rotation);
		glm::mat4 scale = glm::scale(Scale);
		m_ = translate * rotate * scale;
		return m_;
	}

	void Update();

};
