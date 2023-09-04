/* Start Header -----------------------------------------------------------------
File: BoundingUtils.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#include <iostream>
class Object;

class Component
{
public:
	enum ComponentType
	{
		TRANSFORM,
		PRIMITIVE,
		PHYSICS,
		CONTROLLER,
		SCRIPT,
		COLLIDER,
		LIGHT,
		TypeCount
	};
	// List of the component type names
	static const char* ComponentTypeName[ComponentType::TypeCount];
	Object* pOwner_;
	ComponentType componentType;
	char* name;

protected:	
	Component(ComponentType type) : componentType(type) {}
public:
	virtual ~Component() {}

	inline ComponentType GetComponentType() { return componentType; }
	inline Object* GetOwner() { return pOwner_; }
	inline void SetOwner(Object* aOwner) { pOwner_ = aOwner; }

	virtual void Initialize() {};
	virtual void Destroy() {};
	virtual void Update() {};
};
