#include "Transform.h"

void Transform::Update()
{
	Rotation = glm::normalize(Rotation);
}

