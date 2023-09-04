/* Start Header -----------------------------------------------------------------
File: Camera.h
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/
#pragma once
#define GLEW_STATIC
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  UP_A,
  DOWN_A,
  LEFT_A,
  RIGHT_A,
  SPACE
};

const GLfloat SPEED = 6.0f;
const float rotAngle = 0.0872665f; // 10 degrees

class Camera
{
  glm::mat4 persp_matrix_;
  glm::vec3 pos_;
  glm::vec3 initPos_;
  glm::vec3 forward_;
  glm::vec3 up_;
  glm::vec3 right_;

  public:
  Camera(const glm::vec3& pos, float fov, float aspect_ratio, float near, float far) : persp_matrix_(glm::perspective(fov, aspect_ratio, near, far)), pos_(pos), initPos_(pos),
  forward_(glm::vec3(0,0,1)), up_(glm::vec3(0,1,0)), right_(glm::normalize(glm::cross(forward_, up_))){}
  
  inline glm::mat4 GetViewProjection() const { return persp_matrix_ * glm::lookAt(pos_, pos_ + forward_, up_); }

  // moves camera based on input
  void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime)
  {
    GLfloat velocity = SPEED * deltaTime;

    switch (direction)
    {
    case FORWARD:
      pos_ += forward_ * velocity;
      break;
    case BACKWARD:
      pos_ -= forward_ * velocity;
      break;
    case LEFT:
      pos_ -= right_ * velocity;
      break;
    case RIGHT:
      pos_ += right_ * velocity;
      break;
    case UP_A:
      up_ = glm::vec3(glm::rotate(rotAngle, right_) * glm::vec4(up_, 0.0));
      forward_ = glm::vec3(glm::rotate(rotAngle, right_) * glm::vec4(forward_, 0.0));
      break;
    case DOWN_A:
      up_ = glm::vec3(glm::rotate(-rotAngle, right_) * glm::vec4(up_, 0.0));
      forward_ = glm::vec3(glm::rotate(-rotAngle, right_) * glm::vec4(forward_, 0.0));
      break;
    case LEFT_A:
      right_ = glm::vec3(glm::rotate(rotAngle, up_) * glm::vec4(right_, 0.0));
      forward_ = glm::vec3(glm::rotate(rotAngle, up_) * glm::vec4(forward_, 0.0));
      break;
    case RIGHT_A:
      right_ = glm::vec3(glm::rotate(-rotAngle, up_) * glm::vec4(right_, 0.0));
      forward_ = glm::vec3(glm::rotate(-rotAngle, up_) * glm::vec4(forward_, 0.0));
      break;
    case SPACE:
      pos_ = initPos_;
      forward_ = glm::vec3(0, 0, 1);
      up_ = glm::vec3(0, 1, 0);
      right_ = glm::normalize(glm::cross(forward_, up_));
      break;
    default:
      break;
    }
  }

  inline glm::vec3 GetPos() const {return pos_;}
  inline glm::vec3 GetForward() const { return forward_; };
};