/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <raytracing/lens_raytracer.h>
#include <shading/material_handler.h>
#include <shading/shader_factory.h>
#include <strings/join_filepath.h>

#include <iostream>

LensRaytracer::LensRaytracer()
    : mx0_(0.0),
      my0_(0.0),
      mx1_(0.0),
      my1_(0.0),
      mdx_(0.0),
      mdy_(0.0),
      vertical_angle_(0.0),
      horizontal_angle_(0.0),
      window_width_(0),
      window_height_(0),
      draw_axes_(false),
      update_mouse_(true) {}

LensRaytracer::~LensRaytracer() {
  // Delete shaders.
  delete ShaderFactory::Instance();
}

bool LensRaytracer::Initialize(int window_width, int window_height) {
  // Set window width and height.
  window_width_ = window_width;
  window_height_ = window_height;

  // Load shaders.
  const std::string vertex_shader =
      strings::JoinFilepath(LR3D_SHADER_DIR, "vertex.vs");
  const std::string fragment_shader =
      strings::JoinFilepath(LR3D_SHADER_DIR, "fragment.fs");

  if (!ShaderFactory::Instance()->Initialize("Lens Raytracer", vertex_shader,
                                             fragment_shader)) {
    std::cerr << "Failed to initialize shaders." << std::endl;
    return false;
  }
  LinkShader();

  // Initialize viewing vectors.
  eye_ = glm::vec3(0.0, 0.0, -5.0);
  lookat_ = glm::vec3(0.0, 0.0, 0.0);
  up_ = glm::vec3(0.0, 1.0, 0.0);

  // Load the scene.
  const std::string scene_file =
      strings::JoinFilepath(LR3D_CONFIG_DIR, LR3D_SCENE_FILE);
  scene_.LoadFromFile(scene_file);
  scene_.MakeBufferObjects();
  scene_.ComputePaths();

  return true;
}

void LensRaytracer::Update(GLFWwindow* window, double elapsed,
                           double total_elapsed) {
  const glm::vec3 forward = glm::normalize(lookat_ - eye_);
  const glm::vec3 right = glm::normalize(glm::cross(forward, up_));

  if (glfwGetKey(window, 'W')) {
    eye_ += forward * static_cast<float>(total_elapsed);
  }
  if (glfwGetKey(window, 'S')) {
    eye_ -= forward * static_cast<float>(total_elapsed);
  }
  if (glfwGetKey(window, 'D')) {
    eye_ += right * static_cast<float>(total_elapsed);
  }
  if (glfwGetKey(window, 'A')) {
    eye_ -= right * static_cast<float>(total_elapsed);
  }
  if (glfwGetKey(window, 'Q')) {
    eye_ += up_ * static_cast<float>(total_elapsed);
  }
  if (glfwGetKey(window, 'E')) {
    eye_ -= up_ * static_cast<float>(total_elapsed);
  }
  if (glfwGetKey(window, 'F')) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  }
  if (glfwGetKey(window, 'G')) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  if (glfwGetKey(window, 'V')) {
    draw_axes_ = true;
  }
  if (glfwGetKey(window, 'B')) {
    draw_axes_ = false;
  }
  if (glfwGetKey(window, 'N')) {
    update_mouse_ = false;
  }
  if (glfwGetKey(window, 'M')) {
    update_mouse_ = true;
  }

  // Check for mouse input.
  glfwGetCursorPos(window, &mx1_, &my1_);

  if (update_mouse_) {
    // Handle the very first update call.
    if (mx0_ == 0 && my0_ == 0) {
      mx0_ = mx1_;
      my0_ = my1_;
    }

    // Compute delta mouse input.
    mdx_ = mx1_ - mx0_;
    mdy_ = my1_ - my0_;

    // Set old mouse positions.
    mx0_ = mx1_;
    my0_ = my1_;

    // Apply rotations.
    vertical_angle_ -= mdy_ / 400.0;
    horizontal_angle_ -= mdx_ / 400.0;

    // Limit user to looking up or down 85 degrees in either direction.
    if (vertical_angle_ > max_angle_) {
      vertical_angle_ = max_angle_;
    } else if (vertical_angle_ < -max_angle_) {
      vertical_angle_ = -max_angle_;
    }
  }

  lookat_ = glm::vec3(eye_.x + cos(vertical_angle_) * sin(horizontal_angle_),
                      eye_.y + sin(vertical_angle_),
                      eye_.z + cos(vertical_angle_) * cos(horizontal_angle_));
}

void LensRaytracer::Render(double elapsed, double total_elapsed) {

  // Set up model, view, and projection matrices.
  ResetModel();
  ModelTransform();
  ProjectionTransform();
  ViewTransform();

  // Set camera position.
  glUniform3f(camera_pos_, eye_.x, eye_.y, eye_.z);

  // Render the scene.
  scene_.Render(draw_axes_, eye_);
}

void LensRaytracer::LinkShader() {
  // Get vertex and fragment shader.
  static Shader* shader =
      ShaderFactory::Instance()->GetShader("Lens Raytracer");

  // Link matrices.
  model_matrix_ = glGetUniformLocation(shader->GetProgram(), "u_model_matrix");
  view_matrix_ = glGetUniformLocation(shader->GetProgram(), "u_view_matrix");
  proj_matrix_ = glGetUniformLocation(shader->GetProgram(), "u_proj_matrix");

  // Link camera position and look direction.
  camera_pos_ = glGetUniformLocation(shader->GetProgram(), "u_camera_position");

  // Link shader materials with material handler object on CPU side.
  MaterialHandler::Instance()->LinkWithShader(shader);

  // Start using the shader.
  shader->Use();
}

void LensRaytracer::ResetModel() {
  // Get handle to model matrix and set to identity.
  glUniformMatrix4fv(model_matrix_, 1, GL_FALSE,
                     glm::value_ptr(glm::mat4(1.f)));
}

void LensRaytracer::ModelTransform() {}

void LensRaytracer::ViewTransform() {
  // Set the view matrix based on the eye, lookat, and up vectors.
  const glm::mat4 view = glm::lookAt(eye_, lookat_, up_);
  glUniformMatrix4fv(view_matrix_, 1, GL_FALSE, glm::value_ptr(view));
}

void LensRaytracer::ProjectionTransform() {
  // Create a projection matrix using the window aspect ratio and a desired
  // horizontal fov.
  const float aspect_ratio =
      static_cast<float>(window_width_) / static_cast<float>(window_height_);
  const float hfov = 120.f;
  const float near_plane = 0.1f;
  const float far_plane = 400.f;
  const glm::mat4 proj =
      glm::perspective(hfov, aspect_ratio, near_plane, far_plane);
  glUniformMatrix4fv(proj_matrix_, 1, GL_FALSE, glm::value_ptr(proj));
}
