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

#ifndef RAYTRACING_LENS_RAYTRACER_H
#define RAYTRACING_LENS_RAYTRACER_H

#include <cmath>

#include <raytracing/scene.h>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class LensRaytracer {
 public:
  LensRaytracer();
  ~LensRaytracer();

  bool Initialize(int window_width, int window_height);
  void Update(GLFWwindow* window, double elapsed, double total_elapsed);
  void Render(double elapsed, double total_elapsed);

 private:

  // Rendering functions.
  void LinkShader();
  void ResetModel();
  void ModelTransform();
  void ViewTransform();
  void ProjectionTransform();
  void SetMaterial();

  // Variables for tracking mouse position.
  double mx0_, my0_, mx1_, my1_, mdx_, mdy_;

  // Camera variables.
  double vertical_angle_;
  double horizontal_angle_;

  // Transform variables.
  glm::vec3 eye_, lookat_, up_;

  // Window width and height.
  int window_width_;
  int window_height_;

  // Shader handles.
  GLuint model_matrix_;
  GLuint view_matrix_;
  GLuint proj_matrix_;
  GLuint ambient_mat_;
  GLuint diffuse_mat_;
  GLuint specular_mat_;
  GLuint camera_pos_;

  // Rendering options.
  bool draw_axes_;

  // A scene that we will add lenses to.
  Scene scene_;

  // Maximum viewing angle so that a user can't look straight up or down.
  static constexpr double max_angle_ = 85.0 * M_PI / 180.0;
};

#endif
