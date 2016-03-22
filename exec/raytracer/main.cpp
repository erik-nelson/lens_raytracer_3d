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

///////////////////////////////////////////////////////////////////////////////
//
// This file uses OpenGL to raytrace a 3D scene for the purpose of lens design.
//
///////////////////////////////////////////////////////////////////////////////

#include <raytracing/lens_raytracer.h>

#include <iostream>
#include <memory>

#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>

void ErrorCallback(int error, const char* description) {
  std::cerr << description << std::endl;
}

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }
}

int main(int argc, char** argv) {

  // Initialize GLFW.
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW." << std::endl;
    return EXIT_FAILURE;
  }

  // Create a window.
  const int window_size_x = 1920;
  const int window_size_y = 1080;
  GLFWwindow* window = glfwCreateWindow(
      window_size_x, window_size_y, "Lens Raytracer", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create OpenGL context." << std::endl;
    glfwTerminate();
    return EXIT_FAILURE;
  }

  glfwMakeContextCurrent(window);
  glfwSetKeyCallback(window, KeyCallback);

  // Configure OpenGL.
  glClearColor(1.f, 1.f, 1.f, 0.f);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glEnable(GL_ALPHA_TEST);
  glEnable(GL_LINE_SMOOTH);
  glLineWidth(5.f);
  glClearDepth(1.f);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);

  // Create a raytracer object.
  LensRaytracer lr;
  if (!lr.Initialize(window_size_x, window_size_y)) {
    std::cerr << "Failed to initialize lens raytracer." << std::endl;
    glfwTerminate();
    return EXIT_FAILURE;
  }

  // Update until interrupted.
  static double then = glfwGetTime();
  while (!glfwWindowShouldClose(window)) {

    // Get incremental and total elapsed time.
    static double now = glfwGetTime();
    static double dt = now - then;

    // Update and render.
    lr.Update(window, dt, now);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    lr.Render(dt, now);

    // Swap buffers and poll for new keyboard and mouse events.
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();

  return EXIT_SUCCESS;
}

