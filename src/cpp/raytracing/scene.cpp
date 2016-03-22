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

#include <raytracing/scene.h>
#include <strings/has_substring.h>
#include <strings/tokenize.h>

#include <iostream>

Scene::Scene() {}
Scene::~Scene() {}

bool Scene::LoadFromFile(const std::string& filename) {

  // Open the file.
  std::ifstream file;
  file.open(filename.c_str(), std::ifstream::in);
  if (!file.is_open()) {
    std::cout << "Could not open configuration file: " << filename << "."
              << std::endl;
    return false;
  }

  // Read lines from the file.
  std::string object_line;
  while (std::getline(file, object_line)) {

    // Create a lens.
    if (strings::HasSubstring(object_line, "lens")) {

      Lens lens;
      std::string parameter;
      std::vector<std::string> tokens;

      // Load lens parameters.
      if (!std::getline(file, parameter)) break;
      if (!strings::HasSubstring(parameter, "parameters")) break;
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetRadius1(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetRadius2(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetThickness(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetWidth(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetIndexOfRefraction(std::stod(tokens[1], NULL));

      if (!std::getline(file, parameter)) break;
      if (!strings::HasSubstring(parameter, "position")) break;
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetX(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetY(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetZ(std::stod(tokens[1], NULL));

      if (!std::getline(file, parameter)) break;
      if (!strings::HasSubstring(parameter, "orientation")) break;
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetRoll(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetPitch(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      lens.SetYaw(std::stod(tokens[1], NULL));

      lens.Print("Loaded lens with parameters:");
      lenses_.push_back(lens);
    }

    // Create a ray.
    if (strings::HasSubstring(object_line, "ray")) {

      Ray ray;
      std::string parameter;
      std::vector<std::string> tokens;

      // Load ray parameters.
      if (!std::getline(file, parameter)) break;
      if (!strings::HasSubstring(parameter, "origin")) break;
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      ray.SetOriginX(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      ray.SetOriginY(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      ray.SetOriginZ(std::stod(tokens[1], NULL));

      if (!std::getline(file, parameter)) break;
      if (!strings::HasSubstring(parameter, "direction")) break;
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      ray.SetDirectionX(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      ray.SetDirectionY(std::stod(tokens[1], NULL));
      if (!std::getline(file, parameter)) break;
      strings::Tokenize(parameter, ' ', &tokens);
      ray.SetDirectionZ(std::stod(tokens[1], NULL));

      ray.NormalizeDirection();
      ray.Print("Loaded ray with parameters:");
      rays_.push_back(ray);
    }
  }

  // Loop back through and create buffer objects for all new lenses.
  for (size_t ii = 0; ii < lenses_.size(); ++ii)
    lenses_[ii].Initialize();

  return true;
}

void Scene::Render(bool draw_axes) {
  if (draw_axes) {
    // Draw some axes.
    RenderAxes();
  }

  // Render lenses.
  for (size_t ii = 0; ii < lenses_.size(); ++ii) {
    lenses_[ii].Render();
  }
}

void Scene::RenderAxes() {
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(1.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 1.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 1.0);
  glEnd();
}

void Scene::MakeBufferObjects() {
  for (unsigned ii = 0; ii < lenses_.size(); ++ii) {
    lenses_[ii].MakeBufferObjects();
  }
}

void Scene::AddLens(const Lens& lens) {
  lenses_.push_back(lens);
}

void Scene::AddRay(const Ray& ray) {
  rays_.push_back(ray);
}

bool Scene::GetLens(unsigned int index, Lens* lens) const {
  if (index < lenses_.size()) {
    *lens = lenses_[index];
    return true;
  }

  return false;
}

bool Scene::GetRay(unsigned int index, Ray* ray) const {
  if (index < rays_.size()) {
    *ray = rays_[index];
    return true;
  }

  return false;
}
