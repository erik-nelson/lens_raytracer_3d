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

#include <shading/material_handler.h>

#include <iostream>

MaterialHandler* MaterialHandler::instance_ = NULL;

MaterialHandler::MaterialHandler() : linked_(false) {}

MaterialHandler::~MaterialHandler() {}

MaterialHandler* MaterialHandler::Instance() {
  if (instance_ == NULL)
    instance_ = new MaterialHandler();

  return instance_;
}

void MaterialHandler::LinkWithShader(Shader* shader) {
  if (!shader) {
    std::cerr << "Shader pointer is null." << std::endl;
  }

  phong_shading_ = glGetUniformLocation(shader->GetProgram(), "u_phong_shading");
  ambient_mat_ = glGetUniformLocation(shader->GetProgram(), "u_ambient_mat");
  diffuse_mat_ = glGetUniformLocation(shader->GetProgram(), "u_diffuse_mat");
  specular_mat_ = glGetUniformLocation(shader->GetProgram(), "u_specular_mat");
  alpha_ = glGetUniformLocation(shader->GetProgram(), "u_alpha");

  linked_ = true;
}

bool MaterialHandler::CheckLinked() const {
  if (!linked_) {
    std::cerr << "Shader not linked to material handler." << std::endl;
    return false;
  }
  return true;
}

void MaterialHandler::SetPhongShading(bool on_or_off) {
  CheckLinked();
  glUniform1f(phong_shading_, on_or_off ? 1.0 : 0.0);
}

void MaterialHandler::SetAmbient(double r, double g, double b) {
  CheckLinked();
  glUniform3f(ambient_mat_, r, g, b);
}

void MaterialHandler::SetDiffuse(double r, double g, double b) {
  CheckLinked();
  glUniform3f(diffuse_mat_, r, g, b);
}

void MaterialHandler::SetSpecular(double r, double g, double b) {
  CheckLinked();
  glUniform3f(specular_mat_, r, g, b);
}

void MaterialHandler::SetAlpha(double alpha) {
  CheckLinked();
  glUniform1f(alpha_, alpha);
}
