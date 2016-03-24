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
// This class gives access to static functions for globally changing material
// properties. These functions can be called from any object immediately prior
// to rendering to change the OpenGL material properties handed off to the
// shader on the GPU.
//
// ////////////////////////////////////////////////////////////////////////////

#ifndef SHADING_MATERIAL_HANDLER_H
#define SHADING_MATERIAL_HANDLER_H

#include <shading/shader_factory.h>
#include <utils/disallow_copy_and_assign.h>

class MaterialHandler {
 public:
  // Singleton
  static MaterialHandler* Instance();
  ~MaterialHandler();

  void LinkWithShader(Shader* shader);
  bool CheckLinked() const;

  void SetPhongShading(bool on_or_off);
  void SetAmbient(double r, double g, double b);
  void SetDiffuse(double r, double g, double b);
  void SetSpecular(double r, double g, double b);
  void SetAlpha(double alpha);

 private:
  // Ensure that MaterialHandler is a singleton.
  DISALLOW_COPY_AND_ASSIGN(MaterialHandler)
  MaterialHandler();

  // Boolean flag reflecting whether the material handler has been linked with a
  // shader.
  bool linked_;

  // Shader handles.
  GLuint phong_shading_;
  GLuint ambient_mat_;
  GLuint diffuse_mat_;
  GLuint specular_mat_;
  GLuint alpha_;

  static MaterialHandler* instance_;
};

#endif
