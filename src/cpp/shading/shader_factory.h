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
// Factory class for creating shaders.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SHADING_SHADER_FACTORY_H
#define SHADING_SHADER_FACTORY_H

#include <shading/shader.h>
#include <utils/disallow_copy_and_assign.h>

#include <OpenGL/gl.h>
#include <map>

class ShaderFactory {
 public:
  // Singleton.
  static ShaderFactory* Instance();
  ~ShaderFactory();

  bool Initialize(const std::string& key, const std::string& vertex_shader_file,
                  const std::string& fragment_shader_file);
  Shader* GetShader(const std::string& key);

 private:
  // Private constructor.
  ShaderFactory();

  bool LoadShaders(const std::string& key,
                   const std::string& vertex_shader_file,
                   const std::string& fragment_shader_file);
  char* LoadFile(const std::string& filename);
  bool CheckShaderSuccess(GLuint shader);
  bool CheckProgramSuccess(GLuint program);

  static ShaderFactory* instance_;
  std::map<std::string, GLuint> shader_map_;
};

#endif
