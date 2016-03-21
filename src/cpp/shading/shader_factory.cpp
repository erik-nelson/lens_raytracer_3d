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

#include <shading/shader_factory.h>

#include <cstdio>
#include <iostream>

ShaderFactory* ShaderFactory::instance_ = NULL;

ShaderFactory::ShaderFactory() {
  shader_map_ = std::map<std::string, GLuint>();
}

ShaderFactory::~ShaderFactory() {
  std::map<std::string, GLuint>::iterator iter = shader_map_.begin();
  for (; iter != shader_map_.end(); ++iter)
    glDeleteProgram(iter->second);
}

ShaderFactory* ShaderFactory::Instance() {
  if (instance_ == NULL)
    instance_ = new ShaderFactory();

  return instance_;
}

bool ShaderFactory::Initialize(const std::string& key,
                               const std::string& vertex_shader_file,
                               const std::string& fragment_shader_file) {
  return LoadShaders(key, vertex_shader_file, fragment_shader_file);
}

Shader* ShaderFactory::GetShader(const std::string& key) {
  return new Shader(shader_map_[key]);
}

bool ShaderFactory::LoadShaders(const std::string& key,
                               const std::string& vertex_shader_file,
                               const std::string& fragment_shader_file) {
  GLuint program, vsp, fsp;
  char* buffer;

  program = glCreateProgram();

  if (vertex_shader_file.length() > 0) {
    // Prepare buffers.
    vsp = glCreateShader(GL_VERTEX_SHADER);
    buffer = LoadFile(vertex_shader_file);
    if (buffer == NULL) {
      std::cerr << "Failed to load vertex shader: " << vertex_shader_file << "."
                << std::endl;
      return false;
    }

    // Load and compile shader.
    glShaderSource(vsp, 1, (const GLchar**)&buffer, 0);
    glCompileShader(vsp);
    if (!CheckShaderSuccess(vsp))
      return false;

    // Attach shader to program.
    glAttachShader(program, vsp);
    glDeleteShader(vsp);
  }

  if (fragment_shader_file.length() > 0) {
    // Prepare buffers.
    fsp = glCreateShader(GL_FRAGMENT_SHADER);
    buffer = LoadFile(fragment_shader_file);
    if (buffer == NULL) {
      std::cerr << "Failed to load fragment shader: " << fragment_shader_file << "."
                << std::endl;
      return false;
    }

    // Load and compile shader.
    glShaderSource(fsp, 1, (const GLchar**)&buffer, 0);
    glCompileShader(fsp);
    if (!CheckShaderSuccess(fsp))
      return false;

    // Attach shader to program.
    glAttachShader(program, fsp);
    glDeleteShader(fsp);
  }

  // Bind position and normal attributes to the shaders.
  glBindAttribLocation(program, 0, "a_position");
  glBindAttribLocation(program, 1, "a_normal");

  // Link the shader and main program.
  glLinkProgram(program);
  if (!CheckProgramSuccess(program))
    return false;

  // Make note of where this shader is.
  shader_map_[key] = program;

  delete buffer;
  return true;
}

char* ShaderFactory::LoadFile(const std::string& filename) {
  char* data;
  FILE* input;
  fpos_t size;

  // Open the file for reading.
  input = fopen(filename.c_str(), "r");
  if (input == NULL)
    return NULL;

  // Determine file size.
  fseek(input, 0, SEEK_END);
  fgetpos(input, &size);
  fseek(input, 0, SEEK_SET);

  // Allocate and populate array.
  data = static_cast<char*>(malloc(size+1));
  fread(data, sizeof(char), size, input);

  // Clean up and finish.
  fclose(input);
  data[size] = '\0';
  return data;
}

bool ShaderFactory::CheckShaderSuccess(GLuint shader) {
  GLint status, length;
  char* error;

  // Check for success.
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
  if (status)
    return true;

  // If failure, retrieve error log.
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
  error = static_cast<char*>(malloc(length));
  glGetShaderInfoLog(shader, length, &length, error);

  // Output error.
  std::cerr << "Failed to compile shader: " << error;

  // Clean up and exit.
  delete error;
  return false;
}

bool ShaderFactory::CheckProgramSuccess(GLuint program) {
  GLint status, length;
  char* error;

  // Check for success.
  glGetProgramiv(program, GL_LINK_STATUS, &status);
  if (status)
    return true;

  // If failure, retrieve error log.
  glGetShaderiv(program, GL_INFO_LOG_LENGTH, &length);
  error = static_cast<char*>(malloc(length));
  glGetShaderInfoLog(program, length, &length, error);

  // Output error.
  std::cerr << "Failed to link shader: " << error << "." << std::endl;

  // Clean up and exit.
  delete error;
  return false;
}
