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

#include <raytracing/path.h>

#include <iostream>

Path::Path() {}

Path::~Path() {
  // Free buffer objects.
  GLuint buffers[3] = {vertex_buffer_object_,
                       normal_buffer_object_,
                       index_buffer_object_};
  glDeleteBuffers(3, buffers);
}

void Path::Initialize() {
  // Initialize buffer objects.
  GLuint buffers[3];
  glGenBuffers(3, buffers);
  vertex_buffer_object_ = buffers[0];
  normal_buffer_object_ = buffers[1];
  index_buffer_object_ = buffers[2];
}

const std::vector<Ray>& Path::GetPath() const {
  return path_;
}

bool Path::GetRay(unsigned int index, Ray* ray) const {
  if (index < path_.size()) {
    *ray = path_[index];
    return true;
  }

  return false;
}

void Path::SetPath(const std::vector<Ray>& path) {
  path_ = path;
}

void Path::AddRay(const Ray& ray) {
  path_.push_back(ray);
}

void Path::MakeBufferObjects() {
  // Vertex, normal, and index containers.
  std::vector<GLfloat> vertices;
  std::vector<GLfloat> normals;
  std::vector<GLuint> indices;

  for (const auto& ray : path_) {
    vertices.push_back(ray.GetOriginX());
    vertices.push_back(ray.GetOriginY());
    vertices.push_back(ray.GetOriginZ());
    normals.push_back(ray.GetOriginX());
    normals.push_back(ray.GetOriginY());
    normals.push_back(ray.GetOriginZ());
  }

  // Project the last vertex along the final ray's direction.
  Ray last_ray = path_.back();
  vertices.push_back(last_ray.GetOriginX() + 5.f * last_ray.GetDirectionX());
  vertices.push_back(last_ray.GetOriginY() + 5.f * last_ray.GetDirectionY());
  vertices.push_back(last_ray.GetOriginZ() + 5.f * last_ray.GetDirectionZ());
  normals.push_back(last_ray.GetOriginX() + 5.f * last_ray.GetDirectionX());
  normals.push_back(last_ray.GetOriginY() + 5.f * last_ray.GetDirectionY());
  normals.push_back(last_ray.GetOriginZ() + 5.f * last_ray.GetDirectionZ());

  for (int ii = 0; ii < path_.size(); ++ii) {
    indices.push_back(ii);
    indices.push_back(ii + 1);
  }

  // Pack vertex data into a buffer object.
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_object_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
               &vertices.front(), GL_STATIC_DRAW);

  // Pack normal data into a buffer object.
  glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_object_);
  glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLfloat),
               &normals.front(), GL_STATIC_DRAW);

  // Pack index data into a buffer object.
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_object_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               &indices.front(), GL_STATIC_DRAW);
  index_size_ = indices.size() * sizeof(GLuint);
}

void Path::Render() const {
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_object_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_object_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_object_);
  glDrawElements(GL_LINES, index_size_, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}
