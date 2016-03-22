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
// The Path class defines a set of connected rays that have been traced through
// a scene. A Path is defined as an ordered set of Ray objects. Note that the
// Ray object's direction is irrelevant (except for the final ray), and that the
// path will be formed by connecting ray origins.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef RAYTRACING_PATH_H
#define RAYTRACING_PATH_H

#include <raytracing/ray.h>

#include <vector>

class Path {
 public:
  Path();
  ~Path();

  // Allocate buffer objects.
  void Initialize();

  // Getters and setters.
  const std::vector<Ray>& GetPath() const;
  bool GetRay(unsigned int index, Ray* ray) const;
  void SetPath(const std::vector<Ray>& path);

  void AddRay(const Ray& ray);

  // After setting parameters, create vertex, normal, and index buffer objects.
  void MakeBufferObjects();

  // Draw this path.
  void Render() const;

 private:

  std::vector<Ray> path_;

  // Buffer objects.
  GLuint vertex_buffer_object_;
  GLuint normal_buffer_object_;
  GLuint index_buffer_object_;
  int index_size_;
};

#endif
