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
// This class defines a scene containing lenses, objects, and light sources.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef RAYTRACING_SCENE_H
#define RAYTRACING_SCENE_H

#include <lens/lens.h>
#include <raytracing/path.h>
#include <raytracing/ray.h>

#include <fstream>
#include <string>
#include <vector>

class Scene {
 public:
  Scene();
  ~Scene();

  // Load lenses, lights, and objects from a .yaml file.
  bool LoadFromFile(const std::string& filename);

  // Draw lenses and ray paths, if ComputePaths() has been called. Camera
  // position must be passed in to sort lens depths for z buffering.
  void Render(bool draw_axes, const glm::vec3& camera_position_);

  // Raytrace all 'rays_' through the scene and find intersections with
  // 'lenses_'. Store ray paths in the local variable 'paths_'. This method will
  // clear 'paths_' and recompute when called.
  void ComputePaths();

  // Create buffer objects for all loaded lenses, objects, etc.
  void MakeBufferObjects();

  // Getters and setters.
  void AddLens(const Lens& lens);
  void AddRay(const Ray& ray);

  bool GetLens(unsigned int index, Lens* lens) const;
  bool GetRay(unsigned int index, Ray* ray) const;

 private:

  // Draw (x, y, z) axes;
  void RenderAxes();

  // Lenses and rays are loaded into the scene or added manually.
  std::vector<Lens> lenses_;
  std::vector<Ray> rays_;

  // Paths are computed by tracing 'rays_' through all 'lenses_' in the scene
  // upon a call to ComputePaths(). They are stored in the 'paths_' variable
  // until ComputePaths() is called again. Paths are rendered on calls to
  // 'Render()'.
  std::vector<Path> paths_;
};

#endif
