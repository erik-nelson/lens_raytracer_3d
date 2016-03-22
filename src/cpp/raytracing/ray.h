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
// This file defines a ray in 3D space, including an origin and a normalized
// direction.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H

#include <glm/glm.hpp>

#include <string>

class Ray {
 public:
  Ray();
  Ray(const glm::vec3& origin, const glm::vec3& direction);
  Ray(double ox, double oy, double oz, double dx, double dy, double dz);
  ~Ray();

  // Getters and setters.
  const glm::vec3& GetOrigin() const;
  const glm::vec3& GetDirection() const;
  double GetOriginX() const;
  double GetOriginY() const;
  double GetOriginZ() const;
  double GetDirectionX() const;
  double GetDirectionY() const;
  double GetDirectionZ() const;

  void SetOrigin(const glm::vec3& origin);
  void SetDirection(const glm::vec3& direction);
  void SetOriginX(double x);
  void SetOriginY(double y);
  void SetOriginZ(double z);
  void SetDirectionX(double x);
  void SetDirectionY(double y);
  void SetDirectionZ(double z);

  // Normalize direction vector.
  void NormalizeDirection();

  // Print ray parameters.
  void Print(const std::string& prefix = std::string());

 private:

  glm::vec3 origin_;
  glm::vec3 direction_;
};

#endif
