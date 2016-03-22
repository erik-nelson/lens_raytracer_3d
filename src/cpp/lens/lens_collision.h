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
// Model a collision between a 3D ray and a lens. An incoming ray (assuming no
// total internal reflection) will have two collisions with the lens. Therefore
// a collision can be modeled as a set of three rays: one incoming ray, one
// internal ray, and one outgoing ray.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef LENS_LENS_COLLISION_H
#define LENS_LENS_COLLISION_H

#include <lens/lens.h>
#include <raytracing/ray.h>

class LensCollision {
 public:
  LensCollision();
  ~LensCollision();

  // Calculate collisions between a specified ray and a lens, storing collision
  // details internally. Return false if there is no collision between the ray
  // and lens.
  bool Compute(const Ray& incoming, const Lens& lens);

  // Getters and setters.
  const Ray& GetIncomingRay() const;
  const Ray& GetInternalRay() const;
  const Ray& GetOutgoingRay() const;
  const Lens& GetLens() const;

  void SetIncomingRay(const Ray& incoming);
  void SetInternalRay(const Ray& internal);
  void SetOutgoingRay(const Ray& outgoing);
  void SetLens(const Lens& lens);

 private:

  // Three rays model a collision.
  Ray incoming_;
  Ray internal_;
  Ray outgoing_;

  // Copy the lens that this collision refers to.
  Lens lens_;
};

#endif
