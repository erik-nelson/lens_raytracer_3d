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

  double GetIncomingDistance() const;
  double GetInternalDistance() const;

  void SetIncomingRay(const Ray& incoming);
  void SetInternalRay(const Ray& internal);
  void SetOutgoingRay(const Ray& outgoing);
  void SetLens(const Lens& lens);

 private:
  // Check if a ray will intersect the top face of the lens first, or the bottom
  // face.
  bool RayIntersectsTopFaceFirst(const Ray& ray, const Lens& lens) const;

  // Check if the input ray intersects the lens face specified by the input
  // parameter 'top_face'. If an intersection is found, distance will be set as
  // the distance from the ray's origin to the collision.
  bool RayIntersectsLensFace(bool top_face,
                             const Ray& ray,
                             const Lens& lens,
                             double* distance) const;

  // Given an input ray and a lens, use Snell's law to compute an output
  // refracted ray.
  Ray GetRefractedRay(bool top_face,
                      bool coming_in,
                      double distance,
                      const Ray& in,
                      const Lens& lens);

  // Three rays model a collision.
  Ray incoming_;
  Ray internal_;
  Ray outgoing_;

  // Distance along incoming and internal rays before collision.
  double incoming_distance_;
  double internal_distance_;

  // Copy the lens that this collision refers to.
  Lens lens_;
};

#endif
