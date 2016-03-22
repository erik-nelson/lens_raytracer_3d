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

#include <lens/lens_collision.h>

#include <limits>

LensCollision::LensCollision()
    : incoming_distance_(std::numeric_limits<double>::infinity()),
      internal_distance_(std::numeric_limits<double>::infinity()) {}

LensCollision::~LensCollision() {}

bool LensCollision::Compute(const Ray& incoming, const Lens& lens) {

  // Check which face of the lens will be intersected first by the ray.
  bool top_face = RayIntersectsTopFaceFirst(incoming, lens);

  double distance = 0.0;
  if (RayIntersectsLensFace(top_face, incoming, lens, &distance)) {
    // Set the incoming ray.
    incoming_ = incoming;
    incoming_distance_ = distance;

    // Compute how the incoming ray refracts.
    internal_ =
        GetRefractedRay(top_face, true /*out to in*/, distance, incoming, lens);

    // Find where the internal ray collides with the second lens face.
    if (RayIntersectsLensFace(!top_face, internal_, lens, &distance)) {
      // Set the distance along the internal ray before collision.
      internal_distance_ = distance;

      // Compute how the internal ray refracts.
      outgoing_ = GetRefractedRay(!top_face, false /*in to out*/, distance,
                                  internal_, lens);
    } else {
      // We need both collisions with the lens to be successful.
      return false;
    }
  } else {
    // We need both collisions with the lens to be successful.
    return false;
  }

  return true;
}

const Ray& LensCollision::GetIncomingRay() const {
  return incoming_;
}

const Ray& LensCollision::GetInternalRay() const {
  return internal_;
}

const Ray& LensCollision::GetOutgoingRay() const {
  return outgoing_;
}

const Lens& LensCollision::GetLens() const {
  return lens_;
}

double LensCollision::GetIncomingDistance() const {
  return incoming_distance_;
}

double LensCollision::GetInternalDistance() const {
  return internal_distance_;
}

void LensCollision::SetIncomingRay(const Ray& incoming) {
  incoming_ = incoming;
}

void LensCollision::SetInternalRay(const Ray& internal) {
  internal_ = internal;
}

void LensCollision::SetOutgoingRay(const Ray& outgoing) {
  outgoing_ = outgoing;
}

void LensCollision::SetLens(const Lens& lens) {
  lens_ = lens;
}

bool LensCollision::RayIntersectsTopFaceFirst(const Ray& ray,
                                              const Lens& lens) const {
  // TODO: Fix this so that it takes into account lens angle!
  return ray.GetDirectionZ() < 0.0;
}

bool LensCollision::RayIntersectsLensFace(bool top_face,
                                          const Ray& ray,
                                          const Lens& lens,
                                          double* distance) const {
  // Sphere-Line intersection test from:
  // https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
  const double sign = top_face ? 1.0 : -1.0;
  const double radius = top_face ? lens.GetRadius1() : lens.GetRadius2();
  const glm::vec3 l = ray.GetDirection();
  const glm::vec3 o = ray.GetOrigin();
  const glm::vec3 c = lens.GetPosition() +
                      glm::vec3(0.0, 0.0, sign * 0.5 * lens.GetThickness()) -
                      glm::vec3(0.0, 0.0, sign * std::abs(radius));
  // TODO: c will need to be adjusted to take into account lens angle!

  const double discrim = std::pow(glm::dot(l, (o - c)), 2.0) -
                         std::pow(glm::length(o - c), 2.0) +
                         std::pow(radius, 2.0);
  if (discrim < 0.0)
    return false;

  *distance = -glm::dot(l, (o - c)) + sign * std::sqrt(discrim);
  if (*distance < 0.0)
    return false;

  glm::vec3 inter = ray.GetOrigin() + static_cast<float>(*distance) * ray.GetDirection();
  return true;
}

Ray LensCollision::GetRefractedRay(bool top_face,
                                   bool coming_in,
                                   double distance,
                                   const Ray& in,
                                   const Lens& lens) {
  const glm::vec3 collision_pt =
      in.GetOrigin() + static_cast<float>(distance) * in.GetDirection();

  // Compute the normal to the lens surface at the collision point.
  const double radius = top_face ? lens.GetRadius1() : lens.GetRadius2();
  const double sign = top_face ? 1.0 : -1.0;
  const glm::vec3 c = lens.GetPosition() +
                      glm::vec3(0.0, 0.0, sign * 0.5 * lens.GetThickness()) -
                      glm::vec3(0.0, 0.0, sign * std::abs(radius));
  const glm::vec3 n =
      static_cast<float>(sign) * glm::normalize(c - collision_pt);

  // Get angle between incoming ray and normal.
  const double incoming_angle = std::acos(glm::dot(n, -in.GetDirection()));

  // Snell's law rotates incoming light rays 'in-plane', which means we need the
  // normal between the incoming light ray and the surface normal to compute the
  // outgoing ray.
  const glm::vec3 axis = glm::normalize(glm::cross(-in.GetDirection(), n));

  // If the axis elements are nan, that means the input ray and normal face the
  // same direction (i.e. cross product is (0, 0, 0)). In this case, we can skip
  // everything since the output ray will face the same direction as the input
  // ray.
  if (std::isnan(axis.x) || std::isnan(axis.y) || std::isnan(axis.z)) {
    Ray out;
    out.SetDirection(in.GetDirection());
    out.SetOrigin(collision_pt);
    return out;
  }

  const double snell_coefficient = coming_in ? 1.0 / lens.GetIndexOfRefraction()
                                             : lens.GetIndexOfRefraction();
  const double outgoing_angle =
      std::asin(std::sin(incoming_angle) * snell_coefficient);

  // Now use Rodrigues' formula to rotate the reflected normal to the desired
  // outgoing ray:
  // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
  const glm::mat3 u_cross(0.0, -axis.z, axis.y,
                          axis.z, 0.0, -axis.x,
                          -axis.y, axis.x, 0.0);
  const glm::mat3 u_cross2(axis.x * axis.x, axis.x * axis.y, axis.x * axis.z,
                           axis.y * axis.x, axis.y * axis.y, axis.y * axis.z,
                           axis.z * axis.x, axis.z * axis.y, axis.z * axis.z);

  const float ca = static_cast<float>(std::cos(outgoing_angle));
  const float sa = static_cast<float>(std::sin(outgoing_angle));
  const glm::mat3 rot =
      ca * glm::mat3(1.f) + sa * u_cross + (1.f - ca) * u_cross2;

  // Set the outgoing ray's direction and origin.
  Ray out;
  out.SetDirection(rot * -n);
  out.SetOrigin(collision_pt);

  return out;
}
