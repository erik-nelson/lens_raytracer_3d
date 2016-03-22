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

#include <raytracing/ray.h>

#include <iostream>

Ray::Ray() {}

Ray::Ray(const glm::vec3& origin, const glm::vec3& direction)
    : origin_(origin), direction_(direction) {
  NormalizeDirection();
}

Ray::Ray(double ox, double oy, double oz, double dx, double dy, double dz) {
  origin_ = glm::vec3(ox, oy, oz);
  direction_ = glm::vec3(dx, dy, dz);
  NormalizeDirection();
}

Ray::~Ray() {}

const glm::vec3& Ray::GetOrigin() const {
  return origin_;
}

const glm::vec3& Ray::GetDirection() const {
  return direction_;
}

double Ray::GetOriginX() const {
  return origin_.x;
}

double Ray::GetOriginY() const {
  return origin_.y;
}

double Ray::GetOriginZ() const {
  return origin_.z;
}

double Ray::GetDirectionX() const {
  return direction_.x;
}

double Ray::GetDirectionY() const {
  return direction_.y;
}

double Ray::GetDirectionZ() const {
  return direction_.z;
}

void Ray::SetOrigin(const glm::vec3& origin) {
  origin_ = origin;
}

void Ray::SetDirection(const glm::vec3& direction) {
  direction_ = direction;
  NormalizeDirection();
}

void Ray::SetOriginX(double x) {
  origin_.x = x;
}

void Ray::SetOriginY(double y) {
  origin_.y = y;
}

void Ray::SetOriginZ(double z) {
  origin_.z = z;
}

void Ray::SetDirectionX(double x) {
  direction_.x = x;
}

void Ray::SetDirectionY(double y) {
  direction_.y = y;
}

void Ray::SetDirectionZ(double z) {
  direction_.z = z;
}

void Ray::NormalizeDirection() {
  direction_ = glm::normalize(direction_);
}

void Ray::Print(const std::string& prefix) {
  if (!prefix.empty())
    std::cout << prefix << std::endl;

  std::cout << "origin: ("
            << origin_.x << ", "
            << origin_.y << ", "
            << origin_.z << ")" << std::endl
            << "direction: ("
            << direction_.x << ", "
            << direction_.y << ", "
            << direction_.z << ")" << std::endl << std::endl;
}
