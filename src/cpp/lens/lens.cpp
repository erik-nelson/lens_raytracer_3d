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

#include <lens/lens.h>

#include <iostream>

Lens::Lens() : r1_(0.0), r2_(0.0), t_(0.0), w_(0.0), n_(0.0), index_size_(0) {
  // Initialize position and orientation to zero.
  position_ = glm::vec3(0.f);
  orientation_ = glm::vec3(0.f);

  // Initialize buffer objects.
  GLuint buffers[3];
  glGenBuffers(3, buffers);
  vertex_buffer_object_ = buffers[0];
  normal_buffer_object_ = buffers[1];
  index_buffer_object_ = buffers[2];
}

Lens::~Lens() {
  // Free buffer objects.
  GLuint buffers[3] = {vertex_buffer_object_,
                       normal_buffer_object_,
                       index_buffer_object_};
  glDeleteBuffers(3, buffers);
}

double Lens::GetRadius1() const {
  return r1_;
}

double Lens::GetRadius2() const {
  return r2_;
}

double Lens::GetThickness() const {
  return t_;
}

double Lens::GetWidth() const {
  return w_;
}

double Lens::GetX() const {
  return position_.x;
}

double Lens::GetY() const {
  return position_.y;
}

double Lens::GetZ() const {
  return position_.z;
}

double Lens::GetRoll() const {
  return orientation_.x;
}

double Lens::GetPitch() const {
  return orientation_.y;
}

double Lens::GetYaw() const {
  return orientation_.z;
}

double Lens::GetIndexOfRefraction() const {
  return n_;
}

void Lens::SetRadius1(double r1) {
  r1_ = r1;
}

void Lens::SetRadius2(double r2) {
  r2_ = r2;
}

void Lens::SetThickness(double t) {
  t_ = t;
}

void Lens::SetWidth(double w) {
  w_ = w;
}

void Lens::SetIndexOfRefraction(double n) {
  n_ = n;
}

void Lens::SetX(double x) {
  position_.x = x;
}

void Lens::SetY(double y) {
  position_.y = y;
}

void Lens::SetZ(double z) {
  position_.z = z;
}

void Lens::SetRoll(double roll) {
  orientation_.x = roll;
}

void Lens::SetPitch(double pitch) {
  orientation_.y = pitch;
}

void Lens::SetYaw(double yaw) {
  orientation_.z = yaw;
}

void Lens::MakeBufferObjects() {
  // Vertex, normal, and index containers.
  std::vector<GLfloat> vertices;
  std::vector<GLfloat> normals;
  std::vector<GLuint> indices;

  // The lens surface is made of rings of varying vertical angle.
  // We will put all of the vertex and normal data into 'vertices' and
  // 'normals', but will also store indices in 'lens1_rings' and 'lens2_rings'
  // to determine how vertices are connected into faces that we can render.
  typedef std::vector<GLuint> Ring;
  std::vector<Ring> lens1_rings;
  std::vector<Ring> lens2_rings;

  // Figure out how many rings we need for the top surface.
  double alpha = 0.5 * M_PI - std::acos(0.5 * w_ / std::abs(r1_));
  const unsigned int num_rings1 = std::floor(alpha / vertical_increment_);

  // Store vertical angles to iterate over.
  std::vector<double> v_angles, h_angles;
  for (unsigned int ii = 0; ii < num_rings1; ++ii)
    v_angles.push_back(vertical_increment_ * static_cast<double>(ii));
  v_angles.push_back(alpha);

  for (unsigned int ii = 0; ii < 2.0 * M_PI / horizontal_increment_; ++ii)
    h_angles.push_back(horizontal_increment_ * static_cast<double>(ii));

  // The first vertex (right at the top of the lens) can be handled separately.
  vertices.push_back(0.0);
  vertices.push_back(0.0);
  vertices.push_back(r1_);
  Vertex v0 = SphericalToCartesian(r1_, 0.0, 0.0);
  normals.push_back(v0.x);
  normals.push_back(v0.y);
  normals.push_back(v0.z);

  // Iterate around the surface of the first lens vertically from the top out.
  int index_counter = 1;
  for (size_t ii = 1; ii < v_angles.size(); ++ii) {
    double va = v_angles[ii];

    // Iterate around the surface horizontally, storing vertices in a ring.
    Ring ring;
    for (size_t jj = 0; jj < h_angles.size(); ++jj) {
      double ha = h_angles[jj];

      Vertex v = SphericalToCartesian(r1_, va, ha);
      ring.push_back(index_counter++);

      // Store the vertex and normal.
      vertices.push_back(v.x);
      vertices.push_back(v.y);
      vertices.push_back(v.z);

      const double norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      normals.push_back(v.x / norm);
      normals.push_back(v.y / norm);
      normals.push_back(v.z / norm);
    }
    lens1_rings.push_back(ring);
  }

  // Now figure out how to connect lens vertices into triangles.
  // The very top vertices on the lens can be handled separately since they are
  // connected by a single vertex.
  for (size_t ii = 0; ii < lens1_rings[0].size() - 1; ++ii) {
    indices.push_back(0);
    indices.push_back(lens1_rings[0][ii]);
    indices.push_back(lens1_rings[0][ii+1]);
  }
  indices.push_back(0);
  indices.push_back(lens1_rings[0].back());
  indices.push_back(lens1_rings[0].front());

  // Connect vertices on the lens that are not a part of the very top circle.
  for (size_t ii = 0; ii < lens1_rings.size() - 1; ++ii) {
    Ring inner_ring = lens1_rings[ii];
    Ring outer_ring = lens1_rings[ii + 1];

    for (size_t jj = 0; jj < inner_ring.size() - 1; ++jj) {
      // Each set of 4 vertices (inner, outer, inner+1, outer+1) gives 2 faces.
      indices.push_back(inner_ring[jj]);
      indices.push_back(outer_ring[jj]);
      indices.push_back(outer_ring[jj + 1]);

      indices.push_back(inner_ring[jj]);
      indices.push_back(outer_ring[jj + 1]);
      indices.push_back(inner_ring[jj + 1]);
    }

    // Handle the wrap around vertices.
    indices.push_back(inner_ring.back());
    indices.push_back(outer_ring.back());
    indices.push_back(outer_ring.front());

    indices.push_back(inner_ring.back());
    indices.push_back(outer_ring.front());
    indices.push_back(inner_ring.front());
  }





  alpha = 0.5 * M_PI - std::acos(0.5 * w_ / std::abs(r2_));
  const unsigned int num_rings2 = std::floor(alpha / vertical_increment_);

  v_angles.clear();
  for (unsigned int ii = 0; ii < num_rings2; ++ii)
    v_angles.push_back(vertical_increment_ * static_cast<double>(ii));
  v_angles.push_back(alpha);

  // Now repeat for the second lens surface.
  vertices.push_back(0.0);
  vertices.push_back(0.0);
  vertices.push_back(-r2_);
  v0 = SphericalToCartesian(-r2_, 0.0, 0.0);
  normals.push_back(v0.x);
  normals.push_back(v0.y);
  normals.push_back(v0.z);

  // Iterate around the surface of the first lens vertically from the top out.
  index_counter++;
  for (size_t ii = 1; ii < v_angles.size(); ++ii) {
    double va = v_angles[ii];

    // Iterate around the surface horizontally, storing vertices in a ring.
    Ring ring;
    for (size_t jj = 0; jj < h_angles.size(); ++jj) {
      double ha = h_angles[jj];

      Vertex v = SphericalToCartesian(-r2_, va, ha);
      ring.push_back(index_counter++);

      // Store the vertex and normal.
      vertices.push_back(v.x);
      vertices.push_back(v.y);
      vertices.push_back(v.z);

      const double norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      normals.push_back(v.x / norm);
      normals.push_back(v.y / norm);
      normals.push_back(v.z / norm);
    }
    lens2_rings.push_back(ring);
  }

  int last_lens1_index = lens1_rings.back().back();
  for (size_t ii = 0; ii < lens2_rings[0].size() - 1; ++ii) {
    indices.push_back(last_lens1_index + 1);
    indices.push_back(lens2_rings[0][ii]);
    indices.push_back(lens2_rings[0][ii + 1]);
  }
  indices.push_back(last_lens1_index + 1);
  indices.push_back(lens2_rings[0].back());
  indices.push_back(lens2_rings[0].front());

  // Connect vertices on the lens that are not a part of the very top circle.
  for (size_t ii = 0; ii < lens2_rings.size() - 1; ++ii) {
    Ring inner_ring = lens2_rings[ii];
    Ring outer_ring = lens2_rings[ii + 1];

    for (size_t jj = 0; jj < inner_ring.size() - 1; ++jj) {
      // Each set of 4 vertices (inner, outer, inner+1, outer+1) gives 2 faces.
      indices.push_back(inner_ring[jj]);
      indices.push_back(outer_ring[jj]);
      indices.push_back(outer_ring[jj + 1]);

      indices.push_back(inner_ring[jj]);
      indices.push_back(outer_ring[jj + 1]);
      indices.push_back(inner_ring[jj + 1]);
    }

    // Handle the wrap around vertices.
    indices.push_back(inner_ring.back());
    indices.push_back(outer_ring.back());
    indices.push_back(outer_ring.front());

    indices.push_back(inner_ring.back());
    indices.push_back(outer_ring.front());
    indices.push_back(inner_ring.front());
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

#if 0
  // Iterate around the surface of the second lens vertically from the top out.
  for (size_t ii = 0; ii < v_angles.size(); ++ii) {
    double va = v_angles[ii];

    // Iterate around the surface horizontally, storing vertices in a ring.
    Ring ring;
    for (size_t jj = 0; jj < h_angles.size(); ++jj) {
      double ha = h_angles[jj];

      Vertex v = SphericalToCartesian(r2_, va, ha);
      ring.push_back(v);

      // Store the vertex and normal.
      vertices.push_back(v.x);
      vertices.push_back(v.y);
      vertices.push_back(v.z);

      const double norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      normals.push_back(v.x / norm);
      normals.push_back(v.y / norm);
      normals.push_back(v.z / norm);
    }
    lens2_rings.push_back(ring);
  }
#endif

}

void Lens::Render() {
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_object_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_object_);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_object_);
  glDrawElements(GL_TRIANGLES, index_size_, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}

void Lens::Print(const std::string& prefix) {
  if (!prefix.empty())
    std::cout << prefix << std::endl;

  std::cout << "r1: " << r1_ << std::endl
            << "r2: " << r2_ << std::endl
            << "t: " << t_ << std::endl
            << "w: " << w_ << std::endl
            << "n: " << n_ << std::endl
            << "position: ("
            << position_.x << ", "
            << position_.y << ", "
            << position_.z << ")" << std::endl
            << "orientation: ("
            << orientation_.x << ", "
            << orientation_.y << ", "
            << orientation_.z << ")" << std::endl << std::endl;
}

Lens::Vertex Lens::SphericalToCartesian(double radius, double vertical_angle,
                                        double horizontal_angle) {
  Vertex vertex;
  vertex.x = radius * sin(vertical_angle) * cos(horizontal_angle);
  vertex.y = radius * sin(vertical_angle) * sin(horizontal_angle);
  vertex.z = radius * cos(vertical_angle);
  return vertex;
}
