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
}

Lens::~Lens() {
  // Free buffer objects.
  GLuint buffers[3] = {vertex_buffer_object_,
                       normal_buffer_object_,
                       index_buffer_object_};
  glDeleteBuffers(3, buffers);
}

void Lens::Initialize() {
  // Initialize buffer objects.
  GLuint buffers[3];
  glGenBuffers(3, buffers);
  vertex_buffer_object_ = buffers[0];
  normal_buffer_object_ = buffers[1];
  index_buffer_object_ = buffers[2];
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

double Lens::GetIndexOfRefraction() const {
  return n_;
}

const glm::vec3& Lens::GetPosition() const {
  return position_;
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

const glm::vec3& Lens::GetOrientation() const {
  return orientation_;
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

  // Create the lens geometry.
  PopulateLensBufferObjects(true /*1st lens*/, &vertices, &normals, &indices);
  PopulateLensBufferObjects(false /*2nd lens*/, &vertices, &normals, &indices);
  // PopulateCylinderBufferObject(static_cast<int>(vertices.size()) / 3, &indices);

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

bool Lens::PopulateLensBufferObjects(bool lens1,
                                     std::vector<GLfloat>* vertices,
                                     std::vector<GLfloat>* normals,
                                     std::vector<GLuint>* indices) {
  if (!vertices || !normals || !indices) {
    std::cout << "At least one input container is a null pointer." << std::endl;
    return false;
  }

  // The lens surface is made of rings of varying vertical angle.
  // We will put all of the vertex and normal data into 'vertices' and
  // 'normals', but will also store indices in 'lens_rings' to determine how
  // vertices are connected into faces that we can render.
  std::vector<Ring> lens_rings;
  const double radius = lens1 ? r1_ : r2_;
  const double sign = lens1 ? 1.0 : -1.0;

  // Figure out how many rings we need for the this surface.
  const double alpha = 0.5 * M_PI - std::acos(0.5 * w_ / std::abs(radius));
  const unsigned int n_rings = std::floor(alpha / vertical_increment_);

  // Store vertical angles to iterate over.
  std::vector<double> v_angles, h_angles;
  for (unsigned int ii = 0; ii < n_rings; ++ii)
    v_angles.push_back(vertical_increment_ * static_cast<double>(ii));
  v_angles.push_back(alpha);

  for (unsigned int ii = 0; ii < 2.0 * M_PI / horizontal_increment_; ++ii)
    h_angles.push_back(horizontal_increment_ * static_cast<double>(ii));

  // Set an offset for vertex z position.
  const double offset = 0.5 * t_;

  // Check for lens feasibility.
  if (w_ > 2.0 * std::abs(radius)) {
    std::cout << "Lens parameters generate an infeasible lens." << std::endl;
    return false;
  }

  // The first vertex (at the top or bottom of the lens) can be handled
  // separately.
  const int first_index = vertices->size() / 3;
  int index_counter = first_index;
  vertices->push_back(position_.x);
  vertices->push_back(position_.y);
  vertices->push_back(sign * offset + position_.z);
  normals->push_back(0.0);
  normals->push_back(0.0);
  normals->push_back(sign);
  index_counter++;

  // Iterate around the surface of the first lens vertically from the top out.
  for (size_t ii = 1; ii < v_angles.size(); ++ii) {
    double va = v_angles[ii];

    // Iterate around the surface horizontally, storing vertices in a ring.
    Ring ring;
    for (size_t jj = 0; jj < h_angles.size(); ++jj) {
      const double ha = h_angles[jj];

      Vertex v = SphericalToCartesian(radius, va, ha);
      v.z *= sign;

      // Rotate the vertex to the desired orientation.
      // TODO

      // Store the vertex's normal before translating it.
      const double norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      const double flip = radius < 0.0 ? -1.0 : 1.0;
      normals->push_back(flip * v.x / norm);
      normals->push_back(flip * v.y / norm);
      normals->push_back(flip * v.z / norm);

      // Translate the vertex to the desired position.
      v.z += sign * (offset - radius);

      v.x += position_.x;
      v.y += position_.y;
      v.z += position_.z;

      // Store the vertex and normal.
      vertices->push_back(v.x);
      vertices->push_back(v.y);
      vertices->push_back(v.z);

      // Store the index of this vertex/normal pair.
      ring.push_back(index_counter++);
    }
    lens_rings.push_back(ring);
  }

  // Now figure out how to connect lens vertices into triangles.
  // The very top vertices on the lens can be handled separately since they are
  // connected by a single vertex.
  if (lens_rings.size() != 0) {
    for (size_t ii = 0; ii < lens_rings[0].size() - 1; ++ii) {
      indices->push_back(first_index);
      indices->push_back(lens_rings[0][ii]);
      indices->push_back(lens_rings[0][ii + 1]);
    }
    indices->push_back(first_index);
    indices->push_back(lens_rings[0].back());
    indices->push_back(lens_rings[0].front());
  }

  // Connect vertices on the lens that are not a part of the very top circle.
  for (int ii = 0; ii < static_cast<int>(lens_rings.size()) - 1; ++ii) {
    Ring inner_ring = lens_rings[ii];
    Ring outer_ring = lens_rings[ii + 1];

    for (int jj = 0; jj < static_cast<int>(inner_ring.size()) - 1; ++jj) {
      // Each set of 4 vertices (inner, outer, inner+1, outer+1) gives 2 faces.
      indices->push_back(inner_ring[jj]);
      indices->push_back(outer_ring[jj]);
      indices->push_back(outer_ring[jj + 1]);

      indices->push_back(inner_ring[jj]);
      indices->push_back(outer_ring[jj + 1]);
      indices->push_back(inner_ring[jj + 1]);
    }

    // Handle the wrap around vertices.
    indices->push_back(inner_ring.back());
    indices->push_back(outer_ring.back());
    indices->push_back(outer_ring.front());

    indices->push_back(inner_ring.back());
    indices->push_back(outer_ring.front());
    indices->push_back(inner_ring.front());
  }

  return true;
}

bool Lens::PopulateCylinderBufferObject(int n_vertices, std::vector<GLuint>* indices) {
  if (!indices) {
    std::cout << "Index container is a null pointer." << std::endl;
    return false;
  }

  // Connect the outer ring of vertices from each of the two lenses.
  Ring lens1_ring, lens2_ring;
  const int n_ring_vertices = std::floor(2.0 * M_PI / horizontal_increment_);

  for (size_t ii = 0; ii < n_ring_vertices; ++ii) {
    lens1_ring.push_back(n_vertices / 2 - n_ring_vertices + ii);
    lens2_ring.push_back(n_vertices / 1 - n_ring_vertices + ii);
  }

  for (int ii = 0; ii < n_ring_vertices - 1; ++ii) {
    indices->push_back(lens1_ring[ii]);
    indices->push_back(lens1_ring[ii + 1]);
    indices->push_back(lens2_ring[ii]);

    indices->push_back(lens1_ring[ii + 1]);
    indices->push_back(lens2_ring[ii + 1]);
    indices->push_back(lens2_ring[ii]);
  }

  indices->push_back(lens1_ring.back());
  indices->push_back(lens1_ring.front());
  indices->push_back(lens2_ring.back());

  indices->push_back(lens1_ring.front());
  indices->push_back(lens2_ring.front());
  indices->push_back(lens2_ring.back());

  return true;
}

void Lens::Render() const {
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

void Lens::Print(const std::string& prefix) const {
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
