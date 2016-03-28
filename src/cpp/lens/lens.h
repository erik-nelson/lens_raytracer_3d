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
// This class includes variables for parameterizing a lens and raytracing
// through it.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef LENS_LENS_H
#define LENS_LENS_H

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

class Lens {
 public:
  Lens();
  ~Lens();

  // Allocate buffer objects.
  void Initialize();

  // Getters and setters.
  double GetRadius1() const;
  double GetRadius2() const;
  double GetThickness() const;
  double GetWidth() const;
  double GetIndexOfRefraction() const;
  const glm::vec3& GetPosition() const;
  double GetX() const;
  double GetY() const;
  double GetZ() const;
  const glm::vec3& GetOrientation() const;
  double GetRoll() const;
  double GetPitch() const;
  double GetYaw() const;

  void SetRadius1(double r1);
  void SetRadius2(double r2);
  void SetThickness(double t);
  void SetWidth(double w);
  void SetIndexOfRefraction(double n);
  void SetX(double x);
  void SetY(double y);
  void SetZ(double z);
  void SetRoll(double roll);
  void SetPitch(double pitch);
  void SetYaw(double yaw);

  // After setting parameters, create vertex, normal, and index buffer objects.
  void MakeBufferObjects();

  // Render this lens.
  void Render() const;

  // Print all lens parameters.
  void Print(const std::string& prefix = std::string()) const;

 private:

  // A structure to hold (x, y, z) vertices.
  struct Vertex {
    GLfloat x;
    GLfloat y;
    GLfloat z;
  };

  // A set of indices of vertices placed in a circular pattern.
  typedef std::vector<GLuint> Ring;

  // Populate buffer objects with the geometry of the lens faces.
  bool PopulateLensBufferObjects(bool lens1, std::vector<GLfloat>* vertices,
                                 std::vector<GLfloat>* normals,
                                 std::vector<GLuint>* lens_indices,
                                 std::vector<GLuint>* cylinder_indices);

  // Populate buffer objects with the geometry of the cylinder connecting the
  // two lens faces.
  bool PopulateCylinderBufferObject(
      const std::vector<GLuint>& cylinder_indices1,
      const std::vector<GLuint>& cylinder_indices2,
      std::vector<GLuint>* indices);

  Vertex SphericalToCartesian(double radius, double vertical_angle,
                              double horizontal_angle);

  // Lenses are parameterized by an inner and outer radius, a thickness (at its
  // center), a width, and an index of refraction.
  double r1_;
  double r2_;
  double t_;
  double w_;
  double n_;

  // Lens position and orientation.
  glm::vec3 position_;
  glm::vec3 orientation_;

  // Buffer objects: lens vertices and normals, as well as index buffer objects
  // to draw lens faces ('lens_index_buffer_object_') and the outline around
  // lenses ('outline_index_buffer_objet_').
  GLuint vertex_buffer_object_;
  GLuint normal_buffer_object_;
  GLuint lens_index_buffer_object_;
  GLuint outline_index_buffer_object_;
  int lens_index_size_, outline_index_size_;

  // Variables defining how a lens is rendered.
  static constexpr double vertical_increment_ = 1.0 * M_PI / 180.0;
  static constexpr double horizontal_increment_ = 5.0 * M_PI / 180.0;

};

#endif
