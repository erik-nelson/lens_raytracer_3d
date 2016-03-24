#define LIGHTPOS vec3(0, 2, 0)
#define LIGHTCOL vec3(1, 1, 1)

// MVP matrices and materials.
uniform mat4 u_model_matrix;
uniform mat4 u_view_matrix;
uniform mat4 u_proj_matrix;
uniform vec3 u_camera_position;
uniform float u_phong_shading;
uniform float u_alpha;
uniform vec3 u_ambient_mat;
uniform vec3 u_diffuse_mat;
uniform vec3 u_specular_mat;

// Attributes passed from the CPU.
attribute vec3 a_position;
attribute vec3 a_normal;

// Variables that will be passed to the fragment shader.
varying float v_phong_shading;
varying float v_alpha;
varying vec3 v_vertex_normal;
varying vec3 v_ambient_color;
varying vec3 v_diffuse_color;
varying vec3 v_specular_color;
varying vec3 v_light_color;
varying vec3 v_light_position;
varying vec3 v_camera_position;

void main() {
  // Set vertex front color.
  gl_FrontColor = gl_Color;

  // Compute vertex position by applying MVP transforms.
  vec4 vertex_position = u_proj_matrix * u_view_matrix * u_model_matrix *
                         vec4(a_position.x, a_position.y, a_position.z, 1.0);
  gl_Position = vertex_position;

  // Transform vertex normal by model matrix.
  vec4 vertex_normal = vec4(a_normal.x, a_normal.y, a_normal.z, 1.0);
  vertex_normal = u_model_matrix * vertex_normal;
  v_vertex_normal = normalize(vec3(vertex_normal.x, vertex_normal.y, vertex_normal.z));

  // Compute the light position and color.
  v_light_position = LIGHTPOS;
  v_light_color = LIGHTCOL;

  // Set the remaining varying variables for the fragment shader.
  v_camera_position = u_camera_position;
  v_phong_shading = u_phong_shading;
  v_ambient_color = u_ambient_mat;
  v_diffuse_color = u_diffuse_mat;
  v_specular_color = u_specular_mat;
  v_alpha = u_alpha;

}
