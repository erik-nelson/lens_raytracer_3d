// Variables passed in from the vertex shader.
varying vec3 v_vertex_normal;
varying vec3 v_ambient_color;
varying vec3 v_diffuse_color;
varying vec3 v_specular_color;
varying vec3 v_light_color;
varying vec3 v_light_position;
varying vec3 v_camera_position;

void main() {

  vec3 N = v_vertex_normal;
  vec3 V = v_camera_position;
  vec3 L = v_light_position;

  float diffuse_dot = max(0.0, dot(N, L));
  float specular_dot = max(0.0, dot(N, L + V / 2.0));

  // Compute the diffuse, ambient, and specular colors of this pixel fragment.
  vec3 diffuse_light = v_diffuse_color * diffuse_dot;
  vec3 ambient_light = v_ambient_color;
  vec3 specular_light = v_specular_color * pow(specular_dot, 0.2);

  // Color is a sum of diffuse, ambient, and specular light contributions.
  // vec3 vertex_color = specular_light * v_light_color;
  vec3 vertex_color = (diffuse_light + ambient_light) * v_light_color;
  // gl_FragColor = vec4(vertex_color.x, vertex_color.y, vertex_color.z, 1.0);
  gl_FragColor = vec4(v_vertex_normal.x, v_vertex_normal.y, v_vertex_normal.z, 1.0);
}
