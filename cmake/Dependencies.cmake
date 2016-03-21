include(ExternalProject)
set(lens_raytracer_3d_LIBRARIES "")

# Find OpenGL.
find_package( OpenGL REQUIRED )
include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
list(APPEND lens_raytracer_3d_LIBRARIES ${OPENGL_LIBRARIES})

# Find Glew.
find_package( GLEW REQUIRED )
include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})
list(APPEND lens_raytracer_3d_LIBRARIES ${GLEW_LIBRARIES})

# Find Glfw.
find_package( GLFW REQUIRED )
include_directories(SYSTEM ${GLFW_INCLUDE_DIRS})
list(APPEND lens_raytracer_3d_LIBRARIES ${GLFW_LIBRARIES})


message("libraries: ${lens_raytracer_3d_LIBRARIES}")
