#pragma once

#define GLFW_INCLUDE_GLEXT 1
#define GLFW_INCLUDE_GLU 1
#define GLFW_INCLUDE_GLCOREARB 1

// This is a temporary fix that gets around us not currently using GLEW
// for binding the GL function pointers. This is not completely safe.
#define GL_GLEXT_PROTOTYPES 1

#include <GLFW/glfw3.h>


