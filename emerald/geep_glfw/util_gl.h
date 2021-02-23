#pragma once

#include <string>

namespace emerald {
namespace geep_glfw {

//-*****************************************************************************
//! Notational grouping namespace for OpenGL-related functions
//
//! The functions declared in the util_gl namespace are global and not part
//! of any particular class.
namespace util_gl {

//-*****************************************************************************
//! OpenGL does not need extension initialization on Mac OSX, but does
//! require initialization via glewInit on non-mac systems. This function
//! abstracts that.
void Init();

//-*****************************************************************************
//! This function will throw an exception with an attached label if the OpenGL
//! Error flag is set. It will get the error string from OpenGL and attach that
//! to the exception text.
void CheckErrors(std::string const& label);

//-*****************************************************************************
//! Checks framebuffer status.
//! Copied directly out of the spec, modified to throw an exception
//! for any failed checks.
void CheckFramebuffer();

}  // End namespace util_gl
}  // End namespace geep_glfw
}  // End namespace emerald
