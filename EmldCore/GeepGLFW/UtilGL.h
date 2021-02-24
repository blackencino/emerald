//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#ifndef _EmldCore_GeepGLFW_UtilGL_h_
#define _EmldCore_GeepGLFW_UtilGL_h_

#include "Foundation.h"

namespace EmldCore {
namespace GeepGLFW {

//-*****************************************************************************
//! Notational grouping namespace for OpenGL-related functions
//
//! The functions declared in the UtilGL namespace are global and not part
//! of any particular class.
namespace UtilGL {

//-*****************************************************************************
//! OpenGL does not need extension initialization on Mac OSX, but does
//! require initialization via glewInit on non-mac systems. This function
//! abstracts that.
void Init( bool i_experimental = true );

//-*****************************************************************************
//! This function will throw an exception with an attached label if the OpenGL
//! Error flag is set. It will get the error string from OpenGL and attach that
//! to the exception text.
void CheckErrors( const std::string &i_label );

//-*****************************************************************************
//! Checks framebuffer status.
//! Copied directly out of the spec, modified to throw an exception
//! for any failed checks.
void CheckFramebuffer();

} // End namespace UtilGL
} // End namespace GeepGLFW
} // End namespace EmldCore

#endif // ifndef _EmldCore_GeepGLFW_UtilGL_h_


