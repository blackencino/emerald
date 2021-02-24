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

#include "UtilGL.h"

namespace EmldCore {
namespace GeepGLFW {
namespace UtilGL {

//-*****************************************************************************
void Init( bool i_experimental )
{
    CheckErrors( "GeepGLFW::init before anything" );

    // On Mac, GLEW stuff is not necessary.
    #ifndef PLATFORM_DARWIN
    glewExperimental = i_experimental ? GL_TRUE : GL_FALSE;
    glewInit();
    #endif
    // Reset errors.
    glGetError();

    printf( "OPEN GL VERSION: %s\n", glGetString( GL_VERSION ) );

    CheckErrors( "GeepGLFW::init glGetString" );
}

//-*****************************************************************************
void CheckErrors( const std::string &i_label )
{
    GLenum errCode;

#ifndef DEBUG
    EMLD_ASSERT( ( errCode = glGetError() ) == GL_NO_ERROR,
                      "OpenGL Error: "
                      << "Code = " << ( int )errCode
                      << " ( Label: " << i_label
                      << " )" );

#else

    if ( ( errCode = glGetError() ) != GL_NO_ERROR )
    {
        std::cerr << "OpenGL Error: "
                  << "Code = " << ( int )errCode
                  << " ( Label: " << i_label
                  << " )" << std::endl;
        abort();
    }

#endif
}

//-*****************************************************************************
void CheckFramebuffer()
{
    GLenum status;
    status = ( GLenum )glCheckFramebufferStatus( GL_FRAMEBUFFER );
    switch ( status )
    {
    case GL_FRAMEBUFFER_COMPLETE:
        return;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        EMLD_THROW(
            "Framebuffer incomplete, incomplete attachment" );
        break;
    case GL_FRAMEBUFFER_UNSUPPORTED:
        EMLD_THROW(
            "Unsupported framebuffer format" );
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        EMLD_THROW(
            "Framebuffer incomplete, missing attachment" );
        break;
    //case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
    //    EMLD_THROW(
    //        "Framebuffer incomplete, attached images "
    //        "must have same dimensions" );
    //    break;
    //case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
    //    EMLD_THROW(
    //        "Framebuffer incomplete, attached images "
    //        "must have same format" );
    //    break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        EMLD_THROW(
            "Framebuffer incomplete, missing draw buffer" );
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        EMLD_THROW(
            "Framebuffer incomplete, missing read buffer" );
        break;
    }
    EMLD_THROW( "Unknown GL Framebuffer error" );
}

} // End namespace UtilGL
} // End namespace GeepGLFW
} // End namespace EmldCore
