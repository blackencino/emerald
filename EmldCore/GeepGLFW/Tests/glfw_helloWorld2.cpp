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

//-*****************************************************************************
// This comes straight out of the GLFW docs, with a bit of reformatting
// by me. I'm also calling the UtilGL Init inside the GeepGLFW library.
//-*****************************************************************************

#include <EmldCore/GeepGLFW/All.h>
#include <stdlib.h>
#include <stdio.h>

static void error_callback( int error, const char* description )
{
    fputs( description, stderr );
}

static void key_callback( GLFWwindow* window, int key,
                          int scancode, int action, int mods )
{
    if ( key == GLFW_KEY_ESCAPE && action == GLFW_PRESS )
    { glfwSetWindowShouldClose( window, GL_TRUE ); }
}

int main( void )
{
    GLFWwindow* window;
    glfwSetErrorCallback( error_callback );

    if ( !glfwInit() )
    { exit( EXIT_FAILURE ); }

    window = glfwCreateWindow( 640, 480, "Simple example", NULL, NULL );
    if ( !window )
    {
        glfwTerminate();
        exit( EXIT_FAILURE );
    }

    glfwMakeContextCurrent( window );

    // CJH: This is the only part I added.
    EmldCore::GeepGLFW::UtilGL::Init( true );

    glfwSetKeyCallback( window, key_callback );
    while ( !glfwWindowShouldClose( window ) )
    {
        float ratio;
        int width, height;
        glfwGetFramebufferSize( window, &width, &height );
        ratio = width / ( float ) height;
        glViewport( 0, 0, width, height );
        glClear( GL_COLOR_BUFFER_BIT );
#if 0
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity();
        glOrtho( -ratio, ratio, -1.f, 1.f, 1.f, -1.f );
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();
        glRotatef( ( float ) glfwGetTime() * 50.f, 0.f, 0.f, 1.f );
        glBegin( GL_TRIANGLES );
        glColor3f( 1.f, 0.f, 0.f );
        glVertex3f( -0.6f, -0.4f, 0.f );
        glColor3f( 0.f, 1.f, 0.f );
        glVertex3f( 0.6f, -0.4f, 0.f );
        glColor3f( 0.f, 0.f, 1.f );
        glVertex3f( 0.f, 0.6f, 0.f );
        glEnd();
#endif
        glfwSwapBuffers( window );
        glfwPollEvents();
    }

    glfwDestroyWindow( window );
    glfwTerminate();
    exit( EXIT_SUCCESS );
}
