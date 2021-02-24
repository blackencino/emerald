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

#include "Viewer.h"

#error "THIS IS DEPRECATED"

//-*****************************************************************************
namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
static const unsigned char BMASK_LEFT = 0x1;  // binary 001
static const unsigned char BMASK_MIDDLE = 0x2;  // binary 010
static const unsigned char BMASK_RIGHT = 0x4;  // binary 100

//-*****************************************************************************
static State g_state;
static bool g_playStart = false;
static boost::timer g_playbackTimer;

//-*****************************************************************************
void overlay();

//-*****************************************************************************
//-*****************************************************************************
// GLUT STUFF
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
void init( void )
{
    {
        GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
        GLfloat mat_shininess[] = { 100.0 };
        GLfloat light_position[] = { 20.0, 20.0, 20.0, 0.0 };
        GLfloat mat_front_emission[] = {0.0, 0.0, 0.0, 0.0 };
        GLfloat mat_back_emission[] = {1.0f, 0.0, 0.0, 1.0f };

        glClearColor( 0.0, 0.0, 0.0, 0.0 );
        glMaterialfv( GL_FRONT, GL_EMISSION, mat_front_emission );
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular );
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess );

        glMaterialfv( GL_BACK, GL_EMISSION, mat_back_emission );
        glMaterialfv( GL_BACK, GL_SPECULAR, mat_specular );
        glMaterialfv( GL_BACK, GL_SHININESS, mat_shininess );

        glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);
    }

    // glLightfv( GL_LIGHT0, GL_POSITION, light_position );
    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );
    glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);

    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glDisable(GL_CULL_FACE);

    glShadeModel( GL_SMOOTH );

    g_state.cam.setSize( 800, 600 );
    g_state.cam.lookAt( V3d( 24, 18, 24 ), V3d( 0.0 ) );
    g_state.bMask = 0;
    g_state.showHelp = false;
    g_state.anim = g_playStart;
    g_state.pointSize = 3.0f;
    glPointSize( g_state.pointSize );

    g_state.cam.frame( g_state.sim->getBounds() );

    glutSetWindowTitle( g_state.sim->getName().c_str() );
}

//-*****************************************************************************
void TickForward()
{
    g_state.sim->step();
    glutSetWindowTitle( g_state.sim->getName().c_str() );
    g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
    glutPostRedisplay();
}

//-*****************************************************************************
void playFwdIdle()
{
    if ( g_playbackTimer.elapsed() > 1.0/60.0 )
    {
        g_playbackTimer.restart();
        
        TickForward();
    }
}

//-*****************************************************************************
void display( void )
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // glPushMatrix();
    
    double n, f;
    if ( g_state.sim->overrideClipping( g_state.cam, n, f ) )
    {
        g_state.cam.setClippingPlanes( n, f );
    }
    else
    {
        g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
    }
    g_state.sim->outerDraw( g_state.cam );

    // glPopMatrix();
    
    glFlush();
}

//-*****************************************************************************
void reshape( int w, int h )
{
    g_state.cam.setSize( w, h );
    g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );

    glMatrixMode( GL_MODELVIEW );
}

//-*****************************************************************************
void glPrint( GLfloat x, GLfloat y, const std::string &text )
{
    glRasterPos2f( x, y );
    for ( int p = 0; p < text.size(); ++p )
    {
        glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, text[p] );
    }
}

//-*****************************************************************************
void overlay()
{
    if ( !g_state.showHelp )
    {
        return;
    }

    glDisable( GL_LIGHTING );
    glDisable( GL_DEPTH_TEST );

    glColor4f( .1f, .9f, .1f, 1.0f );
    glPrint( -5, 5, "Good" );

    glColor4f( .9f, .1f, .1f, 1.0f );
    glPrint( -5, 4.5, "Inverse Normals" );

    glColor4f( .1f, .1f, .9f, 1.0f );
    glPrint( -5, 4, "Negative Transform" );

    glColor4f( .9f, .1f, .9f, 1.0f );
    glPrint( -5, 3.5, "Negative Transform + Inverse Normals" );

    glEnable( GL_LIGHTING );
    glEnable( GL_DEPTH_TEST );
}

//-*****************************************************************************
void keyboard( unsigned char key, int x, int y )
{
    // std::cout << "Key hit: " << ( int )key << std::endl;

    static bool bf = true;
    static bool xray = false;

    switch ( key )
    {
    case '`':
    case '~':
        glutFullScreen();
        break;
    case 'f':
    case 'F':
        g_state.cam.frame( g_state.sim->getBounds() );
        g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
        glutPostRedisplay();
        break;
    case ' ':
        TickForward();
        break;

    case '>':
    case '.':
        if ( g_state.anim )
        {
            g_state.anim = false;
            glutIdleFunc( NULL );
        }
        else
        {
            g_state.anim = true;
            glutIdleFunc( playFwdIdle );
        }
        break;
    case 'p':
        g_state.pointSize =
            std::max( g_state.pointSize-0.5f, 1.0f );
        glPointSize( g_state.pointSize );
        glutPostRedisplay();
        break;
    case 'P':
        g_state.pointSize =
            std::min( g_state.pointSize+0.5f, 20.0f );
        glPointSize( g_state.pointSize );
        glutPostRedisplay();
        break;
    case 'b':
    case 'B':
        bf = !bf;
        glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, bf ? GL_TRUE : GL_FALSE );
        glutPostRedisplay();
        break;
        break;
    case '\t': // tab
        g_state.showHelp = !g_state.showHelp;
        glutPostRedisplay();
        break;
    case 'c':
        std::cout << "# Camera\n" << g_state.cam.RIB() << std::endl;
        break;
    case 27:
        g_state.sim.reset();
        exit(0);
        break;
    default:
        break;
    }

    g_state.sim->keyboard( key, x, y );
}

//-*****************************************************************************
void mouse( int button, int state, int x, int y )
{
    g_state.last_x = x;
    g_state.last_y = y;
    if ( state == GLUT_DOWN )
    {
        switch( button )
        {
        case GLUT_LEFT_BUTTON:
            g_state.bMask = g_state.bMask | BMASK_LEFT;
            break;
        case GLUT_MIDDLE_BUTTON:
            g_state.bMask = g_state.bMask | BMASK_MIDDLE;
            break;
        case GLUT_RIGHT_BUTTON:
            g_state.bMask = g_state.bMask | BMASK_RIGHT;
            break;
        }
    }
    else
    {
        switch( button )
        {
        case GLUT_LEFT_BUTTON:
            g_state.bMask = g_state.bMask & ~BMASK_LEFT;
            break;
        case GLUT_MIDDLE_BUTTON:
            g_state.bMask = g_state.bMask & ~BMASK_MIDDLE;
            break;
        case GLUT_RIGHT_BUTTON:
            g_state.bMask = g_state.bMask & ~BMASK_RIGHT;
            break;
        }
    }
    g_state.mods = glutGetModifiers();
}

//-*****************************************************************************
void mouseDrag( int x, int y )
{
    int dx = x - g_state.last_x;
    int dy = y - g_state.last_y;

    g_state.last_x = x;
    g_state.last_y = y;

    if ( g_state.mods & GLUT_ACTIVE_ALT )
    {
        if ( ( ( g_state.bMask & BMASK_LEFT ) &&
               ( g_state.bMask & BMASK_MIDDLE ) ) ||
             ( g_state.bMask & BMASK_RIGHT ) )
        {
            g_state.cam.dolly( V2d( dx, dy ) );
            g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
            glutPostRedisplay();
        } 
        else if ( g_state.bMask & BMASK_LEFT )
        {
            g_state.cam.rotate( V2d( dx, dy ) );
            g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
            glutPostRedisplay();
        } 
        else if ( g_state.bMask & BMASK_MIDDLE )
        {
            g_state.cam.track( V2d( dx, dy ) );
            g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
            glutPostRedisplay();
        }
    }
    // Some scroll-wheel mice refuse to give middle button signals,
    // so we allow for 'ctrl+left' for tracking.
    else if ( g_state.mods & GLUT_ACTIVE_CTRL )
    {
        if ( g_state.bMask & BMASK_LEFT )
        {
            g_state.cam.track( V2d( dx, dy ) );
            g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
            glutPostRedisplay();
        }
    }
    // Just to make mac usage easier... we'll use SHIFT for dolly also.
    else if ( g_state.mods & GLUT_ACTIVE_SHIFT )
    {
        if ( g_state.bMask & BMASK_LEFT )
        {
            g_state.cam.dolly( V2d( dx, dy ) );
            g_state.cam.autoSetClippingPlanes( g_state.sim->getBounds() );
            glutPostRedisplay();
        }
    }
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

void SimpleInitSim( SimPtr sim, bool i_playing )
{
    if ( !sim ) { return; }

    g_state.sim = sim;
    g_state.anim = i_playing;
    g_playStart = i_playing;
}

void SimpleViewSim( SimPtr sim, bool i_playing )
{
    SimpleInitSim( sim, i_playing );
    SimpleViewSim();
}

void SimpleViewSim()
{
    // Set up the state.
    //g_state.anim = false;
    //glutInitContextVersion( 2, 1 );
    //glutInitContextProfile( GLUT_CORE_PROFILE );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
    glutInitWindowSize( 800, 600 );
    glutInitWindowPosition( 100, 100 );
    
    char fakeArgv0[] = { 'a', 'b', ( char )0 };
    char *fakeArgv[] = { ( char * )fakeArgv0 };
    int fakeArgc = 1;
    
    glutInit( &fakeArgc, fakeArgv );
    glutCreateWindow( g_state.sim->getName().c_str() );
    
    // Initialize GL.
    //Alembic::GLUtil::InitGL();
    EmldCore::Geep::UtilGL::Init();
    
    // Init local GL stuff
    init();
    g_state.anim = g_playStart;
    
    // Setup Callbacks
    glutDisplayFunc( display );
    glutKeyboardFunc( keyboard );
    glutReshapeFunc( reshape );
    glutMouseFunc( mouse );
    glutMotionFunc( mouseDrag );

    if ( g_state.anim )
    {
        glutIdleFunc( playFwdIdle );
    }
    
    glutMainLoop();
}

} // End namespace SimpleSimViewer
} // End namespace Emld

