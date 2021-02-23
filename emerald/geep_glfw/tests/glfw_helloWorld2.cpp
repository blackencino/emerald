//-*****************************************************************************
// This comes straight out of the GLFW docs, with a bit of reformatting
// by me. I'm also calling the util_gl Init inside the GeepGLFW library.
//-*****************************************************************************

#include <emerald/geep_glfw/foundation.h>
#include <emerald/geep_glfw/util_gl.h>

#include <cstdio>
#include <cstdlib>

static void error_callback(int error, const char* description) {
    std::fputs(description, stderr);
}

static void key_callback(
  GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

int main(int, char*[]) {
    GLFWwindow* window;
    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) { std::exit(EXIT_FAILURE); }

    window = glfwCreateWindow(640, 480, "Simple example", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    // CJH: This is the only part I added.
    emerald::geep_glfw::util_gl::Init();

    glfwSetKeyCallback(window, key_callback);
    while (!glfwWindowShouldClose(window)) {
        float ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float)height;
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);
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
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    std::exit(EXIT_SUCCESS);
}
