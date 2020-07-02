//-*****************************************************************************
// This is the GLFW hello world code, verbatim.
// Only difference is that I'm using the GeepGLFW::util_gl init call.
//-*****************************************************************************

#include <emerald/geep_glfw/foundation.h>
#include <emerald/geep_glfw/util_gl.h>

int main(int, char*[]) {
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit()) { return -1; }

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Hello World", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    /* Init GLEW */
    emerald::geep_glfw::util_gl::Init();

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window)) {
        /* Render here */
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
