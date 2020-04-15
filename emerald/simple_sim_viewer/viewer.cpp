#include <emerald/simple_sim_viewer/viewer.h>

#include <emerald/util/assert.h>

#include <iostream>

//-*****************************************************************************
namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
//-*****************************************************************************
// GLFW GLOBAL THING
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
static void error_callback(int error, const char* description) {
    std::cerr << description << std::endl;
}

//-*****************************************************************************
GLFWGlobal::GLFWGlobal() {
    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) { EMLD_FAIL("glfwInit returned false"); }
}

//-*****************************************************************************
GLFWGlobal::~GLFWGlobal() {
    glfwTerminate();
}

//-*****************************************************************************
void GLFWGlobal::registerWindowAndViewer(GLFWwindow* i_window,
                                         Viewer* i_viewer) {
    EMLD_ASSERT(i_window != nullptr && i_viewer != nullptr,
                "Cannot register null window or viewer.");

    // Check to make sure it's not already registered.
    auto fiter = m_wvMap.find(i_window);
    if (fiter != m_wvMap.end()) {
        EMLD_FAIL("Can't register GLFWwindow twice.");
    }

    // Register!
    m_wvMap[i_window] = i_viewer;
}

//-*****************************************************************************
void GLFWGlobal::unregisterWindowAndAllViewers(GLFWwindow* i_window) {
    // Map can only have one viewer per window anyway.
    m_wvMap.erase(i_window);
}

//-*****************************************************************************
Viewer* GLFWGlobal::viewerFromWindow(GLFWwindow* i_window,
                                     bool i_throwIfNotFound) {
    EMLD_ASSERT(i_window != nullptr, "Cannot get viewer from null window.");

    auto fiter = m_wvMap.find(i_window);
    if (fiter == m_wvMap.end()) {
        if (i_throwIfNotFound) {
            EMLD_FAIL("Could not find viewer for window.");
        } else {
            return nullptr;
        }
    }

    if ((*fiter).second->window() != i_window) {
        EMLD_FAIL("Corrupt window-viewer map in GLFWGlobal");
    }

    return (*fiter).second;
}

//-*****************************************************************************
// STATIC GLOBAL SINGLETON, AS A UNIQUE PTR.
//-*****************************************************************************
static std::unique_ptr<GLFWGlobal> g_glfwGlobal;

//-*****************************************************************************
//-*****************************************************************************
// GLOBAL GLFW FUNCTIONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
void g_reshape(GLFWwindow* i_window, int i_width, int i_height) {
    EMLD_ASSERT(i_window != nullptr, "nullptr Viewer");
    EMLD_ASSERT(g_glfwGlobal.get() != nullptr, "NO GLFW GLOBAL");

    // This will throw if viewer is nullptr.
    Viewer* viewer = g_glfwGlobal->viewerFromWindow(i_window, true);

    viewer->reshape(i_width, i_height);
}

//-*****************************************************************************
void g_character(GLFWwindow* i_window, unsigned int i_char) {
    EMLD_ASSERT(i_window != nullptr, "nullptr Viewer");
    EMLD_ASSERT(g_glfwGlobal.get() != nullptr, "NO GLFW GLOBAL");

    // This will throw if viewer is nullptr.
    Viewer* viewer = g_glfwGlobal->viewerFromWindow(i_window, true);

    viewer->character(i_char);
}

//-*****************************************************************************
void g_keyboard(
    GLFWwindow* i_window, int i_key, int i_scancode, int i_action, int i_mods) {
    EMLD_ASSERT(i_window != nullptr, "nullptr Viewer");
    EMLD_ASSERT(g_glfwGlobal.get() != nullptr, "NO GLFW GLOBAL");

    // This will throw if viewer is nullptr.
    Viewer* viewer = g_glfwGlobal->viewerFromWindow(i_window, true);

    viewer->keyboard(i_key, i_scancode, i_action, i_mods);
}

//-*****************************************************************************
void g_mouse(GLFWwindow* i_window, int i_button, int i_action, int i_mods) {
    EMLD_ASSERT(i_window != nullptr, "nullptr Viewer");
    EMLD_ASSERT(g_glfwGlobal.get() != nullptr, "NO GLFW GLOBAL");

    // This will throw if viewer is nullptr.
    Viewer* viewer = g_glfwGlobal->viewerFromWindow(i_window, true);

    viewer->mouse(i_button, i_action, i_mods);
}

//-*****************************************************************************
void g_mouse_drag(GLFWwindow* i_window, double i_fx, double i_fy) {
    EMLD_ASSERT(i_window != nullptr, "nullptr Viewer");
    EMLD_ASSERT(g_glfwGlobal.get() != nullptr, "NO GLFW GLOBAL");

    // This will throw if viewer is nullptr.
    Viewer* viewer = g_glfwGlobal->viewerFromWindow(i_window, true);

    viewer->mouse_drag(i_fx, i_fy);
}

//-*****************************************************************************
//-*****************************************************************************
// VIEWER
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
Viewer::Viewer(SimPtr i_simPtr, bool i_anim, bool i_run_in_constructor)
  : m_sim(i_simPtr)
  , m_window(nullptr)
  , m_buttonMask(0)
  , m_mouseX(0)
  , m_mouseY(0)
  , m_lastX(0)
  , m_lastY(0)
  , m_keyMods(0)
  , m_animating(i_anim)
  //, m_camera()
  , m_playbackTimer() {
    if (i_run_in_constructor) { run(); }
}

//-*****************************************************************************
Viewer::~Viewer() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        if (g_glfwGlobal) {
            g_glfwGlobal->unregisterWindowAndAllViewers(m_window);
        }
        m_window = nullptr;
    }
}

void Viewer::run() {
    // Initialize the global, if needed. This is so that the global doesn't
    // get created unless a viewer is requested.
    if (!g_glfwGlobal) { g_glfwGlobal.reset(new GLFWGlobal); }

    // Make it all modern-like.
    // glfwWindowHint( GLFW_CLIENT_API, GLFW_OPENGL_API );
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#ifdef DEBUG
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Make a window for this sim!
    auto const preferred_window_size = m_sim->preferred_window_size();
    m_window = glfwCreateWindow(preferred_window_size.x,
                                preferred_window_size.y,
                                m_sim->name().c_str(),
                                nullptr,  // glfwGetPrimaryMonitor(),
                                nullptr);
    if (!m_window) { EMLD_FAIL("Null Window returned from glfwCreateWindow"); }

    // Associate the window with the viewer in the global.
    g_glfwGlobal->registerWindowAndViewer(m_window, this);

    // Make context current.
    glfwMakeContextCurrent(m_window);

    // Init GLEW and other stuff.
    emerald::geep_glfw::util_gl::Init(true);
    init();

    // Register callbacks.
    glfwSetKeyCallback(m_window, g_keyboard);
    glfwSetCharCallback(m_window, g_character);
    glfwSetCursorPosCallback(m_window, g_mouse_drag);
    glfwSetMouseButtonCallback(m_window, g_mouse);
    glfwSetWindowSizeCallback(m_window, g_reshape);

    // Main loop.
    while (!glfwWindowShouldClose(m_window)) {
        tick(false);
        if (m_sim->needs_redraw()) {
            display();
            glFlush();
            glfwSwapBuffers(m_window);
        }
        glfwPollEvents();

        glfwSetWindowTitle(m_window, m_sim->name().c_str());
    }

    glfwDestroyWindow(m_window);
    g_glfwGlobal->unregisterWindowAndAllViewers(m_window);
    m_window = nullptr;
}

//-*****************************************************************************
void Viewer::init(void) {
    util_gl::CheckErrors("Viewer::init() begin");

    glClearColor(0.0, 0.0, 0.0, 0.0);
    util_gl::CheckErrors("Viewer::init() glClearColor");

    glEnable(GL_DEPTH_TEST);
    util_gl::CheckErrors("Viewer::init() glEnable GL_DEPTH_TEST");

    glEnable(GL_BLEND);
    util_gl::CheckErrors("Viewer::init() glEnable GL_BLEND");
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    util_gl::CheckErrors("Viewer::init() glBlendFunc");

    glDisable(GL_CULL_FACE);
    util_gl::CheckErrors("Viewer::init() glDisable GL_CULL_FACE");

    m_buttonMask = 0;

    V2i pws = m_sim->preferred_window_size();
    m_sim->init_draw(pws.x, pws.y);
}

//-*****************************************************************************
void Viewer::tick(bool i_force) {
    if (i_force || m_animating) {
        if (i_force || m_playbackTimer.elapsed() > 1.0 / 60.0) {
            m_playbackTimer.stop();
            m_playbackTimer.start();
            m_sim->step();
        }
    }
}

//-*****************************************************************************
void Viewer::display() {
    m_sim->outer_draw();
    util_gl::CheckErrors("Viewer::display() glFlush");
}

//-*****************************************************************************
void Viewer::reshape(int i_width, int i_height) {
    m_sim->reshape(i_width, i_height);
}

//-*****************************************************************************
void Viewer::keyboard(int i_key, int i_scancode, int i_action, int i_mods) {
    // Shift
    if (i_mods & GLFW_MOD_SHIFT) {
        m_keyMods |= GLFW_MOD_SHIFT;
    } else {
        m_keyMods &= ~GLFW_MOD_SHIFT;
    }

    // Control
    if (i_mods & GLFW_MOD_CONTROL) {
        m_keyMods |= GLFW_MOD_CONTROL;
    } else {
        m_keyMods &= ~GLFW_MOD_CONTROL;
    }

    // Alt
    if (i_mods & GLFW_MOD_ALT) {
        m_keyMods |= GLFW_MOD_ALT;
    } else {
        m_keyMods &= ~GLFW_MOD_ALT;
    }

    // Super
    if (i_mods & GLFW_MOD_SUPER) {
        m_keyMods |= GLFW_MOD_SUPER;
    } else {
        m_keyMods &= ~GLFW_MOD_SUPER;
    }

    // std::cout << "Key hit: " << ( int )i_key << std::endl;
    if (i_key == GLFW_KEY_ESCAPE && i_action == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_window, GL_TRUE);
        return;
    }

    double DBLx;
    double DBLy;
    glfwGetCursorPos(m_window, &DBLx, &DBLy);
    int x = (int)DBLx;
    int y = (int)DBLy;

    m_sim->keyboard(i_key, i_scancode, i_action, i_mods, x, y);
}

//-*****************************************************************************
void Viewer::character(unsigned int i_char) {
    switch (i_char) {
    case 'f':
    case 'F': m_sim->frame_camera(); break;
    case ' ': tick(true); break;

    case '>':
    case '.': m_animating = !m_animating; break;
    default: break;
    }

    double DBLx;
    double DBLy;
    glfwGetCursorPos(m_window, &DBLx, &DBLy);
    int x = (int)DBLx;
    int y = (int)DBLy;
    m_sim->character(i_char, x, y);
}

//-*****************************************************************************
void Viewer::mouse(int i_button, int i_action, int i_mods) {
    m_lastX = m_mouseX;
    m_lastY = m_mouseY;
    glfwGetCursorPos(m_window, &m_mouseX, &m_mouseY);

    // Shift
    if (i_mods & GLFW_MOD_SHIFT) {
        m_keyMods |= GLFW_MOD_SHIFT;
    } else {
        m_keyMods &= ~GLFW_MOD_SHIFT;
    }

    // Control
    if (i_mods & GLFW_MOD_CONTROL) {
        m_keyMods |= GLFW_MOD_CONTROL;
    } else {
        m_keyMods &= ~GLFW_MOD_CONTROL;
    }

    // Alt
    if (i_mods & GLFW_MOD_ALT) {
        m_keyMods |= GLFW_MOD_ALT;
    } else {
        m_keyMods &= ~GLFW_MOD_ALT;
    }

    // Super
    if (i_mods & GLFW_MOD_SUPER) {
        m_keyMods |= GLFW_MOD_SUPER;
    } else {
        m_keyMods &= ~GLFW_MOD_SUPER;
    }

    if (i_action == GLFW_PRESS) {
        switch (i_button) {
        case GLFW_MOUSE_BUTTON_LEFT:
            m_buttonMask = m_buttonMask | BMASK_LEFT;
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            m_buttonMask = m_buttonMask | BMASK_MIDDLE;
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            m_buttonMask = m_buttonMask | BMASK_RIGHT;
            break;
        }
    } else {
        switch (i_button) {
        case GLFW_MOUSE_BUTTON_LEFT:
            m_buttonMask = m_buttonMask & ~BMASK_LEFT;
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            m_buttonMask = m_buttonMask & ~BMASK_MIDDLE;
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            m_buttonMask = m_buttonMask & ~BMASK_RIGHT;
            break;
        }
    }
    m_sim->mouse(
        i_button, i_action, i_mods, m_mouseX, m_mouseY, m_lastX, m_lastY);
}

//-*****************************************************************************
void Viewer::mouse_drag(double x, double y) {
    m_lastX = m_mouseX;
    m_lastY = m_mouseY;
    m_mouseX = x;
    m_mouseY = y;
    double dx = m_mouseX - m_lastX;
    double dy = m_mouseY - m_lastY;

    if (m_keyMods & GLFW_MOD_ALT) {
        if (((m_buttonMask & BMASK_LEFT) && (m_buttonMask & BMASK_MIDDLE)) ||
            (m_buttonMask & BMASK_RIGHT)) {
            m_sim->dolly_camera(dx, dy);
        } else if (m_buttonMask & BMASK_LEFT) {
            m_sim->rotate_camera(dx, dy);
        } else if (m_buttonMask & BMASK_MIDDLE) {
            m_sim->track_camera(dx, dy);
        }
    }
    // Some scroll-wheel mice refuse to give middle button signals,
    // so we allow for 'ctrl+left' for tracking.
    else if (m_keyMods & GLFW_MOD_CONTROL) {
        if (m_buttonMask & BMASK_LEFT) { m_sim->track_camera(dx, dy); }
    }
    // Just to make mac usage easier... we'll use SHIFT for dolly also.
    else if (m_keyMods & GLFW_MOD_SHIFT) {
        if (m_buttonMask & BMASK_LEFT) { m_sim->dolly_camera(dx, dy); }
    } else {
        m_sim->mouse_drag(m_mouseX, m_mouseY, m_lastX, m_lastY);
    }
}

//-*****************************************************************************
void SimpleViewSim(SimPtr i_sim, bool i_playing) {
    Viewer v{i_sim, i_playing};
}

}  // End namespace simple_sim_viewer
}  // namespace emerald
