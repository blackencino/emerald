#pragma once

#include <emerald/simple_sim_viewer/foundation.h>
#include <emerald/simple_sim_viewer/gl_camera.h>
#include <emerald/simple_sim_viewer/sim.h>

#include <emerald/util/timer.h>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
class Viewer;

//-*****************************************************************************
class GLFWGlobal {
public:
    GLFWGlobal();
    ~GLFWGlobal();

    void registerWindowAndViewer(GLFWwindow* i_window, Viewer* i_viewer);

    void unregisterWindowAndAllViewers(GLFWwindow* i_window);

    Viewer* viewerFromWindow(GLFWwindow* i_window,
                             bool i_throwIfNotFound = true);

protected:
    typedef std::map<GLFWwindow*, Viewer*> WindowViewerMap;
    WindowViewerMap m_wvMap;
};

//-*****************************************************************************
class Viewer {
public:
    static const int BMASK_LEFT = 0x1 << 0;    // binary 001
    static const int BMASK_MIDDLE = 0x1 << 1;  // binary 010
    static const int BMASK_RIGHT = 0x1 << 2;   // binary 100

    explicit Viewer(SimPtr i_sim,
                    bool i_anim = false,
                    bool i_run_in_constructor = true);
    ~Viewer();

    virtual void init();
    virtual void run();
    virtual void tick(bool i_force);
    virtual void display();
    virtual void reshape(int i_width, int i_height);
    virtual void keyboard(int i_key, int i_scancode, int i_action, int i_mods);
    virtual void character(unsigned int i_char);
    virtual void mouse(int i_button, int i_action, int i_mods);
    virtual void mouse_drag(double x, double y);
    virtual GLFWwindow* window() {
        return m_window;
    }

protected:
    SimPtr m_sim;

    GLFWwindow* m_window;

    int m_buttonMask;
    double m_mouseX;
    double m_mouseY;
    double m_lastX;
    double m_lastY;
    int m_keyMods;

    bool m_animating;

    // GLCamera m_camera;
    Timer m_playbackTimer;
};

//-*****************************************************************************
void SimpleViewSim(SimPtr i_sim, bool i_playing = false);

}  // End namespace simple_sim_viewer
}  // End namespace emerald
