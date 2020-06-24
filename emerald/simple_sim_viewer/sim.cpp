#include <emerald/simple_sim_viewer/sim.h>

#include <emerald/geep_glfw/util_gl.h>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
BaseSim::~BaseSim() {
}

void BaseSim::outer_draw() {
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    util_gl::CheckErrors("outerDraw glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    util_gl::CheckErrors("outerDraw glClear");

    draw();
}

//-*****************************************************************************
Sim3D::~Sim3D() {
}

void Sim3D::init_draw(int w, int h) {
    m_camera.size.x = w;
    m_camera.size.y = h;
    m_camera = frame(look_at(m_camera, V3d{24, 18, 24}, V3d{0.0}), bounds());
    m_camera_changed_since_last_draw = true;
}

void Sim3D::reshape(int w, int h) {
    m_camera.size.x = w;
    m_camera.size.y = h;
    m_camera_changed_since_last_draw = true;
}

void Sim3D::frame_camera() {
    m_camera = frame(m_camera, bounds());
    m_camera_changed_since_last_draw = true;
}

void Sim3D::dolly_camera(float dx, float dy) {
    m_camera = dolly(m_camera, V2d{dx, dy});
    m_camera_changed_since_last_draw = true;
}

void Sim3D::track_camera(float dx, float dy) {
    m_camera = track(m_camera, V2d{dx, dy});
    m_camera_changed_since_last_draw = true;
}

void Sim3D::rotate_camera(float dx, float dy) {
    m_camera = rotate(m_camera, V2d{dx, dy});
    m_camera_changed_since_last_draw = true;
}

void Sim3D::outer_draw() {
    glViewport(0, 0, (GLsizei)m_camera.size.x, (GLsizei)m_camera.size.y);

    auto const optional_clipping = override_clipping();
    if (optional_clipping.has_value()) {
        m_camera.clip = optional_clipping.value();
    } else {
        m_camera.clip = auto_clipping_planes(m_camera, bounds());
    }

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    util_gl::CheckErrors("outerDraw glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    util_gl::CheckErrors("outerDraw glClear");

    draw();

    m_camera_changed_since_last_draw = false;
}

//-*****************************************************************************
void SimSlab::init_draw(int w, int h) {
    m_width = w;
    m_height = h;
}

void SimSlab::reshape(int w, int h) {
    m_width = w;
    m_height = h;
}

void SimSlab::outer_draw() {
    glViewport(0, 0, (GLsizei)m_width, (GLsizei)m_height);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    util_gl::CheckErrors("outerDraw glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    util_gl::CheckErrors("outerDraw glClear");

    draw();
}

}  // End namespace simple_sim_viewer
}  // End namespace emerald
