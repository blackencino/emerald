#pragma once

#include <emerald/simple_sim_viewer/foundation.h>
#include <emerald/simple_sim_viewer/gl_camera.h>

#include <memory>
#include <optional>
#include <string>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
class BaseSim {
public:
    BaseSim() {
    }

    virtual ~BaseSim();

    virtual std::string name() const {
        return "BaseSim";
    }

    virtual V2i preferred_window_size() const {
        return V2i{800, 600};
    }

    virtual void init_draw(int w, int h) {
    }
    virtual void reshape(int w, int h) {
    }

    //! Just step forward.
    //! ...
    virtual void step() {
    }
    virtual void frame_camera() {
    }
    virtual void dolly_camera(float dx, float dy) {
    }
    virtual void track_camera(float dx, float dy) {
    }
    virtual void rotate_camera(float dx, float dy) {
    }

    virtual bool needs_redraw() {
        return true;
    }

    //! This draws, assuming a camera matrix has already been set.
    //! ...
    virtual void draw() {
    }

    //! This outer draw function sets up the drawing environment.
    //! ...
    virtual void outer_draw();

    virtual void character(unsigned int i_char, int x, int y) {
    }

    virtual void keyboard(
        int i_key, int i_scancode, int i_action, int i_mods, int i_x, int i_y) {
    }

    virtual void mouse(int i_button,
                       int i_action,
                       int i_mods,
                       double x,
                       double y,
                       double lastX,
                       double lastY) {
    }

    virtual void mouse_drag(double x, double y, double lastX, double lastY) {
    }
};

//-*****************************************************************************
class Sim3D : public BaseSim {
public:
    Sim3D()
      : BaseSim() {
    }

    virtual ~Sim3D();

    std::string name() const override {
        return "Sim3D";
    }

    void init_draw(int w, int h) override;

    void reshape(int w, int h) override;
    void frame_camera() override;
    void dolly_camera(float dx, float dy) override;
    void track_camera(float dx, float dy) override;
    void rotate_camera(float dx, float dy) override;
    virtual Box3d bounds() const {
        return Box3d{V3d{-0.1}, V3d{0.1}};
    }

    virtual std::optional<V2d> override_clipping() const {
        return std::nullopt;
    }

    void outer_draw() override;

    GLCamera const& camera() const {
        return m_camera;
    }

protected:
    GLCamera m_camera;
    bool m_camera_changed_since_last_draw = false;
};

//-*****************************************************************************
class SimSlab : public BaseSim {
public:
    SimSlab()
      : BaseSim() {
    }

    std::string name() const override {
        return "SimSlab";
    }

    void init_draw(int w, int h) override;

    void reshape(int w, int h) override;
    void frame_camera() override {
    }
    void dolly_camera(float dx, float dy) override {
    }
    void track_camera(float dx, float dy) override {
    }
    void rotate_camera(float dx, float dy) override {
    }

    void outer_draw() override;

protected:
    int m_width = 0;
    int m_height = 0;
};

using SimPtr = std::shared_ptr<BaseSim>;

}  // End namespace simple_sim_viewer
}  // End namespace emerald
