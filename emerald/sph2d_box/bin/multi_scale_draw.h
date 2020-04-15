#pragma once

#include <emerald/geep_glfw/foundation.h>
#include <emerald/geep_glfw/program.h>
#include <emerald/simple_sim_viewer/foundation.h>
#include <emerald/util/foundation.h>

#include <array>
#include <memory>
#include <vector>

namespace emerald::sph2d_box {

using namespace emerald::simple_sim_viewer;

struct Single_scale_draw {
    std::array<GLuint, 3> vbos = {0, 0, 0};
    GLuint vao = 0;
    size_t capacity = 0;
    size_t count = 0;
    float radius = 0.0f;
    bool active = false;

    Single_scale_draw() = delete;
    Single_scale_draw(Single_scale_draw const&) = delete;
    Single_scale_draw& operator=(Single_scale_draw const&) = delete;
    Single_scale_draw(Single_scale_draw&& other);
    Single_scale_draw& operator=(Single_scale_draw&&) = delete;

    explicit Single_scale_draw(size_t const _capacity);

    ~Single_scale_draw();

    void update(size_t const in_count,
                V2f const* const centers,
                C4c const* const rgbas);

    void draw() const;
};

struct Multi_scale_draw {
    // drawing a bunch of different levels of detail.
    // we'd make a bunch of them and either draw them or not.

    std::unique_ptr<geep_glfw::Program> program;
    GLint modelview_uniform_location = 0;
    GLint projection_uniform_location = 0;
    GLint radius_uniform_location = 0;
    std::vector<Single_scale_draw> single_scale_drawers;

    Multi_scale_draw() = delete;
    Multi_scale_draw(Multi_scale_draw const&) = delete;
    Multi_scale_draw& operator=(Multi_scale_draw const&) = delete;
    Multi_scale_draw(Multi_scale_draw&&) = default;
    Multi_scale_draw& operator=(Multi_scale_draw&&) = delete;

    Multi_scale_draw(size_t const scale_count, size_t const capacity);

    ~Multi_scale_draw() = default;

    void update_scale(int const scale,
                      float const radius,
                      size_t const count,
                      V2f const* const centers,
                      C4c const* const rgbas);

    void deactivate_scale(int const scale);
    void deactivate_all_scales();

    void draw(M44f const& modelview, M44f const& projection) const;
};

}  // namespace emerald::sph2d_box
