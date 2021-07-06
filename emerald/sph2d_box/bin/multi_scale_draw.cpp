#include <emerald/sph2d_box/bin/multi_scale_draw.h>

#include <emerald/geep_glfw/util_gl.h>
#include <emerald/util/assert.h>

#include <fmt/format.h>

#include <algorithm>

namespace emerald::sph2d_box {

static auto const* const vertex_source =
  R"SHADER(
    #version 410

    in vec2 quad_corner;
    in vec2 center;
    in vec4 rgba;

    uniform mat4x4 modelview;
    uniform mat4x4 projection;

    uniform float radius;

    out vec4 v_rgba;
    out vec2 v_ndc;

    void main() {
        v_ndc = quad_corner;
        v_rgba = rgba;

        vec2 pos2d = center + radius * quad_corner;
        gl_Position = projection * modelview * vec4(pos2d, 0, 1);
    }
)SHADER";

static auto const* const fragment_source =
  R"SHADER(
    #version 410

    in vec4 v_rgba;
    in vec2 v_ndc;
    out vec4 fragment_color;

    void main() {
        float r = length(v_ndc);
        if (r > 1) {
            discard;
        }

        fragment_color = v_rgba;
    }
)SHADER";

//------------------------------------------------------------------------------
Single_scale_draw::Single_scale_draw(Single_scale_draw&& other)
  : vbos(other.vbos)
  , vao(other.vao)
  , capacity(other.capacity)
  , count(other.count)
  , radius(other.radius)
  , active(other.active) {
    other.vbos = {0, 0, 0};
    other.vao = 0;
}

Single_scale_draw::Single_scale_draw(size_t const _capacity)
  : capacity(_capacity) {
    util_gl::CheckErrors("Single_scale_draw begin ctor");

    glGenVertexArrays(1, &vao);
    util_gl::CheckErrors("glGenVertexArrays");
    EMLD_ASSERT(vao > 0, "Failed to create VAO");

    glBindVertexArray(vao);
    util_gl::CheckErrors("glBindVertexArray 1");

    glGenBuffers(3, vbos.data());
    util_gl::CheckErrors("glGenBuffers");
    EMLD_ASSERT(
      std::all_of(
        vbos.begin(), vbos.end(), [](auto const vbo) { return vbo > 0; }),
      "Failed to create VBO");

    // bind the quad corners to position 0
    static const std::array quad_vertex_data = {
      V2f{-1.0f, -1.0f}, V2f{1.0f, -1.0f}, V2f{-1.0f, 1.0f}, V2f{1.0f, 1.0f}};

    auto const bind_attribute = [this](GLuint const index,
                                       GLsizeiptr const capacity,
                                       void const* const data,
                                       GLenum const usage,
                                       GLint const dimension,
                                       GLenum const type,
                                       GLboolean const normalized,
                                       GLuint const divisor) {
        glBindBuffer(GL_ARRAY_BUFFER, vbos[index]);
        util_gl::CheckErrors("glBindBuffer");

        glBufferData(GL_ARRAY_BUFFER, capacity, data, usage);
        util_gl::CheckErrors("glBufferData");

        glVertexAttribPointer(index, dimension, type, normalized, 0, nullptr);
        util_gl::CheckErrors("glVertexAttribPointer");

        glVertexAttribDivisor(index, divisor);
        util_gl::CheckErrors("glVertexAttribDivisor");

        glEnableVertexAttribArray(index);
        util_gl::CheckErrors("glEnableVertexAttribArray");
    };

    bind_attribute(0,
                   sizeof(V2f) * 4,
                   reinterpret_cast<void const*>(quad_vertex_data.data()),
                   GL_STATIC_DRAW,
                   2,
                   GL_FLOAT,
                   GL_FALSE,
                   0);
    bind_attribute(1,
                   sizeof(V2f) * capacity,
                   nullptr,
                   GL_DYNAMIC_DRAW,
                   2,
                   GL_FLOAT,
                   GL_FALSE,
                   1);
    bind_attribute(2,
                   sizeof(C4c) * capacity,
                   nullptr,
                   GL_DYNAMIC_DRAW,
                   4,
                   GL_UNSIGNED_BYTE,
                   GL_TRUE,
                   1);

    glBindVertexArray(0);
    util_gl::CheckErrors("glBindVertexArray 3 (unbind)");
}

Single_scale_draw::~Single_scale_draw() {
    if (vao > 0) {
        glDeleteVertexArrays(1, &vao);
        util_gl::CheckErrors("glDeleteVertexArrays");
        vao = 0;
    }

    if (std::all_of(
          vbos.begin(), vbos.end(), [](auto const vbo) { return vbo > 0; })) {
        glDeleteBuffers(static_cast<GLsizei>(vbos.size()), vbos.data());
        util_gl::CheckErrors("glDeleteBuffers");
        std::fill(vbos.begin(), vbos.end(), 0);
    }
}

void Single_scale_draw::update(size_t const in_count,
                               V2f const* const centers,
                               C4c const* const rgbas) {
    // util_gl::CheckErrors("Single_scale_draw::update begin");

    if (in_count > capacity) {
        auto const new_buffer = [this](GLuint const index,
                                       GLsizeiptr const size,
                                       void const* const data) {
            glBindBuffer(GL_ARRAY_BUFFER, vbos[index]);
            // util_gl::CheckErrors("glBindBuffer");

            glBufferData(GL_ARRAY_BUFFER, size, data, GL_DYNAMIC_DRAW);
            // util_gl::CheckErrors("glBufferData");
        };

        new_buffer(
          1, sizeof(V2f) * in_count, reinterpret_cast<GLvoid const*>(centers));

        new_buffer(
          2, sizeof(C4c) * in_count, reinterpret_cast<GLvoid const*>(rgbas));

        capacity = in_count;
    } else {
        auto const fill_buffer = [this](GLuint const index,
                                        GLsizeiptr const size,
                                        void const* const data) {
            glBindBuffer(GL_ARRAY_BUFFER, vbos[index]);
            // util_gl::CheckErrors("glBindBuffer");

            glBufferSubData(GL_ARRAY_BUFFER, 0, size, data);
            // util_gl::CheckErrors("glBufferSubData");
        };

        fill_buffer(
          1, sizeof(V2f) * in_count, reinterpret_cast<GLvoid const*>(centers));

        fill_buffer(
          2, sizeof(C4c) * in_count, reinterpret_cast<GLvoid const*>(rgbas));
    }

    count = in_count;
}

void Single_scale_draw::draw() const {
    // util_gl::CheckErrors("Single_scale_draw enter");

    if (active && radius > 0.0f && count > 0) {
        glBindVertexArray(vao);
        // util_gl::CheckErrors("glBindVertexArray 2: " + std::to_string(vao));

        glDrawArraysInstanced(
          GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(count));
        // util_gl::CheckErrors("glDrawArraysInstanced");
    }
}

//------------------------------------------------------------------------------
Multi_scale_draw::Multi_scale_draw(size_t const scale_count,
                                   size_t const capacity) {
    single_scale_drawers.reserve(scale_count);
    for (size_t i = 0; i < scale_count; ++i) {
        single_scale_drawers.emplace_back(capacity);
    }

    util_gl::CheckErrors("After scale drawers create in Multi_scale_draw ctor");

    geep_glfw::Program::Bindings const vtx_bindings = {
      {0, "quad_corner"}, {1, "center"}, {2, "rgba"}};
    geep_glfw::Program::Bindings const frg_bindings = {{0, "fragment_color"}};
    program =
      std::make_unique<geep_glfw::Program>("Draw quad helper",
                                           vertex_source,
                                           "",
                                           fragment_source,
                                           vtx_bindings,
                                           frg_bindings,
                                           single_scale_drawers.front().vao);

    auto const get_uniform = [this](GLchar const* const name) -> GLint {
        auto const ret = glGetUniformLocation(program->id(), name);
        util_gl::CheckErrors("glGetUniformLocation");

        EMLD_ASSERT(ret >= 0, "Could not find location for uniform: " << name);
        return ret;
    };

    modelview_uniform_location = get_uniform("modelview");
    projection_uniform_location = get_uniform("projection");
    radius_uniform_location = get_uniform("radius");
}

void Multi_scale_draw::update_scale(int const scale,
                                    float const radius,
                                    size_t const count,
                                    V2f const* const centers,
                                    C4c const* const rgbas) {
    auto& single_scale_draw = single_scale_drawers.at(scale);
    single_scale_draw.radius = radius;
    single_scale_draw.active = true;
    single_scale_draw.update(count, centers, rgbas);
}

void Multi_scale_draw::deactivate_scale(int const scale) {
    single_scale_drawers.at(scale).active = false;
}

void Multi_scale_draw::deactivate_all_scales() {
    std::for_each(
      single_scale_drawers.begin(),
      single_scale_drawers.end(),
      [](auto& single_scale_draw) { single_scale_draw.active = false; });
}

void Multi_scale_draw::draw(M44f const& modelview,
                            M44f const& projection) const {
    program->use();
    glUniformMatrix4fv(modelview_uniform_location,
                       1,
                       GL_FALSE,
                       reinterpret_cast<GLfloat const*>(&(modelview[0][0])));
    // util_gl::CheckErrors("glUniformMatrix4fv");

    glUniformMatrix4fv(projection_uniform_location,
                       1,
                       GL_FALSE,
                       reinterpret_cast<GLfloat const*>(&(projection[0][0])));
    // util_gl::CheckErrors("glUniformMatrix4fv");

    for (auto const& single_scale_draw : single_scale_drawers) {
        glUniform1f(radius_uniform_location, single_scale_draw.radius);
        // util_gl::CheckErrors("glUniform1f");
        single_scale_draw.draw();
    }

    program->unuse();
}

}  // namespace emerald::sph2d_box
