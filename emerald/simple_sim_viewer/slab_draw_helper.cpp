#include <emerald/simple_sim_viewer/slab_draw_helper.h>

#include <array>

namespace emerald::simple_sim_viewer {

SlabDrawHelper::SlabDrawHelper(V3f const* const pixels,
                               int const width,
                               int const height)
  : m_width(width)
  , m_height(height) {
    util_gl::CheckErrors("slab draw helper init before anything");

    // Create and bind VAO
    glGenVertexArrays(1, &m_vertexArrayObject);
    util_gl::CheckErrors("glGenVertexArrays");
    EMLD_ASSERT(m_vertexArrayObject > 0, "Failed to create VAO");

    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray");

    // Create vertex buffer.
    glGenBuffers(1, &m_quadVertexBufferObject);
    util_gl::CheckErrors("glGenBuffers");
    EMLD_ASSERT(m_quadVertexBufferObject > 0, "Failed to create VBO");

    static const std::array quad_vertex_data = {
        V2f{-1.0f, -1.0f}, V2f{1.0f, -1.0f}, V2f{-1.0f, 1.0f}, V2f{1.0f, 1.0f}};

    // POS buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVertexBufferObject);
    util_gl::CheckErrors("glBindBuffer POS");
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(V2f) * 4,
                 reinterpret_cast<GLvoid const*>(quad_vertex_data.data()),
                 GL_STATIC_DRAW);
    util_gl::CheckErrors("glBufferData POS");
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    util_gl::CheckErrors("glVertexAttribPointer POS");
    glEnableVertexAttribArray(0);
    util_gl::CheckErrors("glEnableVertexAttribArray POS");

    // Unbind VAO.
    glBindVertexArray(0);
    util_gl::CheckErrors("Unbind VAO");

    // Make texture
    glGenTextures(1, &m_textureObject);
    util_gl::CheckErrors("glGenTextures");
    EMLD_ASSERT(m_textureObject > 0, "Failed to create Texture");

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_textureObject);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D,  // target
                 0,              // mip-map level
                 GL_RGB32F,      // internal format
                 m_width,
                 m_height,
                 0,         // border
                 GL_RGB,    // format of incoming data
                 GL_FLOAT,  // type of incoming data
                 reinterpret_cast<void const*>(pixels));
    util_gl::CheckErrors("glTexImage2D");
    glBindTexture(GL_TEXTURE_2D, 0);

    static auto const* const vertex_source = R"(
        #version 150
        in vec2 position;
        out vec2 texcoord;
        void main() {
            gl_Position = vec4(position, 0.0, 1.0);
            texcoord = position * 0.5 + vec2(0.5, 0.5);
        }
    )";

    static auto const* const fragment_source = R"(
        #version 150
        in vec2 texcoord;
        out vec4 fragment_color;

        uniform sampler2D tex_image;

        void main() {
            fragment_color = vec4(texture(tex_image, texcoord).rgb, 1);
        }
    )";

    // Make program
    geep_glfw::Program::Bindings const vtx_bindings = {{0, "position"}};
    geep_glfw::Program::Bindings const frg_bindings = {{0, "fragment_color"}};
    m_program = std::make_unique<geep_glfw::Program>("Draw quad helper",
                                                    vertex_source,
                                                    "",
                                                    fragment_source,
                                                    vtx_bindings,
                                                    frg_bindings,
                                                    m_vertexArrayObject);

    m_program->use();
    (*m_program)(
        geep_glfw::Uniform{"tex_image", 0, geep_glfw::Uniform::kRequireError});
    m_program->setUniforms();
    m_program->unuse();
}

SlabDrawHelper::~SlabDrawHelper() {
    if (m_vertexArrayObject > 0) {
        glDeleteVertexArrays(1, &m_vertexArrayObject);
        m_vertexArrayObject = 0;
    }

    if (m_quadVertexBufferObject > 0) {
        glDeleteBuffers(1, &m_quadVertexBufferObject);
        m_quadVertexBufferObject = 0;
    }

    if (m_textureObject > 0) {
        glDeleteTextures(1, &m_textureObject);
        m_textureObject = 0;
    }
}

void SlabDrawHelper::update(V3f const* const pixels) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_textureObject);

    glTexSubImage2D(GL_TEXTURE_2D,  // target
                    0,              // mip-map level
                    0,              // x-offset
                    0,              // y-offset
                    m_width,
                    m_height,
                    GL_RGB,    // format of incoming data
                    GL_FLOAT,  // type of incoming data
                    reinterpret_cast<void const*>(pixels));
    util_gl::CheckErrors("glTexSubImage2D");
    glBindTexture(GL_TEXTURE_2D, 0);
}

void SlabDrawHelper::draw() const {
    // Bind the vertex array
    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray draw");

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_textureObject);

    m_program->use();
    m_program->setUniforms();

    // Draw the elements
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    util_gl::CheckErrors("glDrawArrays");

    m_program->unuse();

    // Unbind the vertex array
    glBindVertexArray(0);
    util_gl::CheckErrors("glBindVertexArray 0 draw");

    glBindTexture(GL_TEXTURE_2D, 0);
}

}  // namespace emerald::simple_sim_viewer