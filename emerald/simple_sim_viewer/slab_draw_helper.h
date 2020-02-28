#pragma once

#include <emerald/geep_glfw/program.h>
#include <emerald/simple_sim_viewer/foundation.h>

#include <memory>

namespace emerald::simple_sim_viewer {

class SlabDrawHelper {
public:
    SlabDrawHelper(V3f const* const pixels, int const width, int const height);
    ~SlabDrawHelper();

    void update(V3f const* const pixels);

    void draw() const;

private:
    int m_width = 0;
    int m_height = 0;
    GLuint m_vertexArrayObject = 0;
    GLuint m_quadVertexBufferObject = 0;
    GLuint m_textureObject = 0;
    std::unique_ptr<geep_glfw::Program> m_program;
};

}  // namespace emerald::simple_sim_viewer
