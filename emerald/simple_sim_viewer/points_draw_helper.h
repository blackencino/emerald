#pragma once

#include <emerald/simple_sim_viewer/foundation.h>
#include <emerald/simple_sim_viewer/gl_camera.h>

#include <emerald/geep_glfw/util_gl.h>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
class PointsDrawHelper {
public:
    PointsDrawHelper(bool i_dynamic,
                     size_t i_numPoints,
                     const V3f* i_vtxPosData,
                     const V3f* i_vtxNormData,
                     const V3f* i_vtxColData,
                     const V2f* i_vtxUvData);

    virtual ~PointsDrawHelper();

    // Update the point data
    void update(size_t i_numPoints,
                const V3f* i_vtxPosData,
                const V3f* i_vtxNormData,
                const V3f* i_vtxColData,
                const V2f* i_vtxUvData);

    // Draw.
    void draw(const GLCamera& i_cam) const;

    GLint posVboIdx() const {
        return m_posVboIdx;
    }
    GLint normVboIdx() const {
        return m_normVboIdx;
    }
    GLint colVboIdx() const {
        return m_colVboIdx;
    }
    GLint uvVboIdx() const {
        return m_uvVboIdx;
    }

    GLuint vertexArrayObject() const {
        return m_vertexArrayObject;
    }

protected:
    template <typename T>
    void updateFloatVertexBuffer(const T* i_vtxData,
                                 int i_vboIdx,
                                 bool const resize = true) {
        if (i_vboIdx < 0 || i_vtxData == nullptr || m_numPoints == 0) {
            return;
        }

        glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffers[i_vboIdx]);
        util_gl::CheckErrors("glBindBuffer");
        if (resize) {
            glBufferData(GL_ARRAY_BUFFER,
                         sizeof(T) * m_numPoints,
                         (const GLvoid*)i_vtxData,
                         GL_DYNAMIC_DRAW);
            util_gl::CheckErrors("glBufferData");
        } else {
            glBufferSubData(GL_ARRAY_BUFFER,
                            0,
                            sizeof(T) * m_numPoints,
                            (const GLvoid*)i_vtxData);
            util_gl::CheckErrors("glBufferSubData");
        }
    }

    size_t m_numPoints;
    const V3f* m_vtxPosData;
    const V3f* m_vtxNormData;
    const V3f* m_vtxColData;
    const V2f* m_vtxUvData;

    // The VAO and VBOs
    GLuint m_vertexArrayObject;
    // At most we have pos, norm, col, uv
    GLuint m_vertexBuffers[4];
    int m_numVBOs;
    GLint m_posVboIdx;
    GLint m_normVboIdx;
    GLint m_colVboIdx;
    GLint m_uvVboIdx;
};

}  // End namespace simple_sim_viewer
}  // End namespace emerald
