#include <emerald/simple_sim_viewer/points_draw_helper.h>

#include <emerald/util/assert.h>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
PointsDrawHelper::PointsDrawHelper(bool i_dynamic,
                                   size_t i_numPoints,
                                   const V3f* i_vtxPosData,
                                   const V3f* i_vtxNormData,
                                   const V3f* i_vtxColData,
                                   const V2f* i_vtxUvData)
  : m_numPoints(i_numPoints)
  , m_vtxPosData(i_vtxPosData)
  , m_vtxNormData(i_vtxNormData)
  , m_vtxColData(i_vtxColData)
  , m_vtxUvData(i_vtxUvData)
  , m_vertexArrayObject(0)
  , m_posVboIdx(-1)
  , m_normVboIdx(-1)
  , m_colVboIdx(-1)
  , m_uvVboIdx(-1) {
    //-*************************************************************************
    // OPENGL INIT
    //-*************************************************************************
    util_gl::CheckErrors("point draw helper init before anything");

    // Create and bind VAO
    glGenVertexArrays(1, &m_vertexArrayObject);
    util_gl::CheckErrors("glGenVertexArrays");
    EMLD_ASSERT(m_vertexArrayObject > 0, "Failed to create VAO");

    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray");

    // Figure out how many VBOs to make.
    EMLD_ASSERT(m_vtxPosData != nullptr, "Must have vertex data.");
    m_numVBOs = 1;
    if (m_vtxNormData) { ++m_numVBOs; }
    if (m_vtxColData) { ++m_numVBOs; }
    if (m_vtxUvData) { ++m_numVBOs; }

    // Create vertex buffers.
    glGenBuffers(m_numVBOs, m_vertexBuffers);
    util_gl::CheckErrors("glGenBuffers");
    EMLD_ASSERT(m_vertexBuffers[0] > 0, "Failed to create VBOs");

    GLenum vtxDataDyn = GL_STATIC_DRAW;
    if (i_dynamic) { vtxDataDyn = GL_DYNAMIC_DRAW; }

    // POS buffer
    int vboIdx = 0;
    m_posVboIdx = vboIdx;
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffers[vboIdx]);
    util_gl::CheckErrors("glBindBuffer POS");
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(V3f) * m_numPoints,
                 (const GLvoid*)m_vtxPosData,
                 vtxDataDyn);
    util_gl::CheckErrors("glBufferData POS");
    glVertexAttribPointer(vboIdx, 3, GL_FLOAT, GL_FALSE, 0, 0);
    util_gl::CheckErrors("glVertexAttribPointer POS");
    glEnableVertexAttribArray(vboIdx);
    util_gl::CheckErrors("glEnableVertexAttribArray POS");

    // If NORM.
    m_normVboIdx = -1;
    if (m_vtxNormData) {
        ++vboIdx;
        m_normVboIdx = vboIdx;
        glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffers[vboIdx]);
        util_gl::CheckErrors("glBindBuffer NORM");
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(V3f) * m_numPoints,
                     (const GLvoid*)m_vtxNormData,
                     vtxDataDyn);
        util_gl::CheckErrors("glBufferData NORM");
        glVertexAttribPointer(vboIdx, 3, GL_FLOAT, GL_FALSE, 0, 0);
        util_gl::CheckErrors("glVertexAttribPointer NORM");
        glEnableVertexAttribArray(vboIdx);
        util_gl::CheckErrors("glEnableVertexAttribArray NORM");
    }

    // If COLOR.
    m_colVboIdx = -1;
    if (m_vtxColData) {
        ++vboIdx;
        m_colVboIdx = vboIdx;
        glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffers[vboIdx]);
        util_gl::CheckErrors("glBindBuffer COLOR");
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(V3f) * m_numPoints,
                     (const GLvoid*)m_vtxColData,
                     vtxDataDyn);
        util_gl::CheckErrors("glBufferData COLOR");
        glVertexAttribPointer(vboIdx, 3, GL_FLOAT, GL_FALSE, 0, 0);
        util_gl::CheckErrors("glVertexAttribPointer COLOR");
        glEnableVertexAttribArray(vboIdx);
        util_gl::CheckErrors("glEnableVertexAttribArray COLOR");
    }

    // If UVs.
    m_uvVboIdx = -1;
    if (m_vtxUvData) {
        ++vboIdx;
        m_uvVboIdx = vboIdx;
        glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffers[vboIdx]);
        util_gl::CheckErrors("glBindBuffer UV");
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(V2f) * m_numPoints,
                     (const GLvoid*)m_vtxUvData,
                     vtxDataDyn);
        util_gl::CheckErrors("glBufferData UV");
        glVertexAttribPointer(vboIdx, 2, GL_FLOAT, GL_FALSE, 0, 0);
        util_gl::CheckErrors("glVertexAttribPointer UV");
        glEnableVertexAttribArray(vboIdx);
        util_gl::CheckErrors("glEnableVertexAttribArray UV");
    }

    EMLD_ASSERT(vboIdx == m_numVBOs - 1, "Mismatched vbo idx.");

    // Unbind VAO.
    glBindVertexArray(0);
    util_gl::CheckErrors("Unbind VAO");
}

//-*****************************************************************************
PointsDrawHelper::~PointsDrawHelper() {
    if (m_vertexArrayObject > 0) {
        glDeleteVertexArrays(1, &m_vertexArrayObject);
        m_vertexArrayObject = 0;
    }

    if (m_numVBOs > 0 && m_vertexBuffers[0] > 0) {
        glDeleteBuffers(m_numVBOs, m_vertexBuffers);
        for (int i = 0; i < m_numVBOs; ++i) { m_vertexBuffers[i] = 0; }
    }
}

//-*****************************************************************************
void PointsDrawHelper::update(size_t i_numPoints,
                              const V3f* i_vtxPosData,
                              const V3f* i_vtxNormData,
                              const V3f* i_vtxColData,
                              const V2f* i_vtxUvData) {
    m_numPoints = i_numPoints;

    // Activate VAO
    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray");

    if (i_vtxPosData) {
        m_vtxPosData = i_vtxPosData;
        updateFloatVertexBuffer<V3f>(m_vtxPosData, m_posVboIdx);
    }
    if (i_vtxNormData) {
        m_vtxNormData = i_vtxNormData;
        updateFloatVertexBuffer<V3f>(m_vtxNormData, m_normVboIdx);
    }
    if (i_vtxColData) {
        m_vtxColData = i_vtxColData;
        updateFloatVertexBuffer<V3f>(m_vtxColData, m_colVboIdx);
    }
    if (i_vtxUvData) {
        m_vtxUvData = i_vtxUvData;
        updateFloatVertexBuffer<V2f>(m_vtxUvData, m_uvVboIdx);
    }

    // Unbind
    glBindVertexArray(0);
}

//-*****************************************************************************
void PointsDrawHelper::draw(const GLCamera& i_cam) const {
    // Bind the vertex array
    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray draw");

    // Draw the arrays
    glDrawArrays(GL_POINTS, 0, m_numPoints);
    util_gl::CheckErrors("glDrawArrays");

    // Unbind the vertex array
    glBindVertexArray(0);
    util_gl::CheckErrors("glBindVertexArray 0 draw");
}

}  // End namespace simple_sim_viewer
}  // End namespace emerald
