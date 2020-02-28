#include <emerald/simple_sim_viewer/mesh_draw_helper.h>

#include <emerald/geep_glfw/util_gl.h>
#include <emerald/util/assert.h>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
MeshDrawHelper::MeshDrawHelper(DeformType i_deformType,
                               size_t i_numTriangles,
                               size_t i_numVertices,
                               const V3ui* i_triIndices,
                               const V3f* i_vtxPosData,
                               const V3f* i_vtxNormData,
                               const V3f* i_vtxColData,
                               const V2f* i_vtxUvData)
  : m_deformType(i_deformType)
  , m_numTriangles(i_numTriangles)
  , m_numVertices(i_numVertices)
  , m_triIndices(i_triIndices)
  , m_vtxPosData(i_vtxPosData)
  , m_vtxNormData(i_vtxNormData)
  , m_vtxColData(i_vtxColData)
  , m_vtxUvData(i_vtxUvData)
  , m_vertexArrayObject(0)
  , m_posVboIdx(-1)
  , m_normVboIdx(-1)
  , m_colVboIdx(-1)
  , m_uvVboIdx(-1)
  , m_indicesVboIdx(-1) {
    //-*************************************************************************
    // OPENGL INIT
    //-*************************************************************************
    util_gl::CheckErrors("mesh draw helper init before anything");

    // Create and bind VAO
    glGenVertexArrays(1, &m_vertexArrayObject);
    util_gl::CheckErrors("glGenVertexArrays");
    EMLD_ASSERT(m_vertexArrayObject > 0, "Failed to create VAO");

    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray");

    // Figure out how many VBOs to make.
    EMLD_ASSERT(m_vtxPosData != nullptr && m_triIndices != nullptr,
                "Must have vertex and index data.");
    m_numVBOs = 2;
    if (m_vtxNormData) { ++m_numVBOs; }
    if (m_vtxColData) { ++m_numVBOs; }
    if (m_vtxUvData) { ++m_numVBOs; }

    // Create vertex buffers.
    glGenBuffers(m_numVBOs, m_vertexBuffers);
    util_gl::CheckErrors("glGenBuffers");
    EMLD_ASSERT(m_vertexBuffers[0] > 0, "Failed to create VBOs");

    GLenum vtxDataDyn = GL_STATIC_DRAW;
    if (i_deformType != kStaticDeform) { vtxDataDyn = GL_DYNAMIC_DRAW; }
    GLenum indexDyn = GL_STATIC_DRAW;
    if (i_deformType == kInconsistentDeform) { indexDyn = GL_DYNAMIC_DRAW; }

    // POS buffer
    int vboIdx = 0;
    m_posVboIdx = vboIdx;
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffers[vboIdx]);
    util_gl::CheckErrors("glBindBuffer POS");
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(V3f) * m_numVertices,
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
                     sizeof(V3f) * m_numVertices,
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
                     sizeof(V3f) * m_numVertices,
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
                     sizeof(V2f) * m_numVertices,
                     (const GLvoid*)m_vtxUvData,
                     vtxDataDyn);
        util_gl::CheckErrors("glBufferData UV");
        glVertexAttribPointer(vboIdx, 2, GL_FLOAT, GL_FALSE, 0, 0);
        util_gl::CheckErrors("glVertexAttribPointer UV");
        glEnableVertexAttribArray(vboIdx);
        util_gl::CheckErrors("glEnableVertexAttribArray UV");
    }

    // Indices buffer.
    ++vboIdx;
    m_indicesVboIdx = vboIdx;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexBuffers[vboIdx]);
    util_gl::CheckErrors("glBindBuffer INDICES");
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(V3ui) * m_numTriangles,
                 (const GLvoid*)m_triIndices,
                 indexDyn);
    util_gl::CheckErrors("glBufferData INDICES");

    EMLD_ASSERT(vboIdx == m_numVBOs - 1, "Mismatched vbo idx.");

    // Unbind VAO.
    glBindVertexArray(0);
    util_gl::CheckErrors("Unbind VAO");
}

//-*****************************************************************************
MeshDrawHelper::~MeshDrawHelper() {
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
// Static topology
void MeshDrawHelper::update(const V3f* i_vtxPosData,
                            const V3f* i_vtxNormData,
                            const V3f* i_vtxColData,
                            const V2f* i_vtxUvData) {
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
// Dynamic topology
void MeshDrawHelper::update(size_t i_numTriangles,
                            size_t i_numVertices,
                            const V3ui* i_triIndices,
                            const V3f* i_vtxPosData,
                            const V3f* i_vtxNormData,
                            const V3f* i_vtxColData,
                            const V2f* i_vtxUvData) {
    m_numTriangles = i_numTriangles;
    m_numVertices = i_numVertices;
    m_triIndices = i_triIndices;

    // Activate VAO
    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray");

    if (m_numTriangles > 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexBuffers[m_indicesVboIdx]);
        util_gl::CheckErrors("glBindBuffer INDICES");
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     sizeof(V3ui) * m_numTriangles,
                     (const GLvoid*)m_triIndices,
                     GL_DYNAMIC_DRAW);
        util_gl::CheckErrors("glBufferData INDICES");
    }

    // Update mesh stuff.
    update(i_vtxPosData, i_vtxNormData, i_vtxColData, i_vtxUvData);

    // Unbind
    glBindVertexArray(0);
}

//-*****************************************************************************
void MeshDrawHelper::draw(const GLCamera& i_cam) const {
    // Bind the vertex array
    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray draw");

    // Draw the elements
    glDrawElements(GL_TRIANGLES, 3 * m_numTriangles, GL_UNSIGNED_INT, 0);
    util_gl::CheckErrors("glDrawElements");

    // Unbind the vertex array
    glBindVertexArray(0);
    util_gl::CheckErrors("glBindVertexArray 0 draw");
}

//-*****************************************************************************
void MeshDrawHelper::draw() const {
    // Bind the vertex array
    glBindVertexArray(m_vertexArrayObject);
    util_gl::CheckErrors("glBindVertexArray draw");

    // Draw the elements
    glDrawElements(GL_TRIANGLES, 3 * m_numTriangles, GL_UNSIGNED_INT, 0);
    util_gl::CheckErrors("glDrawElements");

    // Unbind the vertex array
    glBindVertexArray(0);
    util_gl::CheckErrors("glBindVertexArray 0 draw");
}

}  // End namespace simple_sim_viewer
}  // End namespace emerald
