#include <emerald/geep_glfw/program.h>

#include <emerald/geep_glfw/util_gl.h>
#include <emerald/util/assert.h>

#include <iostream>

namespace emerald::geep_glfw {

//-*****************************************************************************
//-*****************************************************************************
// UNIFORM
//-*****************************************************************************
//-*****************************************************************************

Uniform::Uniform()
  : m_name("")
  , m_type(kUniformFloat)
  , m_required(kRequireOptional) {
    m_size[0] = 0;
    m_size[1] = 0;
}

Uniform::Uniform(const Uniform& i_copy)
  : m_name(i_copy.m_name)
  , m_type(i_copy.m_type)
  , m_required(i_copy.m_required) {
    m_size[0] = i_copy.m_size[0];
    m_size[1] = i_copy.m_size[1];
    for (int i = 0; i < 16; ++i) { m_data.f[i] = i_copy.m_data.f[i]; }
}

Uniform& Uniform::operator=(const Uniform& i_copy) {
    m_name = i_copy.m_name;
    m_type = i_copy.m_type;
    m_required = i_copy.m_required;

    m_size[0] = i_copy.m_size[0];
    m_size[1] = i_copy.m_size[1];
    for (int i = 0; i < 16; ++i) { m_data.f[i] = i_copy.m_data.f[i]; }
    return *this;
}

const std::string& Uniform::name() const {
    return m_name;
}

//-*************************************************************************
// FLOATS
//-*************************************************************************

// 1 float
Uniform::Uniform(const std::string& i_name, float i_f, Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_f;
    m_size[0] = 1;
    m_size[1] = 0;
}

// 2 floats
Uniform::Uniform(const std::string& i_name,
                 float i_f0,
                 float i_f1,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_f0;
    m_data.f[1] = i_f1;
    m_size[0] = 2;
    m_size[1] = 0;
}

// V2f
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec2<float>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_v[0];
    m_data.f[1] = i_v[1];
    m_size[0] = 2;
    m_size[1] = 0;
}

// V2d
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec2<double>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = GLfloat(i_v[0]);
    m_data.f[1] = GLfloat(i_v[1]);
    m_size[0] = 2;
    m_size[1] = 0;
}

// 3 floats
Uniform::Uniform(const std::string& i_name,
                 float i_f0,
                 float i_f1,
                 float i_f2,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_f0;
    m_data.f[1] = i_f1;
    m_data.f[2] = i_f2;
    m_size[0] = 3;
    m_size[1] = 0;
}

// V3f
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec3<float>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_v[0];
    m_data.f[1] = i_v[1];
    m_data.f[2] = i_v[2];
    m_size[0] = 3;
    m_size[1] = 0;
}

// V3d
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec3<double>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = GLfloat(i_v[0]);
    m_data.f[1] = GLfloat(i_v[1]);
    m_data.f[2] = GLfloat(i_v[2]);
    m_size[0] = 3;
    m_size[1] = 0;
}

// 4 floats
Uniform::Uniform(const std::string& i_name,
                 float i_f0,
                 float i_f1,
                 float i_f2,
                 float i_f3,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_f0;
    m_data.f[1] = i_f1;
    m_data.f[2] = i_f2;
    m_data.f[3] = i_f3;
    m_size[0] = 4;
    m_size[1] = 0;
}

// V4f
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec4<float>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = i_v[0];
    m_data.f[1] = i_v[1];
    m_data.f[2] = i_v[2];
    m_data.f[3] = i_v[3];
    m_size[0] = 4;
    m_size[1] = 0;
}

// V4d
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec4<double>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformFloat)
  , m_required(i_req) {
    m_data.f[0] = GLfloat(i_v[0]);
    m_data.f[1] = GLfloat(i_v[1]);
    m_data.f[2] = GLfloat(i_v[2]);
    m_data.f[3] = GLfloat(i_v[3]);
    m_size[0] = 4;
    m_size[1] = 0;
}

//-*************************************************************************
// INTS
//-*************************************************************************

// 1 int
Uniform::Uniform(const std::string& i_name, int i_i, Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformInt)
  , m_required(i_req) {
    m_data.i[0] = i_i;
    m_size[0] = 1;
    m_size[1] = 0;
}

// 2 ints
Uniform::Uniform(const std::string& i_name,
                 int i_i0,
                 int i_i1,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformInt)
  , m_required(i_req) {
    m_data.i[0] = i_i0;
    m_data.i[1] = i_i1;
    m_size[0] = 2;
    m_size[1] = 0;
}

// V2i
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec2<int>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformInt)
  , m_required(i_req) {
    m_data.i[0] = i_v[0];
    m_data.i[1] = i_v[1];
    m_size[0] = 2;
    m_size[1] = 0;
}

// 3 ints
Uniform::Uniform(
    const std::string& i_name, int i_i0, int i_i1, int i_i2, Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformInt)
  , m_required(i_req) {
    m_data.i[0] = i_i0;
    m_data.i[1] = i_i1;
    m_data.i[2] = i_i2;
    m_size[0] = 3;
    m_size[1] = 0;
}

// V3i
Uniform::Uniform(const std::string& i_name,
                 const Imath::Vec3<int>& i_v,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformInt)
  , m_required(i_req) {
    m_data.i[0] = i_v[0];
    m_data.i[1] = i_v[1];
    m_data.i[2] = i_v[2];
    m_size[0] = 3;
    m_size[1] = 0;
}

// 4 ints
Uniform::Uniform(const std::string& i_name,
                 int i_i0,
                 int i_i1,
                 int i_i2,
                 int i_i3,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformInt)
  , m_required(i_req) {
    m_data.i[0] = i_i0;
    m_data.i[1] = i_i1;
    m_data.i[2] = i_i2;
    m_data.i[3] = i_i3;
    m_size[0] = 4;
    m_size[1] = 0;
}

//-*************************************************************************
// UNSIGNED INTS
//-*************************************************************************

// 1 unsigned int
Uniform::Uniform(const std::string& i_name,
                 unsigned int i_ui,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformUint)
  , m_required(i_req) {
    m_data.ui[0] = i_ui;
    m_size[0] = 1;
    m_size[1] = 0;
}

// 2 unsigned ints
Uniform::Uniform(const std::string& i_name,
                 unsigned int i_ui0,
                 unsigned int i_ui1,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformUint)
  , m_required(i_req) {
    m_data.ui[0] = i_ui0;
    m_data.ui[1] = i_ui1;
    m_size[0] = 2;
    m_size[1] = 0;
}

// 3 unsigned ints
Uniform::Uniform(const std::string& i_name,
                 unsigned int i_ui0,
                 unsigned int i_ui1,
                 unsigned int i_ui2,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformUint)
  , m_required(i_req) {
    m_data.ui[0] = i_ui0;
    m_data.ui[1] = i_ui1;
    m_data.ui[2] = i_ui2;
    m_size[0] = 3;
    m_size[1] = 0;
}

// 4 unsigned ints
Uniform::Uniform(const std::string& i_name,
                 unsigned int i_ui0,
                 unsigned int i_ui1,
                 unsigned int i_ui2,
                 unsigned int i_ui3,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformUint)
  , m_required(i_req) {
    m_data.ui[0] = i_ui0;
    m_data.ui[1] = i_ui1;
    m_data.ui[2] = i_ui2;
    m_data.ui[3] = i_ui3;
    m_size[0] = 4;
    m_size[1] = 0;
}

// 3x3 matrix
Uniform::Uniform(const std::string& i_name,
                 const Imath::Matrix33<float>& i_mat,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformMatrix)
  , m_required(i_req) {
    const float* m = &(i_mat[0][0]);
    for (int i = 0; i < 9; ++i) { m_data.f[i] = float(m[i]); }
    m_size[0] = 3;
    m_size[1] = 3;
}

// 3x3 matrix
Uniform::Uniform(const std::string& i_name,
                 const Imath::Matrix33<double>& i_mat,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformMatrix)
  , m_required(i_req) {
    const double* m = &(i_mat[0][0]);
    for (int i = 0; i < 9; ++i) { m_data.f[i] = float(m[i]); }
    m_size[0] = 3;
    m_size[1] = 3;
}

// 4x4 matrix
Uniform::Uniform(const std::string& i_name,
                 const Imath::Matrix44<float>& i_mat,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformMatrix)
  , m_required(i_req) {
    const float* m = &(i_mat[0][0]);
    for (int i = 0; i < 16; ++i) { m_data.f[i] = float(m[i]); }
    m_size[0] = 4;
    m_size[1] = 4;
}

Uniform::Uniform(const std::string& i_name,
                 const Imath::Matrix44<double>& i_mat,
                 Requirement i_req)
  : m_name(i_name)
  , m_type(kUniformMatrix)
  , m_required(i_req) {
    const double* m = &(i_mat[0][0]);
    for (int i = 0; i < 16; ++i) { m_data.f[i] = float(m[i]); }
    m_size[0] = 4;
    m_size[1] = 4;
}

void Uniform::set(GLuint i_progId) const {
    // Get out of nonexistent uniforms.
    if (m_size[0] < 1 || m_name == "") { return; }

    // Find the uniform location, and complain if necessary.
    GLint loc = glGetUniformLocation(i_progId, m_name.c_str());
    if (loc < 0) {
        switch (m_required) {
        default:
        case kRequireOptional: break;
        case kRequireWarning:
            EMLD_WARN("WARNING: Couldn't find uniform: " << m_name
                                                         << " in program.");
            break;
        case kRequireError:
            EMLD_FAIL("Couldn't find uniform: " << m_name << " in program.");
            break;
        }

        // No uniform location, bug out.
        return;
    }

    // Set the uniform by type.
    switch (m_type) {
    case kUniformFloat:
        switch (m_size[0]) {
        default:
        case 1: glUniform1f(loc, (GLfloat)m_data.f[0]); break;
        case 2: glUniform2fv(loc, 1, (const GLfloat*)m_data.f); break;
        case 3: glUniform3fv(loc, 1, (const GLfloat*)m_data.f); break;
        case 4: glUniform4fv(loc, 1, (const GLfloat*)m_data.f); break;
        }
        break;
    case kUniformInt:
        switch (m_size[0]) {
        default:
        case 1: glUniform1i(loc, (GLint)m_data.i[0]); break;
        case 2: glUniform2iv(loc, 1, (const GLint*)m_data.i); break;
        case 3: glUniform3iv(loc, 1, (const GLint*)m_data.i); break;
        case 4: glUniform4iv(loc, 1, (const GLint*)m_data.i); break;
        }
        break;
    case kUniformUint:
        switch (m_size[0]) {
        default:
        case 1: glUniform1ui(loc, (GLuint)m_data.ui[0]); break;
        case 2: glUniform2uiv(loc, 1, (const GLuint*)m_data.ui); break;
        case 3: glUniform3uiv(loc, 1, (const GLuint*)m_data.ui); break;
        case 4: glUniform4uiv(loc, 1, (const GLuint*)m_data.ui); break;
        }
        break;
    case kUniformMatrix:
        switch (m_size[0]) {
        default:
        case 3:
            glUniformMatrix3fv(loc, 1, GL_FALSE, (const GLfloat*)m_data.f);
            break;
        case 4:
            glUniformMatrix4fv(loc, 1, GL_FALSE, (const GLfloat*)m_data.f);
            break;
        };
        break;
    };
    util_gl::CheckErrors("glUniform setting");
}

//-*****************************************************************************
//-*****************************************************************************
// PROGRAM
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
Program::Program(const std::string& i_name,
                 const std::string& i_vtx,
                 const std::string& i_geom,
                 const std::string& i_frg,
                 const Bindings& i_vtxBindingsIn,
                 const Bindings& i_frgBindingsOut,
                 GLuint i_vertexArrayObject)
  : m_name(i_name) {
    // Bind the vertex array object
    if (i_vertexArrayObject > 0) {
        glBindVertexArray(i_vertexArrayObject);
        util_gl::CheckErrors("glBindVertexArray");
    }

    // Do the program creation.
    m_progId = glCreateProgram();
    if (m_progId == 0) {
        EMLD_FAIL("Couldn't allocate GLSL program: " << m_name);
    }
    std::cout << "Created GLSL program" << std::endl;

    //-*************************************************************************
    // VERTEX SHADER
    //-*************************************************************************
    m_vtxId = 0;
    std::vector<std::string> vtxSources;
    vtxSources.push_back(i_vtx);
    if (vtxSources.size() > 0) {
        m_vtxId = initShader(m_name + "::vertex", GL_VERTEX_SHADER, vtxSources);
        glAttachShader(m_progId, m_vtxId);
    }
    std::cout << "Created GLSL vertex shader" << std::endl;

    //-*************************************************************************
    // GEOMETRY SHADER (they're optional!)
    //-*************************************************************************
    m_geomId = 0;
    if (i_geom != "") {
        std::vector<std::string> geomSources;
        geomSources.push_back(i_geom);
        if (geomSources.size() > 0) {
            m_geomId = initShader(
                m_name + "::geometry", GL_GEOMETRY_SHADER, geomSources);
            glAttachShader(m_progId, m_geomId);
        }
        std::cout << "Created GLSL geometry shader" << std::endl;
    }

    //-*************************************************************************
    // FRAGMENT SHADER
    //-*************************************************************************
    m_frgId = 0;
    std::vector<std::string> frgSources;
    frgSources.push_back(i_frg);
    if (frgSources.size() > 0) {
        m_frgId =
            initShader(m_name + "::fragment", GL_FRAGMENT_SHADER, frgSources);
        glAttachShader(m_progId, m_frgId);
    }
    std::cout << "Created GLSL fragment shader" << std::endl;

    //-*************************************************************************
    // Bind vertex attributes
    //-*************************************************************************
    for (Bindings::const_iterator biter = i_vtxBindingsIn.begin();
         biter != i_vtxBindingsIn.end();
         ++biter) {
        glBindAttribLocation(
            m_progId, (*biter).first, (const GLchar*)((*biter).second.c_str()));
        util_gl::CheckErrors("glBindAttribLocation");
    }

    //-*************************************************************************
    // Bind fragment data
    //-*************************************************************************
    for (Bindings::const_iterator biter = i_frgBindingsOut.begin();
         biter != i_frgBindingsOut.end();
         ++biter) {
        glBindFragDataLocation(
            m_progId, (*biter).first, (const GLchar*)((*biter).second.c_str()));
    }

    //-*************************************************************************
    // Link program
    //-*************************************************************************
    glLinkProgram(m_progId);
    std::cout << "Linked GLSL program." << std::endl;

    GLint linked = 0;
    glGetProgramiv(m_progId, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        GLint length = 0;
        glGetProgramiv(m_progId, GL_INFO_LOG_LENGTH, &length);

        std::vector<GLchar> log(length + 1);
        glGetProgramInfoLog(m_progId, length, &length, &(log[0]));
        std::string logStr = (const std::string&)&(log[0]);
        EMLD_FAIL("Linking error in program: " << m_name << std::endl
                                               << logStr);
    }

    //-*************************************************************************
    // Check vertex attribute bindings
    //-*************************************************************************
    for (Bindings::const_iterator biter = i_vtxBindingsIn.begin();
         biter != i_vtxBindingsIn.end();
         ++biter) {
        GLint result = glGetAttribLocation(
            m_progId, (const GLchar*)((*biter).second.c_str()));
        util_gl::CheckErrors("glGetAttribLocation");
        EMLD_ASSERT(result == (*biter).first,
                    "Did not successfully bind attribute: "
                        << (*biter).second << ", got result: " << result
                        << ", but wanted: " << (*biter).first);
    }

    GLint validate = 0;
    glValidateProgram(m_progId);
    glGetProgramiv(m_progId, GL_VALIDATE_STATUS, &validate);
    if (validate != GL_TRUE) {
        GLint length = 0;
        glGetProgramiv(m_progId, GL_INFO_LOG_LENGTH, &length);

        std::vector<GLchar> log(length + 1);
        glGetProgramInfoLog(m_progId, length, &length, &(log[0]));
        std::string logStr = (const std::string&)&(log[0]);

        EMLD_FAIL("Given vertex/fragment program: "
                  << m_name << " won't run on this hardware" << std::endl
                  << logStr);
    }
    std::cout << "Validated GLSL program." << std::endl;

    // Bind the vertex array object
    if (i_vertexArrayObject > 0) {
        glBindVertexArray(0);
        util_gl::CheckErrors("glBindVertexArray");
    }
}

//-*****************************************************************************
Program::~Program() {
    if (m_progId > 0) {
        glDeleteProgram(m_progId);
        m_progId = 0;
    }

    if (m_vtxId > 0) {
        glDeleteShader(m_vtxId);
        m_vtxId = 0;
    }

    if (m_frgId > 0) {
        glDeleteShader(m_frgId);
        m_frgId = 0;
    }

    if (m_geomId > 0) {
        glDeleteShader(m_geomId);
        m_geomId = 0;
    }
}

//-*****************************************************************************
// Set uniforms on the program.
void Program::operator()(const Uniform& i_uniform) {
    m_uniforms[i_uniform.name()] = i_uniform;
}

//-*****************************************************************************
void Program::use() const {
    EMLD_ASSERT(m_progId > 0, "Cannot use program 0");

    // Use the program
    glUseProgram(m_progId);

    setUniforms();
}

//-*****************************************************************************
void Program::unuse() const {
    EMLD_ASSERT(m_progId > 0, "Cannot unuse program 0");

    // Unuse program
    glUseProgram(0);
}

//-*****************************************************************************
void Program::setUniforms() const {
    // Set the uniforms.
    for (Uniforms::const_iterator uiter = m_uniforms.begin();
         uiter != m_uniforms.end();
         ++uiter) {
        (*uiter).second.set(m_progId);
    }
}

//-*****************************************************************************
GLuint Program::initShader(const std::string& i_shaderName,
                           GLenum i_type,
                           const std::vector<std::string>& i_sources) {
    GLuint id = 0;

    const GLchar* shaderSources[32];
    GLsizei numSources = i_sources.size();
    assert(numSources > 0);

    if (numSources > 32) {
        EMLD_FAIL("Can't compile shader: " << i_shaderName << std::endl
                                           << "Too many shader sources: "
                                           << numSources << ". Max = 32");
    }

    std::vector<std::string> newSources(i_sources.size());

    for (int i = 0; i < numSources; ++i) {
        shaderSources[i] = (const GLchar*)(i_sources[i].c_str());
    }

    for (int i = numSources; i < 32; ++i) { shaderSources[i] = nullptr; }

    id = glCreateShader(i_type);
    if (id == 0) { EMLD_FAIL("Could not create shader: " << i_shaderName); }

    glShaderSource(id, numSources, shaderSources, nullptr);
    glCompileShader(id);
    GLint compiled = 0;
    glGetShaderiv(id, GL_COMPILE_STATUS, &compiled);
    if (compiled != GL_TRUE) {
        GLint length = 0;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        std::vector<GLchar> log(length + 1);
        glGetShaderInfoLog(id, length, &length, &(log[0]));
        std::string strLog = (const char*)&(log[0]);
        EMLD_FAIL("Compilation error in shader: " << i_shaderName << std::endl
                                                  << strLog);
    }

    return id;
}

}  // End namespace emerald::geep_glfw
