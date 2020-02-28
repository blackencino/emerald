#pragma once

#include <emerald/geep_glfw/foundation.h>

#include <OpenEXR/ImathMatrix.h>
#include <OpenEXR/ImathVec.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace emerald {
namespace geep_glfw {

//-*****************************************************************************
class Uniform {
public:
    enum Type { kUniformFloat, kUniformInt, kUniformUint, kUniformMatrix };

    enum Requirement { kRequireOptional, kRequireWarning, kRequireError };

    Uniform();

    Uniform(const Uniform& i_copy);

    Uniform& operator=(const Uniform& i_copy);

    const std::string& name() const;

    //-*************************************************************************
    // FLOATS
    //-*************************************************************************

    Uniform(const std::string& i_name,
            float i_f,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            float i_f0,
            float i_f1,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec2<float>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec2<double>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            float i_f0,
            float i_f1,
            float i_f2,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec3<float>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec3<double>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            float i_f0,
            float i_f1,
            float i_f2,
            float i_f3,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec4<float>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec4<double>& i_v,
            Requirement i_req = kRequireOptional);

    //-*************************************************************************
    // INTS
    //-*************************************************************************

    Uniform(const std::string& i_name,
            int i_i,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            int i_i0,
            int i_i1,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec2<int>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            int i_i0,
            int i_i1,
            int i_i2,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Vec3<int>& i_v,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            int i_i0,
            int i_i1,
            int i_i2,
            int i_i3,
            Requirement i_req = kRequireOptional);

    //-*************************************************************************
    // UNSIGNED INTS
    //-*************************************************************************

    Uniform(const std::string& i_name,
            unsigned int i_ui,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            unsigned int i_ui0,
            unsigned int i_ui1,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            unsigned int i_ui0,
            unsigned int i_ui1,
            unsigned int i_ui2,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            unsigned int i_ui0,
            unsigned int i_ui1,
            unsigned int i_ui2,
            unsigned int i_ui3,
            Requirement i_req = kRequireOptional);

    //-*************************************************************************
    // MATRICES
    //-*************************************************************************

    Uniform(const std::string& i_name,
            const Imath::Matrix33<float>& i_mat,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Matrix33<double>& i_mat,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Matrix44<float>& i_mat,
            Requirement i_req = kRequireOptional);

    Uniform(const std::string& i_name,
            const Imath::Matrix44<double>& i_mat,
            Requirement i_req = kRequireOptional);

    //-*************************************************************************
    // SET
    //-*************************************************************************
    void set(GLuint i_progId) const;

protected:
    std::string m_name;
    union {
        GLfloat f[16];
        GLint i[16];
        GLuint ui[16];
    } m_data;
    Type m_type = kUniformFloat;
    int m_size[2];
    Requirement m_required = kRequireOptional;
};

//-*****************************************************************************
class Program {
public:
    typedef std::pair<GLuint, std::string> Binding;
    typedef std::vector<Binding> Bindings;

    // The reason the Program constructor takes a vertex array object
    // specifier is because some OpenGL drivers (OSX) require a VAO to be
    // bound in order to validate a program, and without one bound,
    // glValidateProgram will return an error. This constructor will bind
    // and unbind the VAO if it is greater than zero.
    Program(const std::string& i_name,
            const std::string& i_vtx,
            const std::string& i_geom,
            const std::string& i_frg,
            const Bindings& i_vtxBindingsIn,
            const Bindings& i_frgBindingsOut,
            GLuint i_vertexArrayObject);
    ~Program();

    void operator()(const Uniform& i_uniform);

    void use() const;
    void unuse() const;

    void setUniforms() const;

    GLuint id() const {
        return m_progId;
    }
    GLuint vertexShaderId() const {
        return m_vtxId;
    }
    GLuint geometryShaderId() const {
        return m_geomId;
    }
    GLuint fragmentShaderId() const {
        return m_frgId;
    }

protected:
    GLuint initShader(const std::string& i_shaderName,
                      GLenum i_type,
                      const std::vector<std::string>& i_sources);

    std::string m_name;

    GLuint m_progId = 0;
    GLuint m_vtxId = 0;
    GLuint m_geomId = 0;
    GLuint m_frgId = 0;

    typedef std::map<std::string, Uniform> Uniforms;
    Uniforms m_uniforms;
};

}  // End namespace geep_glfw
}  // End namespace emerald
