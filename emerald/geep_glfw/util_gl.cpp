#include <emerald/geep_glfw/util_gl.h>

#include <emerald/geep_glfw/foundation.h>
#include <emerald/util/assert.h>

#include <iostream>

namespace emerald {
namespace geep_glfw {
namespace util_gl {

//-*****************************************************************************
void Init(bool const experimental) {
    CheckErrors("GeepGLFW::init before anything");

// On Mac, GLEW stuff is not necessary.
#ifndef PLATFORM_DARWIN
    glewExperimental = experimental ? GL_TRUE : GL_FALSE;
    glewInit();
#endif
    // Reset errors.
    glGetError();

    std::cout << "OPEN GL VERSION: " << glGetString(GL_VERSION) << std::endl;

    CheckErrors("GeepGLFW::init glGetString");
}

//-*****************************************************************************
void CheckErrors(std::string const& label) {
    auto const errCode = glGetError();

    EMLD_ASSERT(errCode == GL_NO_ERROR,
                "OpenGL Error: "
                    << "Code = " << static_cast<int>(errCode)
                    << " ( Label: " << label << " )");
}

//-*****************************************************************************
void CheckFramebuffer() {
    auto const status =
        static_cast<GLenum>(glCheckFramebufferStatus(GL_FRAMEBUFFER));
    switch (status) {
    case GL_FRAMEBUFFER_COMPLETE: return;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        EMLD_FAIL("Framebuffer incomplete, incomplete attachment");
        break;
    case GL_FRAMEBUFFER_UNSUPPORTED:
        EMLD_FAIL("Unsupported framebuffer format");
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        EMLD_FAIL("Framebuffer incomplete, missing attachment");
        break;
    // case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
    //    EMLD_FAIL(
    //        "Framebuffer incomplete, attached images "
    //        "must have same dimensions" );
    //    break;
    // case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
    //    EMLD_FAIL(
    //        "Framebuffer incomplete, attached images "
    //        "must have same format" );
    //    break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        EMLD_FAIL("Framebuffer incomplete, missing draw buffer");
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        EMLD_FAIL("Framebuffer incomplete, missing read buffer");
        break;
    }
    EMLD_FAIL("Unknown GL Framebuffer error");
}

}  // End namespace util_gl
}  // End namespace geep_glfw
}  // End namespace emerald
