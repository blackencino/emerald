#pragma once

#include <sstream>

namespace emerald {
namespace util {

//------------------------------------------------------------------------------
// This is for aborting in a debug mode to make a more traceable stack.
void __EMLD_ASSERT_FAIL(char const* const msg) noexcept;

//-*****************************************************************************
// This macro will cause an abort.
#define EMLD_FAIL(TEXT)                                        \
    do {                                                       \
        std::stringstream sstr;                                \
        sstr << TEXT;                                          \
        sstr << "\nFile: " << __FILE__ << std::endl            \
             << "Line: " << __LINE__ << std::endl;             \
        emerald::util::__EMLD_ASSERT_FAIL(sstr.str().c_str()); \
    } while (0)

#define EMLD_ASSERT(COND, TEXT)           \
    do {                                  \
        if (!(COND)) { EMLD_FAIL(TEXT); } \
    } while (0)

#define EMLD_WARN(TEXT)                                      \
    do {                                                     \
        std::stringstream sstr;                              \
        sstr << TEXT;                                        \
        sstr << "\nFile: " << __FILE__ << std::endl          \
             << "Line: " << __LINE__ << std::endl;           \
        std::cerr << "WARNING: " << sstr.str() << std::endl; \
    } while (0)

}  // End namespace util
}  // End namespace emerald
