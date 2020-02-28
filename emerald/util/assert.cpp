#include <emerald/util/assert.h>

#include <cstdlib>
#include <iostream>

namespace emerald::util {

void __EMLD_ASSERT_FAIL(char const* const msg) noexcept {
    std::cerr << msg << std::endl;
    std::abort();
}

}  // namespace emerald::util
