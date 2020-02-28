#include <emerald/util/vector_util.h>

#include <fmt/format.h>

namespace emerald::util {

std::string FormatHashKey(VectorHashKey const key) {
    return fmt::format("{:#016x}--{:#016x}", key[0], key[1]);
}

}  // End namespace emerald::util
