#pragma once

#include <emerald/util/foundation.h>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <array>
#include <type_traits>

template <typename T>
struct fmt::formatter<Imath::Vec2<T>> {
    // Presentation format: 'f' - fixed, 'e' - exponential.
    char presentation = 'f';

    // Parses format specifications of the form ['f' | 'e'].
    constexpr auto parse(format_parse_context& ctx) {
        // [ctx.begin(), ctx.end()) is a character range that contains a part of
        // the format string starting from the format specifications to be
        // parsed, e.g. in
        //
        //   fmt::format("{:f} - point of interest", point{1, 2});
        //
        // the range will contain "f} - point of interest". The formatter should
        // parse specifiers until '}' or the end of the range. In this example
        // the formatter should parse the 'f' specifier and return an iterator
        // pointing to '}'.

        // Parse the presentation format and store it in the formatter:
        auto it = ctx.begin();
        auto const end = ctx.end();
        if constexpr (std::is_floating_point_v<T>) {
            if (it != end && (*it == 'f' || *it == 'e')) {
                presentation = *it++;
            }
        }

        // Check if reached the end of the range:
        if (it != end && *it != '}') { throw format_error("invalid format"); }

        // Return an iterator past the end of the parsed range:
        return it;
    }

    // Formats the point p using the parsed format specification (presentation)
    // stored in this formatter.
    template <typename FormatContext>
    auto format(Imath::Vec2<T> const& p, FormatContext& ctx) {
        // ctx.out() is an output iterator to write to.
        if constexpr (std::is_floating_point_v<T>) {
            return format_to(
                ctx.out(),
                presentation == 'f' ? "({:f}, {:f})" : "({:e}, {:e})",
                p.x,
                p.y);
        } else {
            return format_to(ctx.out(), "({}, {})", p.x, p.y);
        }
    }
};

template <typename T>
struct fmt::formatter<Imath::Vec3<T>> {
    // Presentation format: 'f' - fixed, 'e' - exponential.
    char presentation = 'f';

    // Parses format specifications of the form ['f' | 'e'].
    constexpr auto parse(format_parse_context& ctx) {
        // [ctx.begin(), ctx.end()) is a character range that contains a part of
        // the format string starting from the format specifications to be
        // parsed, e.g. in
        //
        //   fmt::format("{:f} - point of interest", point{1, 2});
        //
        // the range will contain "f} - point of interest". The formatter should
        // parse specifiers until '}' or the end of the range. In this example
        // the formatter should parse the 'f' specifier and return an iterator
        // pointing to '}'.

        // Parse the presentation format and store it in the formatter:
        auto it = ctx.begin();
        auto const end = ctx.end();
        if constexpr (std::is_floating_point_v<T>) {
            if (it != end && (*it == 'f' || *it == 'e')) {
                presentation = *it++;
            }
        }

        // Check if reached the end of the range:
        if (it != end && *it != '}') { throw format_error("invalid format"); }

        // Return an iterator past the end of the parsed range:
        return it;
    }

    // Formats the point p using the parsed format specification (presentation)
    // stored in this formatter.
    template <typename FormatContext>
    auto format(Imath::Vec3<T> const& p, FormatContext& ctx) {
        // ctx.out() is an output iterator to write to.
        if constexpr (std::is_floating_point_v<T>) {
            return format_to(ctx.out(),
                             presentation == 'f' ? "({:f}, {:f}, {:f})"
                                                 : "({:e}, {:e}, {:e})",
                             p.x,
                             p.y,
                             p.z);
        } else {
            return format_to(ctx.out(), "({}, {}, {})", p.x, p.y, p.z);
        }
    }
};
