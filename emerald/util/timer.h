#pragma once

#include <emerald/util/format.h>

#include <chrono>

namespace emerald {
namespace util {

//-*****************************************************************************
//! \brief Basic real-time stopwatch. Returns elapsed time in seconds.
class Timer {
public:
    //! Begins the timer and resets the elapsed time to zero
    void start() {
        m_stopped = -1.0;
        m_start = std::chrono::high_resolution_clock::now();
    }

    //! Creates a timer which is started by default
    Timer() {
        start();
    }

    //! Stops the timer and records elapsed time
    double stop() {
        m_stopped =
            std::chrono::duration<double>{
                std::chrono::high_resolution_clock::now() - m_start}
                .count();
        return m_stopped;
    }

    //! Returns the amount of time elapsed.
    double elapsed() const {
        if (m_stopped >= 0.0) { return m_stopped; }

        return std::chrono::duration<double>{
            std::chrono::high_resolution_clock::now() - m_start}
            .count();
    }

private:
    double m_stopped = -1.0;
    std::chrono::high_resolution_clock::time_point m_start;
};

struct Caliper {
    Caliper(char const* const msg)
      : message(msg)
      , start(std::chrono::high_resolution_clock::now()) {
    }
    ~Caliper() {
        auto const end = std::chrono::high_resolution_clock::now();
        fmt::print("{}: {} msec\n",
                   message,
                   std::chrono::duration<double>{end - start}.count() * 1000.0);
    }
    char const* const message = nullptr;
    std::chrono::high_resolution_clock::time_point start;
};

}  // End namespace util
}  // End namespace emerald
