//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

//-*****************************************************************************
//-*****************************************************************************
// C++ 11 FOR SURE
//-*****************************************************************************
//-*****************************************************************************

#define SPH_USE_CXX11 1
#define SPH_STATIC_CONSTEXPR static constexpr

#include <cstdint>
#include <random>

typedef uint64_t seed_type;
typedef std::linear_congruential_engine<uint64_t,
                                        uint64_t(0xDEECE66DUL) |
                                          (uint64_t(0x5) << 32),
                                        0xB,
                                        uint64_t(1) << 48>
  Rand48_Engine;
#define SPH_UNIFORM_REAL_DISTRIBUTION std::uniform_real_distribution

#if SPH_USE_CXX11

//-*****************************************************************************
// Let's see whether we can use fancy cmath stuff.
void testFancyCmathStuff() {
    // Gamma.
    std::cout << "Gamma of 0.5 = " << std::tgamma(0.5) << std::endl;

    // Lround.
    std::cout << "Long round of 191381.1356: " << std::lround(191381.1356)
              << std::endl;

    // Erf & Erfc
    std::cout << "Erf of 18.881: " << std::erf(18.881) << std::endl
              << "Erfc of 18.881: " << std::erfc(18.881) << std::endl;

    // Random stuff
    Rand48_Engine rnd;
    rnd.seed(12345);
    SPH_UNIFORM_REAL_DISTRIBUTION<double> dist(0.0, 1.0);
    std::cout << "First draw from Rand48: " << dist(rnd) << std::endl;
}

//-*****************************************************************************
template <typename ITER, typename F>
void applyToAll(ITER i_begin, ITER i_end, F i_func) {
    int N = static_cast<int>(i_end - i_begin);
    int* ptr = &(*i_begin);
    for (int i = 0; i < N; ++i) { i_func(*(ptr + i), i); }
}

#endif

//-*****************************************************************************
struct Base {
    Base()
      : a(0.0f) {
    }
    float a;

    float A() const {
        return a;
    }
};

//-*****************************************************************************
template <typename T>
struct Derived : public Base {
    Derived()
      : Base()
      , b(T(0)) {
    }

    T b;
    // using Base::A;
    T B() const {
        return b;
    }
    T C() const {
        return b + A();
    }
};

//-*****************************************************************************
template <typename T>
struct Derived2 : public Derived<T> {
    Derived2()
      : Derived<T>()
      , c(T(0)) {
    }

    T c;

    // Valid in C++ before 11!!! (Thank god)
    using Derived<T>::C;

    T C2() const {
        return c + C();
    }
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
#if SPH_USE_CXX11
    std::vector<int> v(17);

    applyToAll(v.begin(), v.end(), [](int& i, int j) { i = j; });
    std::for_each(
      v.begin(), v.end(), [](int i) { std::cout << i << std::endl; });

    testFancyCmathStuff();

#endif

    Derived2<double> D;
    std::cout << "A = " << D.A() << std::endl << "C = " << D.C() << std::endl;

    return 0;
}
