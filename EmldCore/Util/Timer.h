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

#ifndef _EmldCore_Util_Timer_h_
#define _EmldCore_Util_Timer_h_

#include "Foundation.h"

namespace EmldCore {
namespace Util {

//-*****************************************************************************
//! \brief Basic real-time stopwatch. Returns elapsed time in seconds.
class Timer
{
public:
    //! Begins the timer and resets the elapsed time to zero
    void start()
    {
        struct timezone tz;
        gettimeofday( &m_tp0, &tz );
        m_stopped = -1;
    }

    //! Creates a timer which is started by default
    Timer() : m_stopped( -1 ) { start(); }

    //! Stops the timer and records elapsed time
    double stop()
    {
        struct timezone tz;
        struct timeval tp;
        gettimeofday( &tp, &tz );

        double seconds = tp.tv_sec - m_tp0.tv_sec;
        seconds += double(tp.tv_usec) * 1e-6;
        seconds -= double(m_tp0.tv_usec) * 1e-6;
        return ( m_stopped = double(seconds) );
    }

    //! Returns the amount of time elapsed.
    double elapsed() const
    {
        if ( m_stopped >= 0.0 )
        {
            return m_stopped;
        }
    
        struct timezone tz;
        struct timeval tp;
        gettimeofday( &tp, &tz );

        double seconds = tp.tv_sec - m_tp0.tv_sec;
        seconds += double( tp.tv_usec ) * 1e-6;
        seconds -= double( m_tp0.tv_usec ) * 1e-6;
        return double(seconds);
    }

private:
    double m_stopped;
    struct timeval m_tp0;
};

} // End namespace Util
} // End namespace EmldCore

#endif
