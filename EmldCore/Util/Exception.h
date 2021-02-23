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

#ifndef _EmldCore_Util_Exception_h_
#define _EmldCore_Util_Exception_h_

#include "Foundation.h"

namespace EmldCore {
namespace Util {

//-*****************************************************************************
//! Base class for all exceptions in the Emld libraries. Derived
//! from both std::exception and std::string, publicly
//! It is mostly commonly thrown using the macros
class Exception : public std::string, public std::exception
{
public:
    //! default constructor creates exception with
    //! empty message string
    Exception() throw() : std::string( "" ), std::exception() {}

    //!  Creates exception with an explicit message string.
    //! ...
    explicit Exception( const std::string &str ) throw()
      : std::string( str ), std::exception() {}

    //!  Copies exception.
    //! ...
    Exception( const Exception &exc ) throw()
      : std::string( exc.c_str() ), std::exception() {}

    //!  Destructor is empty, but virtual to support polymorphic
    //! destruction of data in any derived classes.
    virtual ~Exception() throw() {}

    //!  Inherited from std::exception, this returns a non-modifiable
    //! character string describing the nature of the exception
    virtual const char *what() const throw() { return c_str(); }
};

//-*****************************************************************************
// This is for aborting in a debug mode to make a more traceable stack.

#ifdef PLATFORM_DARWIN

#if defined __cplusplus
# define __EMLD_DEBUG_ASSERT_VOID_CAST static_cast<void>
#else
# define __EMLD_DEBUG_ASSERT_VOID_CAST (void)
#endif

#else

#if defined __cplusplus && __GNUC_PREREQ (2,95)
# define __EMLD_DEBUG_ASSERT_VOID_CAST static_cast<void>
#else
# define __EMLD_DEBUG_ASSERT_VOID_CAST (void)
#endif

#endif

extern void __EMLD_DEBUG_ASSERT_FAIL( const char *msg ) throw();

//-*****************************************************************************
// This macro will cause an abort.
#define EMLD_FAIL( TEXT )                                              \
do                                                                        \
{                                                                         \
    std::stringstream sstr;                                               \
    sstr << TEXT;                                                         \
    sstr << "\nFile: " << __FILE__ << std::endl                           \
         << "Line: " << __LINE__ << std::endl;                            \
    EmldCore::Util::__EMLD_DEBUG_ASSERT_FAIL( sstr.str().c_str() ); \
}                                                                         \
while( 0 )

//-*****************************************************************************
//! convenient macro which may be used with std::iostream syntax
//! EMLD_THROW( "this integer: " << myInt << " is bad" )
//#ifdef DEBUG
#if 0

#define EMLD_THROW( TEXT )                  \
    do                                         \
    {                                          \
        std::cerr << TEXT << std::endl;        \
        abort();                               \
    }                                          \
    while( 0 )

#else

#define EMLD_THROW( TEXT )                         \
do                                                    \
{                                                     \
    std::stringstream sstr;                           \
    sstr << TEXT;                                     \
    sstr << "\nFile: " << __FILE__ << std::endl       \
         << "Line: " << __LINE__ << std::endl;        \
    EmldCore::Util::Exception exc( sstr.str() );   \
    throw( exc );                                     \
}                                                     \
while( 0 )

#endif

//-*****************************************************************************
#ifdef DEBUG

#define EMLD_ASSERT( COND, TEXT )            \
do                                              \
{                                               \
    if ( !( COND ) )                            \
    {                                           \
        EMLD_FAIL( TEXT );                   \
    }                                           \
}                                               \
while( 0 )

#define EMLD_DEBUG_ASSERT( COND, TEXT )      \
do                                              \
{                                               \
    if ( !( COND ) )                            \
    {                                           \
        EMLD_FAIL( TEXT );                   \
    }                                           \
}                                               \
while( 0 )

#else

#define EMLD_ASSERT( COND, TEXT )            \
do                                              \
{                                               \
    if ( !( COND ) )                            \
    {                                           \
        EMLD_THROW( TEXT );                  \
    }                                           \
}                                               \
while( 0 )

#define EMLD_DEBUG_ASSERT( COND, TEXT ) (__EMLD_DEBUG_ASSERT_VOID_CAST (0))

#endif


} // End namespace Util
} // End namespace EmldCore

#endif
