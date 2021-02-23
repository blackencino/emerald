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

#include <EmldCore/Util/All.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <exception>

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>

namespace f3d = Field3D;
using namespace EmldCore::Util;

//-*****************************************************************************
void writeFile( const std::string& i_fileName )
{
    f3d::DenseField<float>::Ptr field( new f3d::DenseField<float> );
    field->name = "default";
    field->attribute = "levelset";
    field->setSize( V3i( 50, 50, 50 ) );
    field->clear( 1.0f );
    f3d::Field3DOutputFile out;
    out.create( i_fileName );
    out.writeScalarLayer<float>( field );

    std::cout << "Wrote file: " << i_fileName << std::endl;
}

//-*****************************************************************************
void readFile( const std::string& i_fileName )
{
    f3d::Field3DInputFile in;
    f3d::Field<float>::Vec scalarFields;
    in.open( i_fileName );
    scalarFields = in.readScalarLayers<float>();
    f3d::Field<float>::Vec::const_iterator i = scalarFields.begin(); 
    for ( ; i != scalarFields.end(); ++i ) 
    {
        std::cout << "Name: " << (**i).name << std::endl
                  << "Attribute: " << (**i).attribute << std::endl;
    }

    std::cout << "Read file: " << i_fileName << std::endl;
}

//-*****************************************************************************
void doTest( const std::string& i_fileName )
{
    f3d::initIO();

    writeFile( i_fileName );

    readFile( i_fileName );

    if ( remove( i_fileName.c_str() ) != 0 )
    {
        EMLD_THROW( "Error removing file: " << i_fileName );
    }
    else
    {
        std::cout << "Removed file: " << i_fileName << std::endl;
    }
}

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    char buf[512];
    if ( gethostname( buf, 511 ) != 0 )
    {
        std::cerr << "Can't get hostname" << std::endl;
        exit( -1 );
    }
    std::string fileName = 
        ( boost::format( "emldTest_%s_%d.f3d" ) 
          % ( const char* )buf
          % ( int )getpid() 
          ).str();
    doTest( fileName );

    return 0;
}
