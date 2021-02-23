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
void copyFile( const std::string& i_inFileName,
               const std::string& i_outFileName )
{   
    f3d::initIO();

    f3d::Field3DInputFile in;
    f3d::Field<float>::Vec scalarFields;
    in.open( i_inFileName );

#if 0
    f3d::Field3DOutputFile out;
    out.create( i_outFileName );

    scalarFields = in.readScalarLayers<float>();
    f3d::Field<float>::Vec::iterator i = scalarFields.begin(); 
    for ( ; i != scalarFields.end(); ++i ) 
    {
        std::cout << "Name: " << (**i).name << std::endl
                  << "Attribute: " << (**i).attribute << std::endl;

        f3d::DenseField<float>::Ptr outField( new f3d::DenseField<float> );
        outField->name = (**i).name;
        outField->attribute = (**i).attribute;
        outField->matchDefinition( (*i) );
        outField->copyFrom( (*i) );
        out.writeScalarLayer<float>( outField );
    }

    std::cout << "Copied file: " << i_inFileName 
              << " to file: " << i_outFileName << std::endl;
#endif
}

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    if ( argc < 3 )
    {
        std::cerr << "USAGE: " << argv[0] << " <in.f3d> <out.f3d>"
                  << std::endl;
        exit( -1 );
    }

    copyFile( argv[1], argv[2] );

    return 0;
}
