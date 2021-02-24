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

#include "TestViewScene.h"
#include <EmldCore/AbcMeshesScene/All.h>
#include <emerald/simple_sim_viewer/all.h>
#include <iostream>
#include <exception>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using namespace EmldCore;

int main( int argc, char* argv[] )
{
    if ( argc != 3 )
    {
        std::cerr << "USAGE: " << argv[0]
                  << " <file.abc> <ticksPerSecond> " << std::endl;
        exit( -1 );
    }

    std::string abcFileName = argv[1];
    float ticksPerSecond = atof( argv[2] );
    if ( ticksPerSecond == 0.0f )
    {
        std::cerr << "Can't have a zero ticks per second." << std::endl;
        exit( -1 );
    }

    double increment = 1.0 / ticksPerSecond;

    try
    {
        AbcMeshesScene::FileInfoVec files;
        files.push_back( AbcMeshesScene::FileInfo( abcFileName ) );

        AbcMeshesScene::SceneSptr sptr(
            new AbcMeshesScene::Scene( files ) );
        std::cout << "Created scene from file: " << abcFileName
                  << std::endl;

        emerald::simple_sim_viewer::SimPtr vptr(
            new AbcmTest::ViewScene( sptr, increment ) );
        std::cout << "Created view scene." << std::endl;

        emerald::simple_sim_viewer::SimpleViewSim( vptr );
    }
    catch ( std::exception& exc )
    {
        std::cerr << "EXCEPTION: " << exc.what() << std::endl;
        exit( -1 );
    }
    catch ( ... )
    {
        std::cerr << "UNKNOWN EXCEPTION" << std::endl;
        exit( -1 );
    }

    return 0;
}
