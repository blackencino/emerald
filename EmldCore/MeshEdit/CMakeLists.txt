#-******************************************************************************

# Name our source files

SET( H_FILES 
     All.h 
     Foundation.h
     FixOrientation.h
     PolyEdge.h )

SET( CXX_FILES
     FixOrientation.cpp
     PolyEdge.cpp )

SET( SOURCE_FILES ${CXX_FILES} ${H_FILES} )

# Create the library
ADD_LIBRARY( EmldCoreMeshEdit ${SOURCE_FILES} )

# Recurse into test subdirectory
#ADD_SUBDIRECTORY( Tests )

#-******************************************************************************
#-******************************************************************************
# INSTALLATION
#-******************************************************************************
#-******************************************************************************

INSTALL( TARGETS EmldCoreMeshEdit
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib/static )

INSTALL( FILES ${H_FILES}
         DESTINATION include/EmldCore/MeshEdit
         PERMISSIONS OWNER_READ GROUP_READ WORLD_READ )
