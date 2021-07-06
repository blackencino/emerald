#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <EmldCore/TriMesh/All.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace Abc = Alembic::AbcGeom;
namespace TRI = EmldCore::TriMesh;

using Abc::N3f;
using Abc::V3f;
using Abc::V3i;
typedef Imath::Box<V3i> Box3i;
using Abc::Box3f;

// Take a bounding box and a radius.
// Convert the bounds from world space to raster space.
// Rasterize the bounds.

//-*****************************************************************************
void RasterizeBounds(const TRI::TriMesh& iTriMesh,

                     std::vector<V3f>& oPositions,
                     std::vector<V3f>& oClosestPoints,
                     std::vector<float>& oLevelSets,
                     std::vector<N3f>& oNormals,
                     std::vector<float>& oRadius,
                     std::vector<float>& oMass,

                     const V3f& iRasterOrigin,
                     float iRasterRadius,
                     float iMass,
                     const Box3i& iRasterBounds,

                     size_t iMaxRasterPoints,
                     size_t iMaxSubDivs) {
    Box3f worldBounds(
      iRasterOrigin + (iRasterRadius * V3f((float)iRasterBounds.min.x,
                                           (float)iRasterBounds.min.y,
                                           (float)iRasterBounds.min.z)),
      iRasterOrigin + (iRasterRadius * V3f((float)iRasterBounds.max.x,
                                           (float)iRasterBounds.max.y,
                                           (float)iRasterBounds.max.z)));

    // See whether bounds intersect.
    if (!iTriMesh.bounds().intersects(worldBounds)) { return; }

    iMaxRasterPoints = std::max(iMaxRasterPoints, (size_t)16);

    size_t NX = iRasterBounds.max.x - iRasterBounds.min.x;
    size_t NY = iRasterBounds.max.y - iRasterBounds.min.y;
    size_t NZ = iRasterBounds.max.z - iRasterBounds.min.z;
    size_t numPoints = NX * NY * NZ;

    if (iMaxSubDivs > 0 && numPoints > iMaxRasterPoints) {
        // Must split!
        if (NX > NY) {
            if (NX > NZ) {
                size_t HX = NX / 2;

                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(iRasterBounds.min,
                        V3i(static_cast<int>(iRasterBounds.min.x + HX),
                            iRasterBounds.max.y,
                            iRasterBounds.max.z)),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(V3i(static_cast<int>(iRasterBounds.min.x + HX),
                            iRasterBounds.min.y,
                            iRasterBounds.min.z),
                        iRasterBounds.max),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
            } else {
                size_t HZ = NZ / 2;

                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(iRasterBounds.min,
                        V3i(iRasterBounds.max.x,
                            iRasterBounds.max.y,
                            static_cast<int>(iRasterBounds.min.z + HZ))),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(V3i(iRasterBounds.min.x,
                            iRasterBounds.min.y,
                            static_cast<int>(iRasterBounds.min.z + HZ)),
                        iRasterBounds.max),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
            }
        } else {
            if (NY > NZ) {
                size_t HY = NY / 2;

                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(iRasterBounds.min,
                        V3i(iRasterBounds.max.x,
                            static_cast<int>(iRasterBounds.min.y + HY),
                            iRasterBounds.max.z)),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(V3i(iRasterBounds.min.x,
                            static_cast<int>(iRasterBounds.min.y + HY),
                            iRasterBounds.min.z),
                        iRasterBounds.max),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
            } else {
                size_t HZ = NZ / 2;

                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(iRasterBounds.min,
                        V3i(iRasterBounds.max.x,
                            iRasterBounds.max.y,
                            static_cast<int>(iRasterBounds.min.z + HZ))),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
                RasterizeBounds(
                  iTriMesh,

                  oPositions,
                  oClosestPoints,
                  oLevelSets,
                  oNormals,
                  oRadius,
                  oMass,

                  iRasterOrigin,
                  iRasterRadius,
                  iMass,
                  Box3i(V3i(iRasterBounds.min.x,
                            iRasterBounds.min.y,
                            static_cast<int>(iRasterBounds.min.z + HZ)),
                        iRasterBounds.max),

                  iMaxRasterPoints,
                  iMaxSubDivs - 1);
            }
        }
        return;
    }

    // If we get here, we must rasterize our points.
    // CJH: The '<' is not a mistake. don't change to '<='.
    for (int k = iRasterBounds.min.z; k < iRasterBounds.max.z; ++k) {
        for (int j = iRasterBounds.min.y; j < iRasterBounds.max.y; ++j) {
            for (int i = iRasterBounds.min.x; i < iRasterBounds.max.x; ++i) {
                V3f pt = iRasterOrigin +
                         iRasterRadius * V3f((float)i, (float)j, (float)k);

                // std::cout << "Testing point: (" << i
                //          << ", " << j << ", " << k << ") at location: "
                //          << pt << std::endl;

                if (iTriMesh.isInside(pt)) {
                    float rr22 = (25.0f * iRasterRadius * iRasterRadius);

                    TRI::ClosestTriangle ct =
                      iTriMesh.closestPointWithinSquaredDistance(pt, rr22);

                    if (ct.foundAny()) {
                        const V3f& closest = ct.bestPoint;
                        float dist = sqrtf(ct.bestSquaredDist);

                        V3f norm = closest - pt;
                        norm.normalize();

                        oPositions.push_back(pt);
                        oClosestPoints.push_back(closest);
                        oLevelSets.push_back(-dist);
                        oNormals.push_back(norm);
                        oRadius.push_back(iRasterRadius);
                        oMass.push_back(iMass);
                    } else {
                        oPositions.push_back(pt);
                        V3f norm = pt - iRasterOrigin;
                        norm.normalize();

                        oClosestPoints.push_back(pt +
                                                 5.0f * iRasterRadius * norm);
                        oLevelSets.push_back(-5.0f * iRasterRadius);
                        oNormals.push_back(norm);
                        oRadius.push_back(iRasterRadius);
                        oMass.push_back(iMass);
                    }
                }
            }
        }
    }
}

//-*****************************************************************************
// For first mesh encountered,
// compute bounds.
// compute raster origin,
// compute raster bounds,
// create arrays,
// fill arrays,
// save into output.
void RasterizeMesh(Abc::IPolyMesh& iMeshObj,
                   Abc::OObject& iOutParent,
                   float iRasterRadius,
                   float iMass) {
    ABCA_ASSERT(iMeshObj.getNumChildren() == 0,
                "Cannot deal with meshes that have children");

    // Get sample.
    std::cout << "Rasterizing mesh: " << iMeshObj.getName() << std::endl;
    std::cout << "Getting mesh sample." << std::endl;
    Abc::IPolyMeshSchema& mesh = iMeshObj.getSchema();
    Abc::IPolyMeshSchema::Sample msamp;
    mesh.get(msamp);

    // Convert sample to const ranges for tri mesh.
    TRI::ConstV3fRange pointsRange(
      msamp.getPositions()->get(),
      msamp.getPositions()->get() + msamp.getPositions()->size());
    TRI::ConstIntRange indicesRange(
      msamp.getFaceIndices()->get(),
      msamp.getFaceIndices()->get() + msamp.getFaceIndices()->size());
    TRI::ConstIntRange countsRange(
      msamp.getFaceCounts()->get(),
      msamp.getFaceCounts()->get() + msamp.getFaceCounts()->size());

    // Build tri mesh.
    std::cout << "Creating tri mesh." << std::endl;
    TRI::TriMesh triMesh(
      iMeshObj.getName(), pointsRange, indicesRange, countsRange);

    // Get bounds.
    Box3f bnds = triMesh.bounds();
    std::cout << "Bounds of tri mesh: " << bnds.min << " to " << bnds.max
              << std::endl;

    // Get raster origin.
    V3f bndsSize = (bnds.max - bnds.min);
    V3f rasterOrigin = (bnds.max + bnds.min) / 2.0f;

    // Get raster bounds.
    int NX = (int)floorf(0.5f + (bndsSize.x / iRasterRadius));
    int NY = (int)floorf(0.5f + (bndsSize.y / iRasterRadius));
    int NZ = (int)floorf(0.5f + (bndsSize.z / iRasterRadius));
    Box3i rasterBounds(V3i(-(NX + 1) / 2, -(NY + 1) / 2, -(NZ + 1) / 2),
                       V3i(1 + (NX / 2), 1 + (NY / 2), 1 + (NZ / 2)));
    std::cout << "NX = " << NX << ", NY = " << NY << ", NZ = " << NZ
              << std::endl;

    // Create arrays.
    std::vector<V3f> outPositions;
    std::vector<V3f> outClosestPoints;
    std::vector<float> outLevelSets;
    std::vector<N3f> outNormals;
    std::vector<float> outRadius;
    std::vector<float> outMass;

    // Call Rasterize Bounds
    std::cout << "Rasterizing. May take a while." << std::endl;
    RasterizeBounds(triMesh,
                    outPositions,
                    outClosestPoints,
                    outLevelSets,
                    outNormals,
                    outRadius,
                    outMass,

                    rasterOrigin,
                    iRasterRadius,
                    iMass,
                    rasterBounds,

                    16,
                    16);

    // Report sizes.
    size_t numPoints = outPositions.size();
    std::cout << "Created: " << numPoints << " particles" << std::endl;

    // Make output.
    std::cout << "Creating output points." << std::endl;
    Abc::OPoints outPointsObj(iOutParent, iMeshObj.getName());
    Abc::OPointsSchema& outPoints = outPointsObj.getSchema();

    // Set points sample.
    std::cout << "Setting points sample." << std::endl;
    Abc::OPointsSchema::Sample psamp;
    psamp.setPositions(Abc::P3fArraySample(outPositions));
    std::vector<Abc::uint64_t> outIds(numPoints);
    for (size_t p = 0; p < numPoints; ++p) {
        outIds[p] = (Abc::uint64_t)(p + 1);
    }
    psamp.setIds(Abc::UInt64ArraySample(outIds));
    outPoints.set(psamp);

    // Make attributes.
    // They are all 'varying'.
    std::cout << "Making and setting props." << std::endl;
    Abc::MetaData mdata;
    Abc::SetGeometryScope(mdata, Abc::kVaryingScope);
    Abc::OV3fArrayProperty(outPoints, "closestPoint", mdata)
      .set(outClosestPoints);
    Abc::OFloatArrayProperty(outPoints, "levelSet", mdata).set(outLevelSets);
    Abc::ON3fArrayProperty(outPoints, "N", mdata).set(outNormals);
    Abc::OFloatArrayProperty(outPoints, "radius", mdata).set(outRadius);
    Abc::OFloatArrayProperty(outPoints, "mass", mdata).set(outMass);

    // All done.
    std::cout << "All done with: " << iMeshObj.getName() << std::endl;
}

//-*****************************************************************************
void VisitObject(Abc::IObject& iObject,
                 Abc::OObject& oObject,
                 float iRasterRadius,
                 float iMass) {
    std::cout << "Visiting object: " << iObject.getName() << std::endl;

    size_t numChildren = iObject.getNumChildren();
    for (size_t i = 0; i < numChildren; ++i) {
        const Abc::ObjectHeader& childHeader = iObject.getChildHeader(i);

        // See if it is a mesh.
        if (Abc::IPolyMesh::matches(childHeader)) {
            // If so, do the rasterize mesh thing.
            Abc::IPolyMesh pmesh(iObject, childHeader.getName());
            RasterizeMesh(pmesh, oObject, iRasterRadius, iMass);
        } else if (Abc::IXform::matches(childHeader)) {
            // Make an xform.
            Abc::IXform ixform(iObject, childHeader.getName());
            Abc::OXform oxform(oObject, childHeader.getName());

            // And visit them.
            VisitObject(ixform, oxform, iRasterRadius, iMass);
        } else {
            // Just treat them like objects, and visit them.
            // This will lose the properties of the object, c'est la vie.
            Abc::IObject iobj(iObject, childHeader.getName());
            Abc::OObject oobj(oObject, childHeader.getName());
            VisitObject(iobj, oobj, iRasterRadius, iMass);
        }
    }
}

//-*****************************************************************************
void DoIt(const std::string& iInFileName,
          const std::string& iOutFileName,
          float iRasterRadius,
          float iMass) {
    Alembic::AbcCoreFactory::IFactory factory;
    Abc::IArchive inArchive = factory.getArchive(iInFileName);
    Abc::IObject inTopObject(inArchive, Abc::kTop);
    std::cout << "Opened archive: " << iInFileName << " for read. "
              << std::endl;

    Abc::OArchive outArchive(Alembic::AbcCoreOgawa::WriteArchive(),
                             iOutFileName);
    outArchive.setCompressionHint(6);
    Abc::OObject outTopObject(outArchive, Abc::kTop);
    std::cout << "Opened archive: " << iOutFileName << " for write. "
              << std::endl;

    VisitObject(inTopObject, outTopObject, iRasterRadius, iMass);

    std::cout << "All done." << std::endl;
}

//-*****************************************************************************
int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "USAGE: " << argv[0]
                  << " <inFile.abc> <outFile.abc> <sampleRadius>" << std::endl;
        exit(-1);
    }

    const char* inFileName = argv[1];
    const char* outFileName = argv[2];
    float radius = static_cast<float>(atof(argv[3]));

    float density = 1.0f;
    float volume = 8.0f * radius * radius * radius;
    float mass = density * volume;

    DoIt(inFileName, outFileName, radius, mass);

    return 0;
}
