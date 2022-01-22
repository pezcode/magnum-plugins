/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                2020, 2021 Vladimír Vondruš <mosra@centrum.cz>

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

#include "MeshOptimizerSceneConverter.h"

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/MeshTools/Combine.h>
#include <Magnum/MeshTools/CompressIndices.h>
#include <Magnum/MeshTools/Duplicate.h>
#include <Magnum/MeshTools/GenerateIndices.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/Trade/ArrayAllocator.h>
#include <Magnum/Trade/MeshData.h>
#include <meshoptimizer.h>

#define MESHOPT_HAS_COMPRESSION (MESHOPTIMIZER_VERSION >= 140)
#define MESHOPT_HAS_FILTER_DECODING (MESHOPTIMIZER_VERSION >= 140)
#define MESHOPT_HAS_FILTER_ENCODING (MESHOPTIMIZER_VERSION >= 170)

namespace Magnum { namespace Trade {

MeshOptimizerSceneConverter::MeshOptimizerSceneConverter(PluginManager::AbstractManager& manager, const std::string& plugin): AbstractSceneConverter{manager, plugin} {}

MeshOptimizerSceneConverter::~MeshOptimizerSceneConverter() = default;

SceneConverterFeatures MeshOptimizerSceneConverter::doFeatures() const {
    return SceneConverterFeature::ConvertMeshInPlace|SceneConverterFeature::ConvertMesh;
}

namespace {

template<class T> void analyze(const MeshData& mesh, const Utility::ConfigurationGroup& configuration, const UnsignedInt vertexSize, const Containers::StridedArrayView1D<const Vector3> positions, meshopt_VertexCacheStatistics& vertexCacheStats, meshopt_VertexFetchStatistics& vertexFetchStats, meshopt_OverdrawStatistics& overdrawStats) {
    const auto indices = mesh.indices<T>().asContiguous();
    vertexCacheStats = meshopt_analyzeVertexCache(indices.data(), mesh.indexCount(), mesh.vertexCount(), configuration.value<UnsignedInt>("analyzeCacheSize"), configuration.value<UnsignedInt>("analyzeWarpSize"), configuration.value<UnsignedInt>("analyzePrimitiveGroupSize"));
    if(vertexSize) vertexFetchStats = meshopt_analyzeVertexFetch(indices.data(), mesh.indexCount(), mesh.vertexCount(), vertexSize);
    if(positions) overdrawStats = meshopt_analyzeOverdraw(indices.data(), mesh.indexCount(), static_cast<const float*>(positions.data()), mesh.vertexCount(), positions.stride());
}

void analyze(const MeshData& mesh, const Utility::ConfigurationGroup& configuration, const Containers::StridedArrayView1D<const Vector3> positions, Containers::Optional<UnsignedInt>& vertexSize, meshopt_VertexCacheStatistics& vertexCacheStats, meshopt_VertexFetchStatistics& vertexFetchStats, meshopt_OverdrawStatistics& overdrawStats) {
    /* Calculate vertex size out of all attributes. If any attribute is
       implementation-specific, do nothing (warning will be printed by the
       caller) */
    if(!vertexSize) {
        vertexSize = 0;
        for(UnsignedInt i = 0; i != mesh.attributeCount(); ++i) {
            VertexFormat format = mesh.attributeFormat(i);
            const UnsignedInt arraySize = mesh.attributeArraySize(i);
            if(isVertexFormatImplementationSpecific(format)) {
                vertexSize = 0;
                break;
            }
            *vertexSize += vertexFormatSize(format)*(arraySize ? arraySize : 1);
        }
    }

    if(mesh.indexType() == MeshIndexType::UnsignedInt)
        analyze<UnsignedInt>(mesh, configuration, *vertexSize, positions, vertexCacheStats, vertexFetchStats, overdrawStats);
    else if(mesh.indexType() == MeshIndexType::UnsignedShort)
        analyze<UnsignedShort>(mesh, configuration, *vertexSize, positions, vertexCacheStats, vertexFetchStats, overdrawStats);
    else if(mesh.indexType() == MeshIndexType::UnsignedByte)
        analyze<UnsignedByte>(mesh, configuration, *vertexSize, positions, vertexCacheStats, vertexFetchStats, overdrawStats);
    else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
}

void analyzePost(const char* prefix, const MeshData& mesh, const Utility::ConfigurationGroup& configuration, const Containers::StridedArrayView1D<const Vector3> positions, Containers::Optional<UnsignedInt>& vertexSize, meshopt_VertexCacheStatistics& vertexCacheStatsBefore, meshopt_VertexFetchStatistics& vertexFetchStatsBefore, meshopt_OverdrawStatistics& overdrawStatsBefore) {
    /* If vertex size is zero, it means there was an implementation-specific
       vertex format somewhere. Print a warning about that. */
    CORRADE_INTERNAL_ASSERT(vertexSize);
    if(!*vertexSize) for(UnsignedInt i = 0; i != mesh.attributeCount(); ++i) {
        VertexFormat format = mesh.attributeFormat(i);
        if(isVertexFormatImplementationSpecific(format)) {
            Warning{} << prefix << "can't analyze vertex fetch for" << format;
            break;
        }
    }

    meshopt_VertexCacheStatistics vertexCacheStats;
    meshopt_VertexFetchStatistics vertexFetchStats;
    meshopt_OverdrawStatistics overdrawStats;
    analyze(mesh, configuration, positions, vertexSize, vertexCacheStats, vertexFetchStats, overdrawStats);

    Debug{} << prefix << "processing stats:";
    Debug{} << "  vertex cache:\n   "
        << vertexCacheStatsBefore.vertices_transformed << "->"
        << vertexCacheStats.vertices_transformed
        << "transformed vertices\n   "
        << vertexCacheStatsBefore.warps_executed << "->"
        << vertexCacheStats.warps_executed << "executed warps\n    ACMR"
        << vertexCacheStatsBefore.acmr << "->" << vertexCacheStats.acmr
        << Debug::newline << "    ATVR" << vertexCacheStatsBefore.atvr
        << "->" << vertexCacheStats.atvr;
    if(*vertexSize) Debug{} << "  vertex fetch:\n   "
        << vertexFetchStatsBefore.bytes_fetched << "->"
        << vertexFetchStats.bytes_fetched << "bytes fetched\n    overfetch"
        << vertexFetchStatsBefore.overfetch << "->"
        << vertexFetchStats.overfetch;
    if(positions) Debug{} << "  overdraw:\n   "
        << overdrawStatsBefore.pixels_shaded << "->"
        << overdrawStats.pixels_shaded << "shaded pixels\n   "
        << overdrawStatsBefore.pixels_covered << "->"
        << overdrawStats.pixels_covered << "covered pixels\n    overdraw"
        << overdrawStatsBefore.overdraw << "->" << overdrawStats.overdraw;
}

void populatePositions(const MeshData& mesh, Containers::Array<Vector3>& positionStorage, Containers::StridedArrayView1D<const Vector3>& positions) {
    /* MeshOptimizer accepts float positions with stride divisible by four. If
       the input doesn't have that (for example because it's a tightly-packed
       PLY with 24bit RGB colors), we need to supply unpacked aligned copy. */
    if(mesh.attributeFormat(MeshAttribute::Position) == VertexFormat::Vector3 && mesh.attributeStride(MeshAttribute::Position) % 4 == 0)
        positions = mesh.attribute<Vector3>(MeshAttribute::Position);
    else {
        positionStorage = mesh.positions3DAsArray();
        positions = positionStorage;
    }
}

bool optimize(const char* prefix, MeshData& mesh, const SceneConverterFlags flags, const Utility::ConfigurationGroup& configuration, Containers::Array<Vector3>& positionStorage, Containers::StridedArrayView1D<const Vector3>& positions, Containers::Optional<UnsignedInt>& vertexSize,  meshopt_VertexCacheStatistics& vertexCacheStatsBefore, meshopt_VertexFetchStatistics& vertexFetchStatsBefore, meshopt_OverdrawStatistics& overdrawStatsBefore) {
    /* Only doConvert() can handle triangle strips etc, in-place only triangles */
    if(mesh.primitive() != MeshPrimitive::Triangles) {
        Error{} << prefix << "expected a triangle mesh, got" << mesh.primitive();
        return false;
    }

    /* Can't really do anything with non-indexed meshes, sorry */
    if(!mesh.isIndexed()) {
        Error{} << prefix << "expected an indexed mesh";
        return false;
    }

    /* If we need it, get the position attribute, unpack if packed. It's used
       by the verbose stats also but in that case the processing shouldn't fail
       if there are no positions -- so check the hasAttribute() earlier. */
    if((flags & SceneConverterFlag::Verbose && mesh.hasAttribute(MeshAttribute::Position)) ||
       configuration.value<bool>("optimizeOverdraw") ||
       configuration.value<bool>("simplify") ||
       configuration.value<bool>("simplifySloppy"))
    {
        if(!mesh.hasAttribute(MeshAttribute::Position)) {
            Error{} << prefix << "optimizeOverdraw and simplify require the mesh to have positions";
            return false;
        }

        populatePositions(mesh, positionStorage, positions);
    }

    /* Save "before" stats if verbose output is requested. No messages as those
       will be printed only at the end if the processing passes. */
    if(flags & SceneConverterFlag::Verbose) {
        analyze(mesh, configuration, positions, vertexSize, vertexCacheStatsBefore, vertexFetchStatsBefore, overdrawStatsBefore);
    }

    /* Vertex cache optimization. Goes first. */
    if(configuration.value<bool>("optimizeVertexCache")) {
        if(mesh.indexType() == MeshIndexType::UnsignedInt) {
            Containers::ArrayView<UnsignedInt> indices = mesh.mutableIndices<UnsignedInt>().asContiguous();
            meshopt_optimizeVertexCache(indices.data(), indices.data(), mesh.indexCount(), mesh.vertexCount());
        } else if(mesh.indexType() == MeshIndexType::UnsignedShort) {
            Containers::ArrayView<UnsignedShort> indices = mesh.mutableIndices<UnsignedShort>().asContiguous();
            meshopt_optimizeVertexCache(indices.data(), indices.data(), mesh.indexCount(), mesh.vertexCount());
        } else if(mesh.indexType() == MeshIndexType::UnsignedByte) {
            Containers::ArrayView<UnsignedByte> indices = mesh.mutableIndices<UnsignedByte>().asContiguous();
            meshopt_optimizeVertexCache(indices.data(), indices.data(), mesh.indexCount(), mesh.vertexCount());
        } else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
    }

    /* Overdraw optimization. Goes after vertex cache optimization. */
    if(configuration.value<bool>("optimizeOverdraw")) {
        const Float optimizeOverdrawThreshold = configuration.value<Float>("optimizeOverdrawThreshold");

        if(mesh.indexType() == MeshIndexType::UnsignedInt) {
            Containers::ArrayView<UnsignedInt> indices = mesh.mutableIndices<UnsignedInt>().asContiguous();
            meshopt_optimizeOverdraw(indices.data(), indices.data(), mesh.indexCount(), static_cast<const Float*>(positions.data()), mesh.vertexCount(), positions.stride(), optimizeOverdrawThreshold);
        } else if(mesh.indexType() == MeshIndexType::UnsignedShort) {
            Containers::ArrayView<UnsignedShort> indices = mesh.mutableIndices<UnsignedShort>().asContiguous();
            meshopt_optimizeOverdraw(indices.data(), indices.data(), mesh.indexCount(), static_cast<const Float*>(positions.data()), mesh.vertexCount(), positions.stride(), optimizeOverdrawThreshold);
        } else if(mesh.indexType() == MeshIndexType::UnsignedByte) {
            Containers::ArrayView<UnsignedByte> indices = mesh.mutableIndices<UnsignedByte>().asContiguous();
            meshopt_optimizeOverdraw(indices.data(), indices.data(), mesh.indexCount(), static_cast<const Float*>(positions.data()), mesh.vertexCount(), positions.stride(), optimizeOverdrawThreshold);
        } else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
    }

    /* Vertex fetch optimization. Goes after overdraw optimization. Reorders
       the vertex buffer for better memory locality, so if we have no
       attributes it's of no use (also meshoptimizer asserts in that case).
       Skipping silently instead of failing hard, as an attribute-less mesh
       always *is* optimized for vertex fetch, so there's nothing wrong. */
    if(configuration.value<bool>("optimizeVertexFetch") && mesh.attributeCount()) {
        /* This assumes the mesh is interleaved. doConvert() already ensures
           that, doConvertInPlace() has a runtime check */
        Containers::StridedArrayView2D<char> interleavedData = MeshTools::interleavedMutableData(mesh);

        if(mesh.indexType() == MeshIndexType::UnsignedInt) {
            Containers::ArrayView<UnsignedInt> indices = mesh.mutableIndices<UnsignedInt>().asContiguous();
            meshopt_optimizeVertexFetch(interleavedData.data(), indices.data(), mesh.indexCount(), interleavedData.data(), mesh.vertexCount(), interleavedData.stride()[0]);
        } else if(mesh.indexType() == MeshIndexType::UnsignedShort) {
            Containers::ArrayView<UnsignedShort> indices = mesh.mutableIndices<UnsignedShort>().asContiguous();
            meshopt_optimizeVertexFetch(interleavedData.data(), indices.data(), mesh.indexCount(), interleavedData.data(), mesh.vertexCount(), interleavedData.stride()[0]);
        } else if(mesh.indexType() == MeshIndexType::UnsignedByte) {
            Containers::ArrayView<UnsignedByte> indices = mesh.mutableIndices<UnsignedByte>().asContiguous();
            meshopt_optimizeVertexFetch(interleavedData.data(), indices.data(), mesh.indexCount(), interleavedData.data(), mesh.vertexCount(), interleavedData.stride()[0]);
        } else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
    }

    return true;
}

enum class AttributeEncoding: UnsignedInt {
    Unencoded = 1 << 0,
    EncodedStream = 1 << 1,
    FilteredOctahedral = 1 << 2,
    FilteredQuaternion = 1 << 3,
    FilteredExponential = 1 << 4
};

typedef Containers::EnumSet<AttributeEncoding> AttributeEncodings;
CORRADE_ENUMSET_OPERATORS(AttributeEncodings);

/** @todo How do we advertize this to plugin users? AbstractSceneConverter has
    no meshAttributeForName(). */
constexpr MeshAttribute EncodedMeshVertexCount = meshAttributeCustom(0);

bool isEncodedMesh(const MeshData& mesh) {
    return mesh.vertexCount() == 0 &&
        mesh.hasAttribute(EncodedMeshVertexCount);
}

bool isEncodedStream(VertexFormat format) {
    return isVertexFormatImplementationSpecific(format) &&
        vertexFormatUnwrap<AttributeEncodings>(format) == AttributeEncoding::EncodedStream;
}

bool isFilteredAttribute(VertexFormat format) {
    return isVertexFormatImplementationSpecific(format) &&
        vertexFormatUnwrap<AttributeEncodings>(format) <=
            (AttributeEncoding::FilteredOctahedral | AttributeEncoding::FilteredQuaternion | AttributeEncoding::FilteredExponential);
}

/* To allow mixing uncompressed attributes with compressed attributes, they
   both have to have placeholder attributes. Any mesh that has compressed
   attributes mixed with regular attributes is invalid. */
bool isUnencodedAttribute(VertexFormat format) {
    return isVertexFormatImplementationSpecific(format) &&
        vertexFormatUnwrap<AttributeEncodings>(format) == AttributeEncoding::Unencoded;
}

}

bool MeshOptimizerSceneConverter::doConvertInPlace(MeshData& mesh) {
    if((configuration().value<bool>("optimizeVertexCache") ||
        configuration().value<bool>("optimizeOverdraw") ||
        configuration().value<bool>("optimizeVertexFetch")) &&
       !(mesh.indexDataFlags() & DataFlag::Mutable))
    {
        Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): optimizeVertexCache, optimizeOverdraw and optimizeVertexFetch require index data to be mutable";
        return false;
    }

    if(configuration().value<bool>("optimizeVertexFetch")) {
        if(!(mesh.vertexDataFlags() & DataFlag::Mutable)) {
            Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): optimizeVertexFetch requires vertex data to be mutable";
            return false;
        }

        if(!MeshTools::isInterleaved(mesh)) {
            Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): optimizeVertexFetch requires the mesh to be interleaved";
            return false;
        }
    }

    if(configuration().value<bool>("simplify") ||
       configuration().value<bool>("simplifySloppy"))
    {
        Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): mesh simplification can't be performed in-place, use convert() instead";
        return false;
    }

    if(configuration().value<bool>("decodeVertexBuffer") ||
       configuration().value<bool>("decodeIndexBuffer") ||
       configuration().value<bool>("encodeVertexBuffer") ||
       configuration().value<bool>("encodeIndexBuffer") ||
       configuration().value<bool>("decodeFilter") ||
       configuration().value<bool>("encodeFilter"))
    {
        Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): mesh decoding, encoding and filtering can't be performed in-place, use convert() instead";
        return false;
    }

    /* Errors for non-indexed meshes and implementation-specific index buffers
       are printed directly in convertInPlaceInternal() */
    if(mesh.isIndexed()) {
        if(isMeshIndexTypeImplementationSpecific(mesh.indexType())) {
            Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): can't perform any operation on an implementation-specific index type" << reinterpret_cast<void*>(meshIndexTypeUnwrap(mesh.indexType()));
            return false;
        }

        if(Short(meshIndexTypeSize(mesh.indexType())) != mesh.indexStride()) {
            Error{} << "Trade::MeshOptimizerSceneConverter::convertInPlace(): in-place conversion is possible only with contiguous index buffers";
            return false;
        }
    }

    meshopt_VertexCacheStatistics vertexCacheStatsBefore;
    meshopt_VertexFetchStatistics vertexFetchStatsBefore;
    meshopt_OverdrawStatistics overdrawStatsBefore;
    Containers::Array<Vector3> positionStorage;
    Containers::StridedArrayView1D<const Vector3> positions;
    Containers::Optional<UnsignedInt> vertexSize;
    if(!optimize("Trade::MeshOptimizerSceneConverter::convertInPlace():", mesh, flags(), configuration(), positionStorage, positions, vertexSize, vertexCacheStatsBefore, vertexFetchStatsBefore, overdrawStatsBefore))
        return false;

    if(flags() & SceneConverterFlag::Verbose)
        analyzePost("Trade::MeshOptimizerSceneConverter::convertInPlace():", mesh, configuration(), positions, vertexSize, vertexCacheStatsBefore, vertexFetchStatsBefore, overdrawStatsBefore);

    return true;
}

Containers::Optional<MeshData> MeshOptimizerSceneConverter::doConvert(const MeshData& mesh) {
    MeshData out = MeshTools::reference(mesh);

    /** @todo What happens when we don't decode encoded attributes? */
    if(configuration().value<bool>("decodeVertexBuffer") && out.attributeCount() && true /*isEncodedMesh()*/) {
#if !MESHOPT_HAS_COMPRESSION
        Error{} << "Trade::MeshOptimizerSceneConverter::convert(): vertex encoding requires meshoptimizer 0.14 or higher";
        return Containers::NullOpt;
#else
        UnsignedInt encodedAttributeCount = 0;
        for(UnsignedInt i = 0; i != out.attributeCount(); ++i)
            encodedAttributeCount += UnsignedInt(isVertexFormatImplementationSpecific(out.attributeFormat(i)));

        if(encodedAttributeCount > 0) {
            if(encodedAttributeCount != out.attributeCount()) {
                Error{} << "Trade::MeshOptimizerSceneConverter::convert(): TODO";
                return Containers::NullOpt;
            }

            /* We can't support multiple compressed streams  */
            // probably not since we can only assume that the entire blob is encoded.
            // -> can't use offset + vertex count for data size
            if(!MeshTools::isInterleaved(out)) {
                Error{} << "Trade::MeshOptimizerSceneConverter::convert(): TODO";
                return Containers::NullOpt;
            }

            Containers::Array<MeshAttributeData> attributes{NoInit, out.attributeCount()};
            for(UnsignedInt i = 0; i != out.attributeCount(); ++i) {
                const VertexFormat originalFormat = vertexFormatUnwrap<VertexFormat>(out.attributeFormat(i));
                attributes[i] = MeshAttributeData{out.attributeName(i), originalFormat,
                    out.attributeOffset(i), out.vertexCount(), out.attributeStride(i),
                    out.attributeArraySize(i)};
            }

            Containers::ArrayView<const unsigned char> data = Containers::arrayCast<const unsigned char>(out.vertexData());
            // TODO use MeshTools::interleavedData(out)?
            // That includes the min offset, but that's for AFTER decoding
            // and we're not guaranteed to have interleaved data

            // TODO this won't work if the requirement is that compressed
            // stride is always 0. we'll have to get the stride from
            // offset + size of the last (= highest offset) attribute
            const std::size_t vertexSize = out.attributeStride(0);
            Containers::Array<char> decoded{NoInit, out.vertexCount()*vertexSize};
            const int result = meshopt_decodeVertexBuffer(decoded.data(), out.vertexCount(), vertexSize, data.data(), data.size());
            if(result != 0) {
                /* An opaque number is as much info as we can get from meshopt */
                Error{} << "Trade::MeshOptimizerSceneConverter::convert(): vertex buffer decoding failed with error" << result;
                return Containers::NullOpt;
            }

            out = MeshData{out.primitive(),
                // TODO is this a dangling reference when indexData is owned?
                out.indexDataFlags(), out.indexData(), MeshIndexData{out.indices()},
                std::move(decoded), std::move(attributes), out.vertexCount()};
        }
#endif
    }

    if(configuration().value<bool>("decodeIndexBuffer") && out.isIndexed() && true /*isEncodedMesh()*/) {
#if !MESHOPT_HAS_COMPRESSION
        Error{} << "Trade::MeshOptimizerSceneConverter::convert(): index encoding requires meshoptimizer 0.14 or higher";
        return Containers::NullOpt;
#else
        /** @todo We need implementation-specific index type to detect encoded
            data. One should be enough if MeshData also has index stride, then
            we can detect the original type from the stride? Or should we stay
            generic, in case we do need the original data stride in the future,
            similar to vertex attribute stride? */
        /** @todo Check that the index type is implementation-specific before
            decoding */
        if(true) {
            // TODO unwarp from implementation-specific
            const MeshIndexType indexType = out.indexType();
            if(indexType == MeshIndexType::UnsignedByte) {
                /* Decoding 8-bit indices is not supported by meshopt. However,
                   encoding them is allowed because meshopt converts all
                   indices to 32-bit. */
                /** @todo Is this purely a user choice when decoding? If so, we
                    could expose this as a config option. */
                Error{} << "Trade::MeshOptimizerSceneConverter::convert(): can't decode 8-bit index buffer";
                return Containers::NullOpt;
            }

            // TODO this may not exist for implementation-specific index types
            const auto encoded = Containers::arrayCast<const unsigned char>(out.indices().asContiguous());

            int result;
            Containers::Array<char> decoded{NoInit, out.indexCount()*meshIndexTypeSize(indexType)};
            if(out.primitive() == MeshPrimitive::Triangles) {
                if(indexType == MeshIndexType::UnsignedInt) {
                    UnsignedInt* output = reinterpret_cast<UnsignedInt*>(decoded.data());
                    result = meshopt_decodeIndexBuffer(output, out.indexCount(), encoded.data(), encoded.size());
                } else if(indexType == MeshIndexType::UnsignedShort) {
                    UnsignedShort* output = reinterpret_cast<UnsignedShort*>(decoded.data());
                    result = meshopt_decodeIndexBuffer(output, out.indexCount(), encoded.data(), encoded.size());
                } else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
            } else {
                /* Apart from picking the optimized decoding function for
                   triangles, we don't care about the primitive type. Even if
                   it's implementation-specific, we're just decoding integers. */
                if(indexType == MeshIndexType::UnsignedInt) {
                    UnsignedInt* output = reinterpret_cast<UnsignedInt*>(decoded.data());
                    result = meshopt_decodeIndexSequence(output, out.indexCount(), encoded.data(), encoded.size());
                } else if(indexType == MeshIndexType::UnsignedShort) {
                    UnsignedShort* output = reinterpret_cast<UnsignedShort*>(decoded.data());
                    result = meshopt_decodeIndexSequence(output, out.indexCount(), encoded.data(), encoded.size());
                } else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
            }

            if(result != 0) {
                Error{} << "Trade::MeshOptimizerSceneConverter::convert(): index buffer decoding failed with error" << result;
                return Containers::NullOpt;
            }

            const MeshIndexData indices{indexType, decoded};
            out = MeshData{out.primitive(), std::move(decoded), indices,
                // TODO is this a dangling reference when vertexData is owned?
                out.vertexDataFlags(), out.vertexData(), out.releaseAttributeData(), out.vertexCount()};
        }
#endif
    }

    if(configuration().value<bool>("decodeFilter") && out.attributeCount() && true /*isFilteredMesh()*/) {
#if !MESHOPT_HAS_FILTER_DECODING
        Error{} << "Trade::MeshOptimizerSceneConverter::convert(): filter decoding requires meshoptimizer 0.14 or higher";
        return Containers::NullOpt;
#else
        for(UnsignedInt i = 0; i != out.attributeCount(); ++i) {
            if(isFilteredAttribute(out.attributeFormat(i))) {
                const AttributeEncodings type = vertexFormatUnwrap<AttributeEncodings>(out.attributeFormat(i));

                const auto data = out.mutableAttribute(i);
                if(!data.isContiguous()) {
                    // TODO error, interleaved not allowed
                    return Containers::NullOpt;
                }

                const UnsignedInt stride = out.attributeStride(i);
                VertexFormat format;
                if(type == AttributeEncoding::FilteredOctahedral) {
                    if(stride == 4) {
                        format = VertexFormat::Vector4bNormalized;
                    } else if(stride == 8) {
                        format = VertexFormat::Vector4sNormalized;
                    } else {
                        // TODO error
                        return Containers::NullOpt;
                    }
                    meshopt_decodeFilterOct(data.asContiguous(), out.vertexCount(), stride);
                } else if(type == AttributeEncoding::FilteredQuaternion) {
                    format = VertexFormat::Vector4;
                    meshopt_decodeFilterQuat(data.asContiguous(), out.vertexCount(), stride);
                } else if(type == AttributeEncoding::FilteredExponential) {
                    if(stride == 4) {
                        format = VertexFormat::Float;
                    } else if(stride == 8) {
                        format = VertexFormat::Vector2;
                    } else if(stride == 12) {
                        format = VertexFormat::Vector3;
                    } else if(stride == 16) {
                        format = VertexFormat::Vector4;
                    } else {
                        // TODO error
                        return Containers::NullOpt;
                    }
                    meshopt_decodeFilterExp(data.asContiguous(), out.vertexCount(), stride);
                } else {
                    // TODO error
                    return Containers::NullOpt;
                }

                // TODO set new format
            }
        }
#endif
    }

    /* If the mesh is indexed with an implementation-specific index type,
       interleave() won't be able to turn its index buffer into a contiguous
       one. So fail early if that's the case. The mesh doesn't necessarily have
       to be indexed though -- it could be e.g. a triangle strip which we turn
       into an indexed mesh right after. */
    if(mesh.isIndexed() && isMeshIndexTypeImplementationSpecific(mesh.indexType())) {
        Error{} << "Trade::MeshOptimizerSceneConverter::convert(): can't perform any operation on an implementation-specific index type" << reinterpret_cast<void*>(meshIndexTypeUnwrap(mesh.indexType()));
        return {};
    }

    /* Make the mesh interleaved (with a contiguous index array) and owned */
    // TODO this will assert in the future for non-interleaved meshes with
    // implementation-specific vertex formats
    // is this programmer error or should we check for this?
    out = MeshTools::owned(MeshTools::interleave(std::move(out)));
    CORRADE_INTERNAL_ASSERT(MeshTools::isInterleaved(out));
    CORRADE_INTERNAL_ASSERT(!out.isIndexed() || out.indices().isContiguous());

    /* Convert to an indexed triangle mesh if we have a strip or a fan */
    if(out.primitive() == MeshPrimitive::TriangleStrip || out.primitive() == MeshPrimitive::TriangleFan) {
        if(out.isIndexed()) out = MeshTools::duplicate(out);
        out = MeshTools::generateIndices(std::move(out));
    }

    /** @todo What about other primitive types? */
    // optimize() errors out if we don't have a triangle mesh
    // we shouldn't care at this point
    // but really we don't need it in optimize() if those functions aren't used
    // (e.g. decode only). then for vertex-optimization only we don't need to care
    // about the primitive type
    // -> more granular checks, then decode

    /* Convert to a contiguous index array if it's not */
    if(out.isIndexed() && Short(meshIndexTypeSize(out.indexType())) != out.indexStride()) {
        /** @todo This needs some more efficient tool, it's not wise to do a
            minmax over the whole array just for a passthrough copy. It also
            will drop the initial offset, which may be HIGHLY undesirable. */
        out = MeshTools::compressIndices(std::move(out), out.indexType());
    }

    meshopt_VertexCacheStatistics vertexCacheStatsBefore;
    meshopt_VertexFetchStatistics vertexFetchStatsBefore;
    meshopt_OverdrawStatistics overdrawStatsBefore;
    Containers::Array<Vector3> positionStorage;
    Containers::StridedArrayView1D<const Vector3> positions;
    Containers::Optional<UnsignedInt> vertexSize;
    if(!optimize("Trade::MeshOptimizerSceneConverter::convert():", out, flags(), configuration(), positionStorage, positions, vertexSize, vertexCacheStatsBefore, vertexFetchStatsBefore, overdrawStatsBefore))
        return Containers::NullOpt;

    if(configuration().value<bool>("simplify") ||
       configuration().value<bool>("simplifySloppy"))
    {
        const UnsignedInt targetIndexCount = out.indexCount()*configuration().value<Float>("simplifyTargetIndexCountThreshold");
        const Float targetError = configuration().value<Float>("simplifyTargetError");

        /* In this case meshoptimizer doesn't provide overloads, so let's do
           this on our side instead */
        Containers::Array<UnsignedInt> inputIndicesStorage;
        Containers::ArrayView<const UnsignedInt> inputIndices;
        if(out.indexType() == MeshIndexType::UnsignedInt)
            inputIndices = out.indices<UnsignedInt>().asContiguous();
        else {
            inputIndicesStorage = out.indicesAsArray();
            inputIndices = inputIndicesStorage;
        }

        Containers::Array<UnsignedInt> outputIndices;
        Containers::arrayResize<ArrayAllocator>(outputIndices, NoInit, out.indexCount());

        /* The nullptr at the end is not needed but without it GCC's
           -Wzero-as-null-pointer-constant fires due to the default argument
           being `= 0`. WHAT THE FUCK, how is this warning useful?! Why
           everything today feels like hastily patched together by incompetent
           idiots?! */
        UnsignedInt vertexCount;
        if(configuration().value<bool>("simplifySloppy")) {
            vertexCount = meshopt_simplifySloppy(outputIndices.data(), inputIndices.data(), out.indexCount(), static_cast<const float*>(positions.data()), out.vertexCount(), positions.stride(), targetIndexCount
                #if MESHOPTIMIZER_VERSION >= 160
                , targetError, nullptr
                #endif
            );
        } else {
            vertexCount = meshopt_simplify(outputIndices.data(), inputIndices.data(), out.indexCount(), static_cast<const float*>(positions.data()), out.vertexCount(), positions.stride(), targetIndexCount, targetError
                #if MESHOPTIMIZER_VERSION >= 160
                , nullptr
                #endif
            );
        }

        Containers::arrayResize<ArrayAllocator>(outputIndices, vertexCount);

        /* Take the original mesh vertex data with the reduced index buffer and
           call combineIndexedAttributes() to throw away the unused vertices */
        /** @todo provide a way to use the new vertices with the original
            vertex buffer for LODs */
        MeshIndexData indices{outputIndices};
        out = MeshData{out.primitive(),
            Containers::arrayAllocatorCast<char, ArrayAllocator>(std::move(outputIndices)), indices,
            out.releaseVertexData(), out.releaseAttributeData()};
        out = MeshTools::combineIndexedAttributes({out});

        /* If we're printing stats after, repopulate the positions to avoid
           using a now-gone array */
        if(flags() & SceneConverterFlag::Verbose)
            populatePositions(out, positionStorage, positions);
    }

    /* Print before & after stats if verbose output is requested */
    if(flags() & SceneConverterFlag::Verbose)
        analyzePost("Trade::MeshOptimizerSceneConverter::convert():", out, configuration(), positions, vertexSize, vertexCacheStatsBefore, vertexFetchStatsBefore, overdrawStatsBefore);

    if(configuration().value<bool>("encodeFilter")) {
#if !MESHOPT_HAS_FILTER_ENCODING
        Error{} << "Trade::MeshOptimizerSceneConverter::convert(): filter encoding requires meshoptimizer 0.17 or higher";
        return Containers::NullOpt;
#else
        /** @todo

        Which attributes are filtered how? Check gltfpack source

        config option:
            encodeFilterAttribute=<MeshAttribute>
        add some sane defaults
        this should work with implementation-specific values

        meshopt_encodeFilterOct for unit vectors -> normal, tangent, bitangent (input is tightly packed Vector4, ehhhh)
        meshopt_encodeFilterQuat for normalized quaternions -> no known VertexFormat
        meshopt_encodeFilterExp for arbitrary float vectors -> position, color?, texture coordinates?

        Do we unpack interleaved attributes or just return with an error?

        Output must be separated into one blob per filtered attribute (multiple of the same could be interleaved?), and
        one blob with the rest, but packed again. Sigh. Is there something in MeshTools?
        */

        for(UnsignedInt i = 0; i != configuration().valueCount("encodeFilterAttribute"); ++i) {
            // TODO use attribute indices here, there is no fromString
            // check range first :eyes:
            const MeshAttribute name = configuration().value<MeshAttribute>("encodeFilterAttribute", i);
            for(UnsignedInt j = 0; j != out.attributeCount(name); ++j) {
                VertexFormat format = out.attributeFormat(name, j);
                if(isVertexFormatImplementationSpecific(format)) {
                    // TODO warning
                    continue;
                }

                if(vertexFormatComponentFormat(format) != VertexFormat::Float) {
                    // TODO warning
                    continue;
                }

                const auto data = Containers::arrayCast<Float>(out.attribute(name, j));
                if(!data.isContiguous()) {
                    // TODO deinterleave, pad to Vector4 for normal, tangent, bitangent
                    continue;
                }

                AttributeEncodings type;
                Containers::Array<char> filtered;
                if(name == MeshAttribute::Normal || name == MeshAttribute::Tangent || name == MeshAttribute::Bitangent) {
                    // TODO meshopt expects they're normalized, document this
                    type = AttributeEncoding::FilteredOctahedral;
                    // TODO how to make this configurable? encodeFilterOctahedralStride, encodeFilterOctahedralBits
                    format = VertexFormat::Vector4bNormalized; // Vector4sNormalized
                    const UnsignedInt stride = vertexFormatSize(format);
                    const UnsignedInt bits = 16;
                    filtered = Containers::Array<char>{NoInit, out.vertexCount()*stride};
                    meshopt_encodeFilterOct(filtered, out.vertexCount(), stride, bits, data.asContiguous());
                } else {
                    type = AttributeEncoding::FilteredExponential;
                    // TODO encodeFilterExponentialBits
                    const UnsignedInt stride = vertexFormatSize(format);
                    const UnsignedInt bits = 24;
                    filtered = Containers::Array<char>{NoInit, out.vertexCount()*stride};
                    meshopt_encodeFilterExp(filtered, out.vertexCount(), stride, bits, data.asContiguous());
                }

                // TODO combine data
                // TODO set new data and formats
                format = vertexFormatWrap(type);
            }
        }
#endif
    }

#if MESHOPT_HAS_COMPRESSION
    if(configuration().value<bool>("encodeVertexBuffer") && out.attributeCount()) {
        Containers::Array<MeshAttributeData> attributes{NoInit, out.attributeCount()};
        for(UnsignedInt i = 0; i != out.attributeCount(); ++i) {
            // TODO how to handle implementation-specific formats?
            const VertexFormat encodedFormat = !isVertexFormatImplementationSpecific(out.attributeFormat(i)) ?
                vertexFormatWrap(out.attributeFormat(i)) : out.attributeFormat(i);
            attributes[i] = MeshAttributeData{out.attributeName(i), encodedFormat,
                out.attributeOffset(i), out.vertexCount(), out.attributeStride(i),
                out.attributeArraySize(i)};
        }

        // TODO which attributes to encode? -> config option
        // encodeVertexBufferAttribute=<MeshAttribute>
        // add some sane defaults

        // TODO does it make any sense to allow encoding > 1 vertex streams?
        // we'd have to identify which blobs belong together, is that even
        // reasonably doable?
        // data is already interleaved... no use trying

        // TODO
        //assert(vertex_size > 0 && vertex_size <= 256);
        //assert(vertex_size % 4 == 0);

        /* If not set, this returns 0 -- the first codec version. Decodable by
           all previous and future versions. */
        const int version = configuration().value<int>("encodeVertexVersion");
        /** @todo meshopt asserts that version is valid, should we produce an
            error to prevent the assert? Then we'd have to update the check for
            every new meshopt version that updates the encoding algorithm.
            We could use MESHOPTIMIZER_VERSION to at least perform the check
            on known library versions. */
        meshopt_encodeVertexVersion(version);

        const std::size_t vertexSize = out.attributeStride(0);
        const std::size_t maxEncodedSize = meshopt_encodeVertexBufferBound(out.vertexCount(), vertexSize);
        // TODO do we really want the interleaved data? it includes the min offset, we'd have to
        // patch that...
        const Containers::StridedArrayView2D<const char> interleavedData = MeshTools::interleavedData(out);

        Containers::Array<char> encoded{NoInit, maxEncodedSize};
        /* TODO does data() work if all attributes start at a non-0 offset? */
        const std::size_t encodedSize = meshopt_encodeVertexBuffer(reinterpret_cast<unsigned char*>(encoded.data()), encoded.size(), static_cast<const unsigned char*>(interleavedData.data()), out.vertexCount(), vertexSize);

        /* Return value is 0 when the buffer doesn't hold enough space, but we
           got the upper bound from meshopt */
        CORRADE_INTERNAL_ASSERT(encodedSize > 0 && encodedSize <= encoded.size());

        Containers::arrayResize(encoded, encodedSize);
        Containers::arrayShrink(encoded);

        out = MeshData{out.primitive(),
            out.indexDataFlags(), out.indexData(), MeshIndexData{out.indices()},
            std::move(encoded), std::move(attributes), out.vertexCount()};
    }

    if(configuration().value<bool>("encodeIndexBuffer") && out.isIndexed()) {
        CORRADE_INTERNAL_ASSERT(out.primitive() == MeshPrimitive::Triangles);

        if(out.indexCount()%3 != 0) {
            // TODO error
            return Containers::NullOpt;
        }

        const int version = configuration().value<int>("encodeIndexVersion");
        meshopt_encodeIndexVersion(version);

        const std::size_t maxEncodedSize = meshopt_encodeIndexBufferBound(out.indexCount(), out.vertexCount());

        Containers::Array<char> encoded{NoInit, maxEncodedSize};
        std::size_t encodedSize;
        if(out.indexType() == MeshIndexType::UnsignedInt) {
            encodedSize = meshopt_encodeIndexBuffer(reinterpret_cast<unsigned char*>(encoded.data()), encoded.size(), out.indices<UnsignedInt>().data(), out.indexCount());
        } else if(out.indexType() == MeshIndexType::UnsignedShort) {
            encodedSize = meshopt_encodeIndexBuffer(reinterpret_cast<unsigned char*>(encoded.data()), encoded.size(), out.indices<UnsignedShort>().data(), out.indexCount());
        } else if(out.indexType() == MeshIndexType::UnsignedByte) {
            encodedSize = meshopt_encodeIndexBuffer(reinterpret_cast<unsigned char*>(encoded.data()), encoded.size(), out.indices<UnsignedByte>().data(), out.indexCount());
        } else CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */

        /* Return value is 0 when the buffer doesn't hold enough space, but we
           got the upper bound from meshopt */
        CORRADE_INTERNAL_ASSERT(encodedSize > 0 && encodedSize <= encoded.size());

        Containers::arrayResize(encoded, encodedSize);
        Containers::arrayShrink(encoded);

        // TODO set implementation-specific index type
        // TODO set stride to 0
        // WARNING meshopt converts all index data to 32-bit, the output should
        // probably always be UnsignedInt? Or is this pure user choice? If so,
        // set it to UnsignedShort if the original type was that, and force
        // UnsignedByte to UnsignedShort because 8-bit decoding is not allowed.
        // BETTER use min of type and vertexCount as an upper limit
        MeshIndexType indexType = MeshIndexType::UnsignedInt;

        const MeshIndexData indices{indexType, encoded};
        out = MeshData{out.primitive(), std::move(encoded), indices,
            out.vertexDataFlags(), out.vertexData(), out.releaseAttributeData(), out.vertexCount()};
    }
#endif

    /* GCC 4.8 needs an explicit conversion, otherwise it tries to copy the
       thing and fails */
    return Containers::optional(std::move(out));
}

}}

CORRADE_PLUGIN_REGISTER(MeshOptimizerSceneConverter, Magnum::Trade::MeshOptimizerSceneConverter,
    "cz.mosra.magnum.Trade.AbstractSceneConverter/0.1.1")
