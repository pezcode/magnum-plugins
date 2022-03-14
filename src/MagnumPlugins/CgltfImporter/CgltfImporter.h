#ifndef Magnum_Trade_CgltfImporter_h
#define Magnum_Trade_CgltfImporter_h
/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                2020, 2021, 2022 Vladimír Vondruš <mosra@centrum.cz>
    Copyright © 2021 Pablo Escobar <mail@rvrs.in>

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

/** @file
 * @brief Class @ref Magnum::Trade::CgltfImporter
 * @m_since_latest_{plugins}
 */

#include <Magnum/Trade/AbstractImporter.h>

#include "MagnumPlugins/CgltfImporter/configure.h"

namespace Magnum { namespace Trade {

#ifndef DOXYGEN_GENERATING_OUTPUT
#ifndef MAGNUM_CGLTFIMPORTER_BUILD_STATIC
    #ifdef CgltfImporter_EXPORTS
        #define MAGNUM_CGLTFIMPORTER_EXPORT CORRADE_VISIBILITY_EXPORT
    #else
        #define MAGNUM_CGLTFIMPORTER_EXPORT CORRADE_VISIBILITY_IMPORT
    #endif
#else
    #define MAGNUM_CGLTFIMPORTER_EXPORT CORRADE_VISIBILITY_STATIC
#endif
#define MAGNUM_CGLTFIMPORTER_LOCAL CORRADE_VISIBILITY_LOCAL
#else
#define MAGNUM_CGLTFIMPORTER_EXPORT
#define MAGNUM_CGLTFIMPORTER_LOCAL
#endif

/**
@brief Cgltf importer plugin
@m_since_latest_{plugins}

@m_keywords{GltfImporter}

Imports glTF and binary glTF using the [cgltf](https://github.com/jkuhlmann/cgltf)
library.

This plugin provides the `GltfImporter` plugin.

@m_class{m-block m-success}

@thirdparty This plugin makes use of the [cgltf](https://github.com/jkuhlmann/cgltf)
    library, licensed under @m_class{m-label m-success} **MIT**
    ([license text](https://github.com/jkuhlmann/cgltf/blob/master/LICENSE),
    [choosealicense.com](https://choosealicense.com/licenses/mit/)).
    It requires attribution for public use. Cgltf itself uses
    [jsmn](https://github.com/nlohmann/json), licensed under
    @m_class{m-label m-success} **MIT** as well
    ([license text](https://github.com/zserge/jsmn/blob/master/LICENSE)).

@section Trade-CgltfImporter-usage Usage

This plugin depends on the @ref Trade library and the @ref AnyImageImporter
plugin and is built if `WITH_CGLTFIMPORTER` is enabled when building Magnum
Plugins. To use as a dynamic plugin, load @cpp "CgltfImporter" @ce via
@ref Corrade::PluginManager::Manager.

Additionally, if you're using Magnum as a CMake subproject, bundle the
[magnum-plugins repository](https://github.com/mosra/magnum-plugins) and do the
following:

@code{.cmake}
set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(magnum EXCLUDE_FROM_ALL)

set(WITH_CGLTFIMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(magnum-plugins EXCLUDE_FROM_ALL)

# So the dynamically loaded plugin gets built implicitly
add_dependencies(your-app MagnumPlugins::CgltfImporter)
@endcode

To use as a static plugin or as a dependency of another plugin with CMake, put
[FindMagnumPlugins.cmake](https://github.com/mosra/magnum-plugins/blob/master/modules/FindMagnumPlugins.cmake)
into your `modules/` directory, request the `CgltfImporter` component of the
`MagnumPlugins` package and link to the `MagnumPlugins::CgltfImporter`
target:

@code{.cmake}
find_package(MagnumPlugins REQUIRED CgltfImporter)

# ...
target_link_libraries(your-app PRIVATE MagnumPlugins::CgltfImporter)
@endcode

See @ref building-plugins, @ref cmake-plugins, @ref plugins and
@ref file-formats for more information.

@section Trade-CgltfImporter-behavior Behavior and limitations

The plugin supports @ref ImporterFeature::OpenData and
@ref ImporterFeature::FileCallback features. All buffers are loaded on-demand
and kept in memory for any later access. As a result, external file loading
callbacks are called with @ref InputFileCallbackPolicy::LoadPermanent.
Resources returned from file callbacks can only be safely freed after closing
the importer instance. In case of images, the files are loaded on-demand inside
@ref image2D() calls with @ref InputFileCallbackPolicy::LoadTemporary and
@ref InputFileCallbackPolicy::Close is emitted right after the file is fully
read.

Error reporting by cgltf is rather rudimentary, resulting in vague error
messages and no line numbers for several classes of errors, including
out-of-bounds indices and missing required attributes. If you need more
detailed errors, consider using the [glTF Validator](https://github.khronos.org/glTF-Validator/).

The content of the global [extensionsRequired](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#specifying-extensions)
array is checked against all extensions supported by the plugin. If a glTF file
requires an unknown extension, the import will fail. This behaviour can be
disabled with the @cb{.ini} ignoreRequiredExtensions @ce
@ref Trade-CgltfImporter-configuration "configuration option".

Import of morph data is not supported at the moment.

@subsection Trade-CgltfImporter-behavior-objects Scene import

-   If no @cb{.json} "scene" @ce property is present and the file contains at
    least one scene, @ref defaultScene() returns @cpp 0 @ce instead of
    @cpp -1 @ce. According to the [glTF 2.0 specification](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#scenes)
    the importer is free to not render anything, but the suggested behavior
    would break even some official sample models.
-   Imported scenes always have @ref SceneMappingType::UnsignedInt and are
    always 3D. The @ref objectCount() returns count of all nodes in the file,
    while @ref SceneData::mappingBound() returns an upper bound on node IDs
    contained in a particular scene.
-   Nodes that are not referenced by any scene are ignored.
-   All objects contained in a scene have a @ref SceneField::Parent (of type
    @ref SceneFieldType::Int). Size of this field is the count of nodes
    contained in the scene. The mapping is unordered and may be sparse if the
    file contains multiple scenes or nodes not referenced by any scene.
-   All nodes that contain transformation matrices or TRS components have a
    @ref SceneField::Transformation (of type @ref SceneFieldType::Matrix4x4).
    This field is not present if all such nodes have TRS components, in which
    a matrix is considered redundant. Nodes that don't have any transformation
    matrix nor a TRS component don't have this field assigned.
-   If any node contains a translation, a @ref SceneField::Translation (of type
    @ref SceneFieldType::Vector3) is present; if any node contains a rotation,
    a @ref SceneField::Rotation (of type @ref SceneFieldType::Quaternion) is
    present; if any node contains a scaling, a @ref SceneField::Scaling (of
    type @ref SceneFieldType::Vector3) is present.
-   If the scene references meshes, a @ref SceneField::Mesh (of type
    @ref SceneFieldType::UnsignedInt) is present. If any of the referenced
    meshes have assigned materials, @ref SceneField::MeshMaterial (of type
    @ref SceneFieldType::Int) is present as well. While a single node can only
    reference a single mesh at most, in case it references a multi-primitive
    mesh, it's represented as several @ref SceneField::Mesh (and
    @ref SceneField::MeshMaterial) assignments. See
    @ref Trade-CgltfImporter-behavior-meshes and
    @ref Trade-CgltfImporter-behavior-materials for further details.
-   If the scene references skins, a @ref SceneField::Skin (of type
    @ref SceneFieldType::UnsignedInt) is present. A single node can only
    reference one skin at most. See
    @ref Trade-CgltfImporter-behavior-animations for further details.
-   If the scene references cameras, a @ref SceneField::Camera (of type
    @ref SceneFieldType::UnsignedInt) is present. A single node can only
    reference one camera at most. See
    @ref Trade-CgltfImporter-behavior-cameras for further details.
-   If the scene references lights, a @ref SceneField::Light (of type
    @ref SceneFieldType::UnsignedInt) is present. A single node can only
    reference one light at most. See
    @ref Trade-CgltfImporter-behavior-lights for further details.
-   If node rotation quaternion is not normalized, the importer prints a
    warning and normalizes it. Can be disabled per-object with the
    @cb{.ini} normalizeQuaternions @ce
    @ref Trade-CgltfImporter-configuration "configuration option".

@subsection Trade-CgltfImporter-behavior-animations Animation and skin import

-   Linear quaternion rotation tracks are postprocessed in order to make it
    possible to use the faster
    @ref Math::lerp(const Quaternion<T>&, const Quaternion<T>&, T) "Math::lerp()" /
    @ref Math::slerp(const Quaternion<T>&, const Quaternion<T>&, T) "Math::slerp()"
    functions instead of
    @ref Math::lerpShortestPath(const Quaternion<T>&, const Quaternion<T>&, T) "Math::lerpShortestPath()" /
    @ref Math::slerpShortestPath(const Quaternion<T>&, const Quaternion<T>&, T) "Math::slerpShortestPath()". Can be disabled per-animation with the
    @cb{.ini} optimizeQuaternionShortestPath @ce
    @ref Trade-CgltfImporter-configuration "configuration option". This doesn't
    affect spline-interpolated rotation tracks.
-   If linear quaternion rotation tracks are not normalized, the importer
    prints a warning and normalizes them. Can be disabled per-animation with
    the @cb{.ini} normalizeQuaternions @ce
    @ref Trade-CgltfImporter-configuration "configuration option". This doesn't
    affect spline-interpolated rotation tracks.
-   Skin `skeleton` property is not imported
-   Morph targets are not supported
-   Animation tracks are always imported with
    @ref Animation::Extrapolation::Constant, because glTF doesn't support
    anything else
-   It's possible to request all animation clips to be merged into one using
    the @cb{.ini} mergeAnimationClips @ce option in order to for example
    preserve cinematic animations when using the Blender glTF exporter (as it
    otherwise outputs a separate clip for each object). When this option is
    enabled, @ref animationCount() always report either @cpp 0 @ce or
    @cpp 1 @ce and the merged animation has no name. With this option enabled,
    however, it can happen that multiple conflicting tracks affecting the same
    node are merged in the same clip, causing the animation to misbehave.

@subsection Trade-CgltfImporter-behavior-cameras Camera import

-   Cameras in glTF are specified with vertical FoV and vertical:horizontal
    aspect ratio, these values are recalculated for horizontal FoV and
    horizontal:vertical aspect ratio as is common in Magnum

@subsection Trade-CgltfImporter-behavior-lights Light import

-   The importer supports the [KHR_lights_punctual](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_lights_punctual/README.md)
    extension

@subsection Trade-CgltfImporter-behavior-meshes Mesh import

-   Indices are imported as either @ref MeshIndexType::UnsignedByte,
    @ref MeshIndexType::UnsignedShort or @ref MeshIndexType::UnsignedInt
-   Positions are imported as @ref VertexFormat::Vector3,
    @ref VertexFormat::Vector3ub, @ref VertexFormat::Vector3b,
    @ref VertexFormat::Vector3us, @ref VertexFormat::Vector3s,
    @ref VertexFormat::Vector3ubNormalized,
    @ref VertexFormat::Vector3bNormalized,
    @ref VertexFormat::Vector3usNormalized or
    @ref VertexFormat::Vector3sNormalized (which includes the additional types
    specified by [KHR_mesh_quantization](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_mesh_quantization/README.md))
-   Normals are imported as @ref VertexFormat::Vector3,
    @ref VertexFormat::Vector3bNormalized or
    @ref VertexFormat::Vector3sNormalized
-   Tangents are imported as @ref VertexFormat::Vector4,
    @ref VertexFormat::Vector4bNormalized or
    @ref VertexFormat::Vector4sNormalized
-   Texture coordinates are imported as @ref VertexFormat::Vector2,
    @ref VertexFormat::Vector2ub, @ref VertexFormat::Vector2b,
    @ref VertexFormat::Vector2us, @ref VertexFormat::Vector2s,
    @ref VertexFormat::Vector2ubNormalized,
    @ref VertexFormat::Vector2bNormalized,
    @ref VertexFormat::Vector2usNormalized or
    @ref VertexFormat::Vector2sNormalized (which includes the additional types
    specified by [KHR_mesh_quantization](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_mesh_quantization/README.md)). The
    data are by default Y-flipped on import unless
    @cb{.ini} textureCoordinateYFlipInMaterial @ce is either explicitly
    enabled, or if the file contains non-normalized integer or normalized
    signed integer texture coordinates (which can't easily be flipped). In that
    case texture coordinate data are kept as-is and materials provide a texture
    transformation that does the Y-flip instead.
-   Colors are imported as @ref VertexFormat::Vector3,
    @ref VertexFormat::Vector4,
    @ref VertexFormat::Vector3ubNormalized,
    @ref VertexFormat::Vector4ubNormalized,
    @ref VertexFormat::Vector3usNormalized or
    @ref VertexFormat::Vector4usNormalized
-   Joint IDs and weights for skinning are imported as custom vertex attributes
    named "JOINTS" and "WEIGHTS". Their mapping to/from a string can be queried
    using @ref meshAttributeName() and @ref meshAttributeForName().
    Joint IDs are imported as @ref VertexFormat::Vector4ub or
    @ref VertexFormat::Vector4us. Joint weights are imported as
    @ref VertexFormat::Vector4, @ref VertexFormat::Vector4ubNormalized or
    @ref VertexFormat::Vector4usNormalized.
-   Per-vertex object ID attribute is imported as either
    @ref VertexFormat::UnsignedInt, @ref VertexFormat::UnsignedShort or
    @ref VertexFormat::UnsignedByte. By default `_OBJECT_ID` is the recognized
    name, use the @cb{.ini} objectIdAttribute @ce
    @ref Trade-CgltfImporter-configuration "configuration option" to change
    the identifier that's being looked for.
-   Multi-primitive meshes are split into individual meshes, nodes that
    reference a multi-primitive mesh have multiple @ref SceneField::Mesh
    (and @ref SceneField::MeshMaterial) entries in the imported @ref SceneData.
-   Attribute-less meshes either with or without an index buffer are supported,
    however since glTF has no way of specifying vertex count for those,
    returned @ref Trade::MeshData::vertexCount() is set to @cpp 0 @ce

Custom and unrecognized vertex attributes of allowed types are present in the
imported meshes as well. Their mapping to/from a string can be queried using
@ref meshAttributeName() and @ref meshAttributeForName(). Attributes with
unsupported types (such as non-normalized integer matrices) cause the import to
fail.

@subsection Trade-CgltfImporter-behavior-materials Material import

-   If present, builtin [metallic/roughness](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#metallic-roughness-material) material is imported,
    setting @ref MaterialType::PbrMetallicRoughness on the @ref MaterialData.
-   If the [KHR_materials_pbrSpecularGlossiness](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Archived/KHR_materials_pbrSpecularGlossiness/README.md)
    extension is present, its properties are imported with
    @ref MaterialType::PbrSpecularGlossiness present in material types.
-   Additional normal, occlusion and emissive maps are imported, together with
    related properties
-   If the [KHR_materials_unlit](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_unlit/README.md)
    extension is present, @ref MaterialType::Flat is set in material types,
    replacing @ref MaterialType::PbrMetallicRoughness or
    @ref MaterialType::PbrSpecularGlossiness.
-   If the [KHR_materials_clearcoat](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_clearcoat/README.md)
    extension is present, @ref MaterialType::PbrClearCoat is set in material
    types, and a new layer with clearcoat properties is added
-   Custom texture coordinate sets as well as [KHR_texture_transform](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_texture_transform/README.md)
    properties are imported on all textures.
-   Unrecognized material extensions are imported as custom layers with a `#`
    prefix. Extension properties are imported with their raw names and types,
    the following of which are supported:
    -   @ref MaterialAttributeType::String
    -   @ref MaterialAttributeType::Bool
    -   All numbers as @ref MaterialAttributeType::Float to avoid inconsistency
        with different glTF exporters. The only exception are texture indices
        and coordinate sets inside [textureInfo](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-textureinfo)
        objects, which get imported as @ref MaterialAttributeType::UnsignedInt,
        consistently with types of builtin `*Texture` and `*TextureCoordinates`
        @ref MaterialAttribute entries.
    -   Number arrays as @ref MaterialAttributeType::Vector2 /
        @ref MaterialAttributeType::Vector3 /
        @ref MaterialAttributeType::Vector4.
        Empty arrays, arrays of size 5 or higher as well as arrays containing
        anything that isn't a number are ignored.
    -   [textureInfo](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-textureinfo)
        objects, including all attributes handled for regular textures. Texture
        attributes are prefixed by the name of the object: e.g. if an extension
        has a `someTexture` property, the texture index, matrix, coordinate set
        and scale would be imported as `someTexture`, `someTextureMatrix`,
        `someTextureCoordinates` and `someTextureScale`, consistently with
        builtin texture-related @ref MaterialAttribute names. Non-texture
        object types are ignored.
    If you handle any of these custom material extensions, it may make sense
    to enable the @cb{.ini} ignoreRequiredExtensions @ce
    @ref Trade-CgltfImporter-configuration "configuration option".
-   [Extras](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-extras)
    metadata is imported into the base material layer. The `extras` attribute
    must be an object, otherwise it's ignored with a warning. Type support is
    the same as for unrecognized material extensions, except for [textureInfo](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-textureinfo)
    objects --- contrary to glTF material extensions, where sub-objects can be
    assumed to contain texture info, the `extras` can contain just anything.
-   If the on-by-default @cb{.ini} phongMaterialFallback @ce
    @ref Trade-CgltfImporter-configuration "configuration option" is
    enabled, the importer provides a Phong fallback for backwards
    compatibility:
    -   @ref MaterialType::Phong is added to material types
    -   Base color and base color texture along with custom texture coordinate
        set and transformation, if present, is exposed as a diffuse color and
        texture, unless already present together with specular color / texture
        from the specular/glossiness material
    -   All other @ref PhongMaterialData values are is kept at their defaults

@subsection Trade-CgltfImporter-behavior-textures Texture and image import

<ul>
<li>Texture type is always @ref Trade::TextureType::Texture2D, as glTF doesn't
support anything else</li>
<li>Z coordinate of @ref Trade::TextureData::wrapping() is always
@ref SamplerWrapping::Repeat, as glTF doesn't support 3D textures</li>
<li>
@m_class{m-nopadb}

glTF leaves the defaults of sampler properties to the application, the
following defaults have been chosen for this importer:

-   Minification/magnification/mipmap filter: @ref SamplerFilter::Linear,
    @ref SamplerMipmap::Linear
-   Wrapping (all axes): @ref SamplerWrapping::Repeat
</li>
<li>
    The importer supports the following extensions for image types not defined
    in the [core glTF 2.0 specification](https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#gltf-basics):
    [MSFT_texture_dds](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Vendor/MSFT_texture_dds/README.md)
    for DirectDraw Surface images (`*.dds`),
    [KHR_texture_basisu](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_texture_basisu/README.md)
    for Khronos Texture 2.0 images (`*.ktx2`) with [Basis Universal](https://github.com/binomialLLC/basis_universal)
    supercompression, as well as the original provisional `GOOGLE_texture_basis`
    extension for referencing plain Basis Universal files (`*.basis`). There was
    no formal specification of the extension but the use is like below,
    [equivalently to Basis own glTF example](https://github.com/BinomialLLC/basis_universal/blob/1cae1d57266e2c95bc011b0bf1ccb9940988c184/webgl/gltf/assets/AgiHqSmall.gltf#L230-L240):

    @code{.json}
    {
        ...
        "textures": [
            {
                "extensions": {
                    "GOOGLE_texture_basis": {
                        "source": 0
                    }
                }
            }
        ],
        "images": [
            {
                "mimeType": "image/x-basis",
                "uri": "texture.basis"
            }
        ],
        "extensionsUsed": [
            "GOOGLE_texture_basis"
        ],
        "extensionsRequired": [
            "GOOGLE_texture_basis"
        ]
    }
    @endcode

    The MIME type (if one exists) is ignored by the importer. Delegation to the
    correct importer alias happens via @ref AnyImageImporter which uses the
    file extension or buffer content to determine the image type.
</li>
</ul>

@section Trade-CgltfImporter-configuration Plugin-specific config

It's possible to tune various output options through @ref configuration(). See
below for all options and their default values.

@snippet MagnumPlugins/CgltfImporter/CgltfImporter.conf config

See @ref plugins-configuration for more information and an example showing how
to edit the configuration values.
*/
class MAGNUM_CGLTFIMPORTER_EXPORT CgltfImporter: public AbstractImporter {
    public:
        /**
         * @brief Default constructor
         *
         * In case you want to open images, use
         * @ref CgltfImporter(PluginManager::Manager<AbstractImporter>&)
         * instead.
         */
        explicit CgltfImporter();

        /**
         * @brief Constructor
         *
         * The plugin needs access to plugin manager for importing images.
         */
        explicit CgltfImporter(PluginManager::Manager<AbstractImporter>& manager);

        /** @brief Plugin manager constructor */
        explicit CgltfImporter(PluginManager::AbstractManager& manager, const Containers::StringView& plugin);

        ~CgltfImporter();

    private:
        struct Document;

        MAGNUM_CGLTFIMPORTER_LOCAL ImporterFeatures doFeatures() const override;

        MAGNUM_CGLTFIMPORTER_LOCAL bool doIsOpened() const override;

        MAGNUM_CGLTFIMPORTER_LOCAL void doOpenData(Containers::Array<char>&& data, DataFlags dataFlags) override;
        MAGNUM_CGLTFIMPORTER_LOCAL void doOpenFile(const std::string& filename) override;
        MAGNUM_CGLTFIMPORTER_LOCAL void doClose() override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doAnimationCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doAnimationForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doAnimationName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<AnimationData> doAnimation(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doCameraCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doCameraForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doCameraName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<CameraData> doCamera(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doLightCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doLightForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doLightName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<LightData> doLight(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL Int doDefaultScene() const override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doSceneCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doSceneForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doSceneName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<SceneData> doScene(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedLong doObjectCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Long doObjectForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doObjectName(UnsignedLong id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doSkin3DCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doSkin3DForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doSkin3DName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<SkinData3D> doSkin3D(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doMeshCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doMeshForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doMeshName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<MeshData> doMesh(UnsignedInt id, UnsignedInt level) override;
        MAGNUM_CGLTFIMPORTER_LOCAL MeshAttribute doMeshAttributeForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doMeshAttributeName(UnsignedShort name) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doMaterialCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doMaterialForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doMaterialName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<MaterialData> doMaterial(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doTextureCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doTextureForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doTextureName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<TextureData> doTexture(UnsignedInt id) override;

        MAGNUM_CGLTFIMPORTER_LOCAL AbstractImporter* setupOrReuseImporterForImage(UnsignedInt id, const char* errorPrefix);

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doImage2DCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doImage2DLevelCount(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doImage2DForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doImage2DName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Optional<ImageData2D> doImage2D(UnsignedInt id, UnsignedInt level) override;

        Containers::Pointer<Document> _d;
};

}}

#endif
