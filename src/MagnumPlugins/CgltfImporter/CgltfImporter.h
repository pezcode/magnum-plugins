#ifndef Magnum_Trade_CgltfImporter_h
#define Magnum_Trade_CgltfImporter_h
/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                2020, 2021 Vladimír Vondruš <mosra@centrum.cz>
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
        explicit CgltfImporter(PluginManager::AbstractManager& manager, const std::string& plugin);

        ~CgltfImporter();

    private:
        struct Document;

        MAGNUM_CGLTFIMPORTER_LOCAL ImporterFeatures doFeatures() const override;

        MAGNUM_CGLTFIMPORTER_LOCAL bool doIsOpened() const override;

        MAGNUM_CGLTFIMPORTER_LOCAL void doOpenData(Containers::ArrayView<const char> data) override;
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

        MAGNUM_CGLTFIMPORTER_LOCAL UnsignedInt doObject3DCount() const override;
        MAGNUM_CGLTFIMPORTER_LOCAL Int doObject3DForName(const std::string& name) override;
        MAGNUM_CGLTFIMPORTER_LOCAL std::string doObject3DName(UnsignedInt id) override;
        MAGNUM_CGLTFIMPORTER_LOCAL Containers::Pointer<ObjectData3D> doObject3D(UnsignedInt id) override;

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
