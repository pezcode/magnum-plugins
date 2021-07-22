EAPI=7

EGIT_REPO_URI="git://github.com/mosra/magnum-plugins.git"

inherit cmake git-r3

DESCRIPTION="Plugins for the Magnum C++11/C++14 graphics engine"
HOMEPAGE="https://magnum.graphics"

LICENSE="MIT"
SLOT="0"
KEYWORDS="~amd64 ~x86"
IUSE=""

RDEPEND="
	dev-libs/magnum
	media-libs/devil
	media-libs/faad2
	media-libs/freetype
	media-libs/harfbuzz
	media-libs/openexr
	virtual/jpeg
	media-libs/libpng
	media-libs/assimp
	dev-util/glslang
	dev-util/spirv-tools
"
DEPEND="${RDEPEND}"

src_configure() {
	# general configuration
	local mycmakeargs=(
		-DCMAKE_INSTALL_PREFIX="${EPREFIX}/usr"
		-DCMAKE_BUILD_TYPE=Release
		-DWITH_ASSIMPIMPORTER=ON
		-DWITH_BASISIMAGECONVERTER=OFF
		-DWITH_BASISIMPORTER=OFF
		-DWITH_DDSIMPORTER=ON
		-DWITH_DEVILIMAGEIMPORTER=ON
		-DWITH_DRFLACAUDIOIMPORTER=ON
		-DWITH_DRMP3AUDIOIMPORTER=ON
		-DWITH_DRWAVAUDIOIMPORTER=ON
		-DWITH_FAAD2AUDIOIMPORTER=ON
		-DWITH_FREETYPEFONT=ON
		-DWITH_GLSLANGSHADERCONVERTER=ON
		-DWITH_HARFBUZZFONT=ON
		-DWITH_ICOIMPORTER=ON
		-DWITH_JPEGIMAGECONVERTER=ON
		-DWITH_JPEGIMPORTER=ON
        -DWITH_KTXIMAGECONVERTER=ON
        -DWITH_KTXIMPORTER=ON
		-DWITH_MESHOPTIMIZERSCENECONVERTER=OFF
		-DWITH_MINIEXRIMAGECONVERTER=ON
		-DWITH_OPENEXRIMAGECONVERTER=ON
		-DWITH_OPENEXRIMPORTER=ON
		-DWITH_OPENGEXIMPORTER=ON
		-DWITH_PNGIMAGECONVERTER=ON
		-DWITH_PNGIMPORTER=ON
		-DWITH_PRIMITIVEIMPORTER=ON
		-DWITH_SPIRVTOOLSSHADERCONVERTER=ON
		-DWITH_STANFORDIMPORTER=ON
		-DWITH_STANFORDSCENECONVERTER=ON
		-DWITH_STBDXTIMAGECONVERTER=ON
		-DWITH_STBIMAGECONVERTER=ON
		-DWITH_STBIMAGEIMPORTER=ON
		-DWITH_STBTRUETYPEFONT=ON
		-DWITH_STBVORBISAUDIOIMPORTER=ON
		-DWITH_STLIMPORTER=ON
		-DWITH_TINYGLTFIMPORTER=ON
	)
	cmake_src_configure
}

src_install() {
	cmake_src_install
	mkdir "${ED}/usr/$(get_libdir)/magnum/"
	cp -av "${BUILD_DIR}"/Gentoo/lib/magnum/* "${ED}/usr/$(get_libdir)/magnum/" || die
}

# kate: replace-tabs off;
