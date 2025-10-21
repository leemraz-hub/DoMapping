
set(TOOLCHAIN_DIR "/DoMapping/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${TOOLCHAIN_DIR}/cmake-3.12.0-Linux-x86_64/share/cmake-3.12/Modules )

set(EXTERNEL_LIBRARY ${TOOLCHAIN_DIR}/third_party_install)
set(INTERNAL_LIBRARY ${TOOLCHAIN_DIR}/install)


set(CMAKE_FIND_ROOT_PATH   ${TOOLCHAIN_DIR})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3  -Wall -fPIC -frounding-math -lm -lpthread -fopenmp")#  -lgomp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -lm -Wall -Wextra -Wundef -pedantic -Wno-enum-compare -Wno-deprecated -Wno-write-strings -funroll-loops -fPIC -frounding-math -fpermissive -lpthread")# -fopenmp -lgomp


## externel-library path
set(ZLIB_ROOT ${EXTERNEL_LIBRARY}/zlib-1.2.11)
set(PNG_ROOT ${EXTERNEL_LIBRARY}/libpng-1.6.35)
set(JPEG_ROOT ${EXTERNEL_LIBRARY}/libjpeg-turbo-2.0.4)
set(TIFF_ROOT ${EXTERNEL_LIBRARY}/tiff-4.0.4)


set(GEOTIFF_ROOT ${EXTERNEL_LIBRARY}/libgeotiff-1.4.2)
set(GEOTIFF_INCLUDE_DIR ${GEOTIFF_ROOT}/include)
set(GEOTIFF_LIBRARY 
	${GEOTIFF_ROOT}/lib/libgeotiff.a
	${GEOTIFF_ROOT}/lib/libxtiff.a)

set(PROJ4_INCLUDE_DIR ${EXTERNEL_LIBRARY}/proj-4.9.1/include)
set(PROJ4_LIBRARY ${EXTERNEL_LIBRARY}/proj-4.9.1/lib/libproj.a)
set(GDAL_DIR  ${EXTERNEL_LIBRARY}/gdal-2.2.3)
set(GDAL_LIBRARY  ${GDAL_DIR}/lib/libgdal.a)
set(GDAL_FOUND  ON)
set(GDAL_INCLUDE_DIR  ${GDAL_DIR}/include)

set(GEOS_INCLUDE_DIR   ${EXTERNEL_LIBRARY}/geos-3.1.0rc2/include)
set(GEOS_LIBRARY ${EXTERNEL_LIBRARY}/geos-3.1.0rc2/lib/libgeos_c.a ${EXTERNEL_LIBRARY}/geos-3.1.0rc2/lib/libgeos.a)

set(WEBP_DIR ${EXTERNEL_LIBRARY}/libwebp-1.2.0)
set(WEBP_LIBRARIES  ${WEBP_DIR}/lib/libwebp.a ${WEBP_DIR}/lib/libwebpdecoder.a ${WEBP_DIR}/lib/libwebpdemux.a ${WEBP_DIR}/lib/libwebpmux.a )
set(WEBP_INCLUDE_DIR  ${WEBP_DIR}/include)

set(SQLITE_INCLUDE_DIR   ${EXTERNEL_LIBRARY}/sqlite/include)
set(SQLITE_LIBRARY ${EXTERNEL_LIBRARY}/sqlite/lib/libsqlite3.a)

set(MQTT_INCLUDE_DIR   ${EXTERNEL_LIBRARY}/paho.mqtt.cpp-1.5.2/include)
set(MQTT_LIBRARY 
	${EXTERNEL_LIBRARY}/paho.mqtt.cpp-1.5.2/lib/libpaho-mqtt3a.a  
	${EXTERNEL_LIBRARY}/paho.mqtt.cpp-1.5.2/lib/libpaho-mqtt3as.a  
	${EXTERNEL_LIBRARY}/paho.mqtt.cpp-1.5.2/lib/libpaho-mqtt3c.a  
	${EXTERNEL_LIBRARY}/paho.mqtt.cpp-1.5.2/lib/libpaho-mqtt3cs.a  
	${EXTERNEL_LIBRARY}/paho.mqtt.cpp-1.5.2/lib/libpaho-mqttpp3.a
)

set(RTMP_INCLUDE_DIR ${EXTERNEL_LIBRARY}/rtmpdump-2.3/include/librtmp)
set(RTMP_LIBRARY ${EXTERNEL_LIBRARY}/rtmpdump-2.3/lib/librtmp.a)

set(FFMPEG_INCLUDE_DIRS 
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/include/libavcodec
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/include/libavformat
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/include/libavfilter
)
set(FFMPEG_LIBRARIES 
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/lib/libavcodec.so
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/lib/libavformat.so
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/lib/libavutil.so
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/lib/libswscale.so
	${EXTERNEL_LIBRARY}/FFmpeg-n4.2.1/lib/libavfilter.so
)

set(Boost_FOUND  ON)
set(BOOST_ROOT ${EXTERNEL_LIBRARY}/boost_1_70_0)
set(Boost_LIBRARY_DIRS ${BOOST_ROOT}/boost_1_70_0/lib)
set(Boost_INCLUDE_DIRS ${BOOST_ROOT}/boost_1_70_0/include)
set(Boost_LIBRARIES
${BOOST_ROOT}/lib/libboost_iostreams.so
${BOOST_ROOT}/lib/libboost_program_options.so
${BOOST_ROOT}/lib/libboost_system.so
${BOOST_ROOT}/lib/libboost_filesystem.so
${BOOST_ROOT}/lib/libboost_serialization.so
${BOOST_ROOT}/lib/libboost_thread.so
${BOOST_ROOT}/lib/libboost_regex.so
${BOOST_ROOT}/lib/libboost_chrono.so
${BOOST_ROOT}/lib/libboost_date_time.so
${BOOST_ROOT}/lib/libboost_atomic.so)

set(OPENCV_PATH  ${EXTERNEL_LIBRARY}/opencv-4.10.0/lib/cmake/opencv4)
set(TBB_ROOT_DIR ${EXTERNEL_LIBRARY}/tbb)
set(TBB_LIBRARY ${EXTERNEL_LIBRARY}/tbb/lib/libtbb.a)
set(TBB_INCLUDE_DIR ${EXTERNEL_LIBRARY}/tbb/include)

set(EIGEN_ROOT ${EXTERNEL_LIBRARY}/eigen3)
set(EIGEN_INCLUDE_DIR_HINT ${EXTERNEL_LIBRARY}/eigen3)
set(Eigen3_DIR ${EXTERNEL_LIBRARY}/eigen3)

set(JSONCPP_LIBRARY ${EXTERNEL_LIBRARY}/jsoncpp/lib/libjsoncpp.so)
set(JSONCPP_INCLUDE ${EXTERNEL_LIBRARY}/jsoncpp/include)
set(JSONCPP_INCLUDE_DIR ${JSONCPP_INCLUDE})
set(JSONC_LIBRARY ${EXTERNEL_LIBRARY}/json-c-0.12/lib/libjson-c.a)


set(ZSTD_INCLUDE_DIR ${EXTERNEL_LIBRARY}/zstd/include)
set(ZSTD_LIBRARY ${EXTERNEL_LIBRARY}/zstd/lib/libzstd.so)


set(OPENSSL_INCLUDE_DIR ${EXTERNEL_LIBRARY}/openssl/include)
set(OPENSSL_CRYPTO_LIBRARY ${EXTERNEL_LIBRARY}/openssl/lib/libcrypto.so)
set(OPENSSL_SSL_LIBRARY ${EXTERNEL_LIBRARY}/openssl/lib/libssl.so)


set(CURL_INCLUDE_DIR ${EXTERNEL_LIBRARY}/curl-7.70.0/include)
set(CURL_LIBRARY ${EXTERNEL_LIBRARY}/curl-7.70.0/lib/libcurl.so)

set(PDAL_DIR ${EXTERNEL_LIBRARY}/pdal/lib/cmake/PDAL)
set(LASZIP_INCLUDE ${EXTERNEL_LIBRARY}/laszip/include)
set(LASZIP_LIBRARY ${EXTERNEL_LIBRARY}/laszip/lib/liblaszip.so ${EXTERNEL_LIBRARY}/laszip/lib/liblaszip_api.so)


set(GMP_INCLUDE_DIR ${EXTERNEL_LIBRARY}/gmp-6.1.2/include)
set(GMP_LIBRARY ${EXTERNEL_LIBRARY}/gmp-6.1.2/lib/libgmp.a)

set(MPFR_INCLUDE_DIR ${EXTERNEL_LIBRARY}/mpfr-4.0.2/include)
set(MPFR_LIBRARY ${EXTERNEL_LIBRARY}/mpfr-4.0.2/lib/libmpfr.a)

set(CGAL_FOUND  ON)
set(CGAL_DIR  ${EXTERNEL_LIBRARY}/CGAL-4.13)
set(CGAL_INCLUDE_DIRS  ${EXTERNEL_LIBRARY}/CGAL-4.13/include)
set(CGAL_LIBRARY_DIRS  ${EXTERNEL_LIBRARY}/CGAL-4.13/lib)
set(CGAL_LIB  
	${CGAL_DIR}/lib/libCGAL.a 
	${CGAL_DIR}/lib/libCGAL_Core.a 
	${CGAL_DIR}/lib/libCGAL_ImageIO.a 
	${GMP_LIBRARY} ${MPFR_LIBRARY})


set(OpenCL_INCLUDE_DIR ${TOOLCHAIN_DIR}/opencv-4.5.1/3rdparty/include/opencl/1.2)
set(OpenCL_LIBRARY /usr/lib/x86_64-linux-gnu/libOpenCL.so)

set(Pangolin_DIR  ${TOOLCHAIN_DIR}/Pangolin/install/lib/cmake/Pangolin)