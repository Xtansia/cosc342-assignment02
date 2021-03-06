cmake_minimum_required(VERSION 3.0)
project(RayTracer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -O2")

set(SOURCE_FILES Camera.cpp Camera.h Colour.cpp Colour.h Cone.cpp Cone.h CSG.cpp CSG.h Direction.cpp Direction.h
        Display.cpp Display.h LightSource.cpp LightSource.h Material.h Matrix.cpp Matrix.h NonCopyable.h
        Normal.cpp Normal.h Object.cpp Object.h PinholeCamera.cpp PinholeCamera.h Point.cpp Point.h
        PointLightSource.cpp PointLightSource.h Ray.h RayIntersection.h rayTracerMain.cpp Scene.cpp Scene.h
        SceneReader.cpp SceneReader.h Sphere.cpp Sphere.h Transform.cpp Transform.h utility.h Vector.cpp Vector.h Cube.cpp Cube.h)
add_executable(rayTracer ${SOURCE_FILES})
target_link_libraries(rayTracer ${OpenCV_LIBS})

find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)
