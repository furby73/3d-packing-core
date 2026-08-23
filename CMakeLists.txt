cmake_minimum_required(VERSION 3.14)
project(3DPackingSolver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
    tinyobjloader
    GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader.git
    GIT_TAG        release
)
FetchContent_MakeAvailable(tinyobjloader)

set(SOURCES
    src/main.cpp
    src/Collisions.cpp
    src/MeshIO.cpp
)

add_executable(packer ${SOURCES})
target_link_libraries(packer PRIVATE tinyobjloader)

target_include_directories(packer PRIVATE src)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(packer PRIVATE -O3)
elseif(MSVC)
    target_compile_options(packer PRIVATE /O2 /Ob2 /Oi /Ot /fp:fast)
endif()