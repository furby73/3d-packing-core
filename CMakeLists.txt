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
target_compile_definitions(tinyobjloader PUBLIC TINYOBJLOADER_USE_DOUBLE)

set(SOURCES
    src/main.cpp
    src/Collisions.cpp
    src/MeshIO.cpp
    src/Packing.cpp
)

if(NOT DEFINED PACKER_VERSION_STRING)
    set(PACKER_VERSION_STRING "dev")
endif()

add_executable(packer ${SOURCES})
target_link_libraries(packer PRIVATE tinyobjloader)
target_compile_definitions(packer PRIVATE PACKER_VERSION="${PACKER_VERSION_STRING}")

target_include_directories(packer PRIVATE src)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(packer PRIVATE -O3)
elseif(MSVC)
    target_compile_options(packer PRIVATE /O2 /Ob2 /Oi /Ot /fp:precise)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(geometry_tests tests/geometry_tests.cpp src/Collisions.cpp src/Packing.cpp)
    target_include_directories(geometry_tests PRIVATE src)
    add_test(NAME geometry_and_packing COMMAND geometry_tests)
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME cli_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.py $<TARGET_FILE:packer>)
        add_test(NAME blender_result_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_result.py)
    endif()
endif()
