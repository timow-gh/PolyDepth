cmake_minimum_required(VERSION 3.21) # CMakePresets are supported since version 3.21

project(PolyDepth
        VERSION 1.0.0
        LANGUAGES CXX)

if ((CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR))
    set(IS_TOP_LEVEL_PROJECT TRUE)
else ()
    set(IS_TOP_LEVEL_PROJECT FALSE)
endif ()

if (IS_TOP_LEVEL_PROJECT)
    option(${PROJECT_NAME}_INSTALL "Install project targets" ON)
    option(${PROJECT_NAME}_BUILD_SAMPLES "Build samples" ON)
    option(${PROJECT_NAME}_PACKAGE "Use CPack to create an installer" ON)
else ()
    option(${PROJECT_NAME}_INSTALL "Install project targets" OFF)
    option(${PROJECT_NAME}_BUILD_SAMPLES "Build samples" OFF)
    option(${PROJECT_NAME}_PACKAGE "Use CPack to create an installer" OFF)
endif ()

add_subdirectory(dependencies)

set(
        HDRS
        ${CMAKE_CURRENT_LIST_DIR}/include/PolyDepth/ClearConfiguration.h
        ${CMAKE_CURRENT_LIST_DIR}/include/PolyDepth/MeshObject.h
        ${CMAKE_CURRENT_LIST_DIR}/include/PolyDepth/PolyDepth.h
)

set(
        SRCS
        ${CMAKE_CURRENT_LIST_DIR}/src/MeshObject.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/PolyDepth.cpp
)

add_library(polydepth STATIC ${SRCS} ${HDRS})
add_library(polydepth::polydepth ALIAS polydepth)

target_link_libraries(polydepth PUBLIC PQP C2A::C2A)

target_include_directories(
        polydepth
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(
        polydepth
        PROPERTIES
        DEBUG_POSTFIX d
        POSITION_INDEPENDENT_CODE ON
        VERSION ${PROJECT_VERSION}
)

if (${PROJECT_NAME}_BUILD_SAMPLES)
    add_subdirectory(PolyDepthDemo)
endif ()