#===============================================================================
# TLE92466ED Driver - Root CMake
# Defines the main build target for the driver. Minimal, platform-agnostic.
#===============================================================================

cmake_minimum_required(VERSION 3.16)

project(hf_tle92466ed_driver LANGUAGES CXX)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/hf_tle92466ed_build_settings.cmake)

if(HF_TLE92466ED_SOURCE_FILES)
    add_library(${HF_TLE92466ED_TARGET_NAME} STATIC ${HF_TLE92466ED_SOURCE_FILES})
    target_include_directories(${HF_TLE92466ED_TARGET_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
            $<BUILD_INTERFACE:${HF_TLE92466ED_VERSION_HEADER_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    target_compile_features(${HF_TLE92466ED_TARGET_NAME} PUBLIC cxx_std_20)
else()
    add_library(${HF_TLE92466ED_TARGET_NAME} INTERFACE)
    target_include_directories(${HF_TLE92466ED_TARGET_NAME}
        INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
            $<BUILD_INTERFACE:${HF_TLE92466ED_VERSION_HEADER_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    target_compile_features(${HF_TLE92466ED_TARGET_NAME} INTERFACE cxx_std_20)
endif()

add_library(hf::tle92466ed ALIAS ${HF_TLE92466ED_TARGET_NAME})

set_target_properties(${HF_TLE92466ED_TARGET_NAME} PROPERTIES VERSION ${HF_TLE92466ED_VERSION} SOVERSION ${HF_TLE92466ED_VERSION_MAJOR})

option(HF_TLE92466ED_ENABLE_WARNINGS "Enable extra warnings for TLE92466ED driver" OFF)
if(HF_TLE92466ED_ENABLE_WARNINGS)
    if(TARGET ${HF_TLE92466ED_TARGET_NAME})
        target_compile_options(${HF_TLE92466ED_TARGET_NAME} INTERFACE -Wall -Wextra -Wpedantic)
    endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${HF_TLE92466ED_TARGET_NAME} EXPORT hf_tle92466edTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY inc/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.h.in" EXCLUDE)
install(FILES "${HF_TLE92466ED_VERSION_HEADER}" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT hf_tle92466edTargets
    FILE hf_tle92466edTargets.cmake
    NAMESPACE hf::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hf_tle92466ed
)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/hf_tle92466edConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tle92466edConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hf_tle92466ed
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tle92466edConfigVersion.cmake"
    VERSION ${HF_TLE92466ED_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tle92466edConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tle92466edConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hf_tle92466ed
)
