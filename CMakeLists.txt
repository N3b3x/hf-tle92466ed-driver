# CMakeLists.txt for TLE92466ED Driver
cmake_minimum_required(VERSION 3.15)

project(TLE92466ED_Driver 
    VERSION 2.0.0
    DESCRIPTION "C++11+ Driver for Infineon TLE92466ED Six-Channel Low-Side Solenoid Driver"
    LANGUAGES CXX
)

# Set C++ standard (configurable - supports C++11 to C++23)
if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 23)  # Default to C++23 for full features
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler options
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Driver library
add_library(tle92466ed
    src/TLE92466ED.cpp
)

target_include_directories(tle92466ed PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Set target properties
set_target_properties(tle92466ed PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "include/TLE92466ED.hpp;include/TLE92466ED_HAL.hpp;include/TLE92466ED_Registers.hpp"
)

# Optional: Build examples
option(BUILD_EXAMPLES "Build example applications" OFF)

if(BUILD_EXAMPLES)
    add_executable(basic_example
        examples/basic_usage.cpp
    )
    
    target_link_libraries(basic_example PRIVATE tle92466ed)
endif()

# Optional: Build documentation with Doxygen
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" OFF)

if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    else()
        message("Doxygen not found, cannot build documentation")
    endif()
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS tle92466ed
    EXPORT TLE92466EDTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/tle92466ed
)

install(DIRECTORY docs/
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)

# Export targets
install(EXPORT TLE92466EDTargets
    FILE TLE92466EDTargets.cmake
    NAMESPACE TLE92466ED::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/TLE92466ED
)

# Create package config
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/TLE92466EDConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/TLE92466ED
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/TLE92466EDConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/TLE92466EDConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/TLE92466EDConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/TLE92466ED
)


