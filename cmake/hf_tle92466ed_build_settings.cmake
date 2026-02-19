#===============================================================================
# TLE92466ED Driver - Build Settings
# Shared variables for target name, includes, sources, and dependencies.
# This file is the SINGLE SOURCE OF TRUTH for the driver version.
#===============================================================================

include_guard(GLOBAL)

set(HF_TLE92466ED_TARGET_NAME "hf_tle92466ed")

#===============================================================================
# Versioning (single source of truth)
#===============================================================================
set(HF_TLE92466ED_VERSION_MAJOR 1)
set(HF_TLE92466ED_VERSION_MINOR 0)
set(HF_TLE92466ED_VERSION_PATCH 0)
set(HF_TLE92466ED_VERSION "${HF_TLE92466ED_VERSION_MAJOR}.${HF_TLE92466ED_VERSION_MINOR}.${HF_TLE92466ED_VERSION_PATCH}")

#===============================================================================
# Generate version header from template (into build directory)
#===============================================================================
set(HF_TLE92466ED_VERSION_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/../inc/tle92466ed_version.h.in")
set(HF_TLE92466ED_VERSION_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/hf_tle92466ed_generated")
set(HF_TLE92466ED_VERSION_HEADER     "${HF_TLE92466ED_VERSION_HEADER_DIR}/tle92466ed_version.h")

file(MAKE_DIRECTORY "${HF_TLE92466ED_VERSION_HEADER_DIR}")

if(EXISTS "${HF_TLE92466ED_VERSION_TEMPLATE}")
    configure_file(
        "${HF_TLE92466ED_VERSION_TEMPLATE}"
        "${HF_TLE92466ED_VERSION_HEADER}"
        @ONLY
    )
    message(STATUS "TLE92466ED driver v${HF_TLE92466ED_VERSION} — generated tle92466ed_version.h in ${HF_TLE92466ED_VERSION_HEADER_DIR}")
else()
    message(WARNING "tle92466ed_version.h.in not found at ${HF_TLE92466ED_VERSION_TEMPLATE}")
endif()

#===============================================================================
# Public include directories
#===============================================================================
set(HF_TLE92466ED_PUBLIC_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../inc"
    "${HF_TLE92466ED_VERSION_HEADER_DIR}"
)

#===============================================================================
# Source files (empty for header-only)
#===============================================================================
set(HF_TLE92466ED_SOURCE_FILES)

#===============================================================================
# ESP-IDF component dependencies
#===============================================================================
set(HF_TLE92466ED_IDF_REQUIRES driver)
