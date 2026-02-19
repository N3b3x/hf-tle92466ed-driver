###############################################################################
# HF TLE92466ED Driver - ESP-IDF Component Wrapper
###############################################################################
# Note:
#   cmake_minimum_required() is intentionally omitted here — ESP-IDF's
#   project.cmake already sets it and adding another can cause policy conflicts.
###############################################################################

set(HF_TLE92466ED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." CACHE PATH
    "Path to root of hf-tle92466ed-driver (auto-detected from component location)")

get_filename_component(HF_TLE92466ED_ROOT "${HF_TLE92466ED_ROOT}" ABSOLUTE)
if(NOT EXISTS "${HF_TLE92466ED_ROOT}/cmake/hf_tle92466ed_build_settings.cmake")
    message(FATAL_ERROR
        "Cannot find TLE92466ED driver root.\n"
        "  Resolved path: ${HF_TLE92466ED_ROOT}\n"
        "  Expected file: ${HF_TLE92466ED_ROOT}/cmake/hf_tle92466ed_build_settings.cmake\n"
        "  Fix: set HF_TLE92466ED_ROOT to the correct driver checkout path.")
endif()

include("${HF_TLE92466ED_ROOT}/cmake/hf_tle92466ed_build_settings.cmake")

idf_component_register(
    SRCS ${HF_TLE92466ED_SOURCE_FILES}
    INCLUDE_DIRS ${HF_TLE92466ED_PUBLIC_INCLUDE_DIRS}
    REQUIRES ${HF_TLE92466ED_IDF_REQUIRES}
)

if(HF_TLE92466ED_SOURCE_FILES)
    target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
else()
    target_compile_features(${COMPONENT_LIB} INTERFACE cxx_std_20)
endif()

