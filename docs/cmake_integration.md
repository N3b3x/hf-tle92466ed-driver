---
layout: default
title: "⚙️ CMake Integration"
description: "How to integrate the TLE92466ED driver into your CMake project"
nav_order: 5
parent: "📚 Documentation"
permalink: /docs/cmake_integration/
---

# TLE92466ED — CMake Integration Guide

> **Build contract architecture and templates:**
> [CMake Build Contract](../../../../../docs/development/CMAKE_BUILD_CONTRACT.md).

## Quick Start (Generic CMake)

```cmake
add_subdirectory(external/hf-tle92466ed-driver)
target_link_libraries(my_app PRIVATE hf::tle92466ed)
```

## ESP-IDF Integration

Component wrapper: `examples/esp32/components/hf_tle92466ed/`

```cmake
idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES hf_tle92466ed driver
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
```

## Build Variables

| Variable | Value |
|----------|-------|
| `HF_TLE92466ED_TARGET_NAME` | `hf_tle92466ed` |
| `HF_TLE92466ED_VERSION` | Current `MAJOR.MINOR.PATCH` |
| `HF_TLE92466ED_SOURCE_FILES` | `""` (header-only) |
| `HF_TLE92466ED_IDF_REQUIRES` | `driver` |

---

**Navigation**
⬅️ [Back to Documentation Index](../../DOCUMENTATION_INDEX.md) | [CMake Build Contract ↗](../../../../../docs/development/CMAKE_BUILD_CONTRACT.md)
