# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

set(CMAKE_CUDA_ARCHITECTURES 53)
set(CMAKE_CUDA_COMPILER /usr/local/cuda-10.2/bin/nvcc)
enable_language(CUDA)
find_package(CUDAToolkit)

add_library(k4a_depth STATIC 
            depth.c
            depthengine.c
            cudade.cu
            )

set_property(TARGET k4a_depth PROPERTY CUDA_STANDARD 14)

if (K4A_MTE_VERSION)
    target_compile_definitions(k4a_depth PRIVATE K4A_MTE_VERSION)
endif ()

# Consumers should #include <k4ainternal/depth.h>
target_include_directories(k4a_depth PUBLIC 
    ${K4A_PRIV_INCLUDE_DIR})

# Dependencies of this library
target_link_libraries(k4a_depth PUBLIC
    CUDA::cudart
    CUDA::cuda_driver
    k4ainternal::depth_mcu
    k4ainternal::logging)


# Define alias for other targets to link against
add_library(k4ainternal::depth ALIAS k4a_depth)

# stand-alone depth library
add_library(k4a_depth_standalone STATIC
    cudade.cu)
set_property(TARGET k4a_depth_standalone PROPERTY CUDA_STANDARD 14)

target_link_libraries(k4a_depth_standalone PUBLIC
    CUDA::cudart
    CUDA::cuda_driver)
