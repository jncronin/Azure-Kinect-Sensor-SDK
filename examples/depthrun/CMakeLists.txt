# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_executable(depthrun main.c)
target_link_libraries(depthrun PRIVATE k4a::k4a
    )