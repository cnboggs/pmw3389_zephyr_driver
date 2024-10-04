# Copyright (c) 2024 Noah Boggs 
# SPDX-License-Identifier: Apache-2.0

message("ADDING PMW3389 LIBS")

zephyr_library()
zephyr_library_sources(pmw3389.c)
zephyr_library_sources(pmw3389_priv.c)
