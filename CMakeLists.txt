#
# RasPi3 (BCM2837) RPI SD driver
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)

#-------------------------------------------------------------------------------
# Declare RasPi3 (BCM2837) SPI MSD Driver CAmkES Component
#       TODO
#
# Parameters:
#
#   <name>
#     required, component instance name
#
#   LIBS <lib_1> <lib_2> ...
#     additional libraries. At least a TimeServer client library is required
#
function(DeclareCAmkESComponent_RPI_SD_DRV name)

    cmake_parse_arguments(
        PARSE_ARGV
        1
        RPI_SD
        ""
        ""
        "LIBS"
    )

    DeclareCAmkESComponent(
        ${name}
        INCLUDES
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/include
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/lib
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/env/include
        SOURCES
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/RPI_SD.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/lib/emmc.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/lib/bcm2837_gpio.c

            # For EMMC mode 
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/env/lib/util.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/env/lib/synchronize.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/env/lib/memio.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/environment.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/mailboxInterface.c 

            # Additional files for SDHost mode
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/env/lib/spinlock.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/env/lib/string.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/lib/mmchost.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/circle/lib/bcm2837_sdhost.c
        C_FLAGS
            -Wall
            -Werror
        LIBS
            os_core_api
            os_libs
            ${RPI_SD_LIBS}
    )

endfunction()
