cmake_minimum_required(VERSION 3.13)
enable_language(ASM)

set(port_srcs
    portASM.s
    port.c
    portmacro.h
)

add_library(freertos_kernel_port OBJECT)

target_sources(freertos_kernel_port PRIVATE ${port_srcs})

add_library(freertos_kernel_port_headers INTERFACE)
target_include_directories(freertos_kernel_port_headers INTERFACE .)

target_link_libraries(freertos_kernel_port PRIVATE
    freertos_kernel_port_headers
    freertos_kernel_include)
