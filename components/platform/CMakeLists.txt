
include_directories(
       ${CMAKE_CURRENT_SOURCE_DIR}/include
       ${CMAKE_CURRENT_SOURCE_DIR}/arch
       ${CMAKE_CURRENT_SOURCE_DIR}/arch/include/arch_nv307/
       ${CMAKE_CURRENT_SOURCE_DIR}/driver/flash
       ${CMAKE_CURRENT_SOURCE_DIR}/hal/common
)


aux_source_directory(./arch DIR_UNITY_SRCS)
aux_source_directory(./unity/src DIR_UNITY_CORE_SRCS)


file(GLOB_RECURSE platform_arch_cs ${CMAKE_CURRENT_SOURCE_DIR}/arch/*.c)
file(GLOB_RECURSE platform_arch_asms ${CMAKE_CURRENT_SOURCE_DIR}/arch/*.S)
file(GLOB_RECURSE platform_arch_cpps ${CMAKE_CURRENT_SOURCE_DIR}/arch/*.cpp)

list (APPEND DIR_PLATFORM_SRCS ${platform_arch_cs})
list (APPEND DIR_PLATFORM_SRCS ${platform_arch_asms})
list (APPEND DIR_PLATFORM_SRCS ${platform_arch_cpps})


file(GLOB platform_driver_cs ${CMAKE_CURRENT_SOURCE_DIR}/driver/*.c)
file(GLOB platform_driver_asms ${CMAKE_CURRENT_SOURCE_DIR}/driver/*.S)
file(GLOB platform_driver_cpps ${CMAKE_CURRENT_SOURCE_DIR}/driver/*.cpp)

list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cs})
list (APPEND DIR_PLATFORM_SRCS ${platform_driver_asms})
list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cpps})

file(GLOB_RECURSE platform_driver_cs ${CMAKE_CURRENT_SOURCE_DIR}/driver/flash/*.c)
file(GLOB_RECURSE platform_driver_asms ${CMAKE_CURRENT_SOURCE_DIR}/driver/flash/*.S)
file(GLOB_RECURSE platform_driver_cpps ${CMAKE_CURRENT_SOURCE_DIR}/driver/flash/*.cpp)

list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cs})
list (APPEND DIR_PLATFORM_SRCS ${platform_driver_asms})
list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cpps})

message("board=" ${BOARD})

if("${BOARD}"  STREQUAL "FPGA")
    file(GLOB_RECURSE platform_driver_cs ${CMAKE_CURRENT_SOURCE_DIR}/driver/FPGA_evb_board/*.c)
    file(GLOB_RECURSE platform_driver_asms ${CMAKE_CURRENT_SOURCE_DIR}/driver/FPGA_evb_board/*.S)
    file(GLOB_RECURSE platform_driver_cpps ${CMAKE_CURRENT_SOURCE_DIR}/driver/FPGA_evb_board/*.cpp)
    list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cs})
    list (APPEND DIR_PLATFORM_SRCS ${platform_driver_asms})
    list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cpps})
elseif("${BOARD}" STREQUAL "1008")
    file(GLOB_RECURSE platform_driver_cs ${CMAKE_CURRENT_SOURCE_DIR}/driver/1008_evb_board/*.c)
    file(GLOB_RECURSE platform_driver_asms ${CMAKE_CURRENT_SOURCE_DIR}/driver/1008_evb_board/*.S)
    file(GLOB_RECURSE platform_driver_cpps ${CMAKE_CURRENT_SOURCE_DIR}/driver/1008_evb_board/*.cpp)
    list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cs})
    list (APPEND DIR_PLATFORM_SRCS ${platform_driver_asms})
    list (APPEND DIR_PLATFORM_SRCS ${platform_driver_cpps})
endif( )



if("${BT_SUPPORT}"  STREQUAL "YES")

else()
file(GLOB_RECURSE platform_test_cs ${CMAKE_CURRENT_SOURCE_DIR}/test/*.c)
file(GLOB_RECURSE platform_test_asms ${CMAKE_CURRENT_SOURCE_DIR}/test/*.S)
file(GLOB_RECURSE platform_test_cpps ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
list (APPEND DIR_PLATFORM_SRCS ${platform_test_cs})
list (APPEND DIR_PLATFORM_SRCS ${platform_test_asms})
list (APPEND DIR_PLATFORM_SRCS ${platform_test_cpps})
endif( )



if("${SOC}"  STREQUAL "MS1008")
    file(GLOB_RECURSE platform_ms_cs ${CMAKE_CURRENT_SOURCE_DIR}/hal/ms1008/*.c)
    file(GLOB_RECURSE platform_ms_asms ${CMAKE_CURRENT_SOURCE_DIR}/hal/ms1008/*.S)
    file(GLOB_RECURSE platform_ms_cpps ${CMAKE_CURRENT_SOURCE_DIR}/hal/ms1008/*.cpp)

    list (APPEND DIR_PLATFORM_SRCS ${platform_ms_cs})
    list (APPEND DIR_PLATFORM_SRCS ${platform_ms_asms})
    list (APPEND DIR_PLATFORM_SRCS ${platform_ms_cpps})
elseif("${SOC}"  STREQUAL "MS1008_V2")
    file(GLOB_RECURSE platform_ms_cs ${CMAKE_CURRENT_SOURCE_DIR}/hal/ms1008_v2/*.c)
    file(GLOB_RECURSE platform_ms_asms ${CMAKE_CURRENT_SOURCE_DIR}/hal/ms1008_v2/*.S)
    file(GLOB_RECURSE platform_ms_cpps ${CMAKE_CURRENT_SOURCE_DIR}/hal/ms1008_v2/*.cpp)

    list (APPEND DIR_PLATFORM_SRCS ${platform_ms_cs})
    list (APPEND DIR_PLATFORM_SRCS ${platform_ms_asms})
    list (APPEND DIR_PLATFORM_SRCS ${platform_ms_cpps})   
    add_definitions(-D__MS1008_V2)
endif( )

file(GLOB platform_mshal_cs ${CMAKE_CURRENT_SOURCE_DIR}/hal/*.c)
file(GLOB platform_mshal_asms ${CMAKE_CURRENT_SOURCE_DIR}/hal/*.S)
file(GLOB platform_mshal_cpps ${CMAKE_CURRENT_SOURCE_DIR}/hal/*.cpp)

list (APPEND DIR_PLATFORM_SRCS ${platform_mshal_cs})
list (APPEND DIR_PLATFORM_SRCS ${platform_mshal_asms})
list (APPEND DIR_PLATFORM_SRCS ${platform_mshal_cpps})

file(GLOB_RECURSE platform_mshal_cs ${CMAKE_CURRENT_SOURCE_DIR}/hal/common/*.c)
file(GLOB_RECURSE platform_mshal_asms ${CMAKE_CURRENT_SOURCE_DIR}/hal/common/*.S)
file(GLOB_RECURSE platform_mshal_cpps ${CMAKE_CURRENT_SOURCE_DIR}/hal/common/*.cpp)

list (APPEND DIR_PLATFORM_SRCS ${platform_mshal_cs})
list (APPEND DIR_PLATFORM_SRCS ${platform_mshal_asms})
list (APPEND DIR_PLATFORM_SRCS ${platform_mshal_cpps})


message("platform compiled file=" ${DIR_PLATFORM_SRCS})

add_library (platform ${DIR_PLATFORM_SRCS})
