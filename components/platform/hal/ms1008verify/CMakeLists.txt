cmake_minimum_required (VERSION 3.1)

project(test_ms1008)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_C_COMPILER   "riscv-nuclei-elf-gcc")
SET(CMAKE_CXX_COMPILER   "riscv-nuclei-elf-g++")
SET(CMAKE_AR  "riscv-nuclei-elf-ar")
SET(CMAKE_AS  "riscv-nuclei-elf-as")
SET(CMAKE_STRIP  "riscv-nuclei-elf-strip")
SET(CMAKE_LD  "riscv-nuclei-elf-ld")
SET(CMAKE_NM  "riscv-nuclei-elf-nm")
SET(CMAKE_RANLIB  "riscv-nuclei-elf-ranlib")

ENABLE_LANGUAGE(ASM)


add_compile_options(-march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -std=gnu11 -DDOWNLOAD_MODE=DOWNLOAD_MODE_ILM  -std=gnu11 -MMD -MP)
#SET(CMAKE_C_FLAGS "-march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DDOWNLOAD_MODE=DOWNLOAD_MODE_ILM  -std=gnu11 -MMD -MP")
#SET(CMAKE_CXX_FLAGS "-march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DDOWNLOAD_MODE=DOWNLOAD_MODE_ILM  -std=gnu11 -MMD -MP")

string(REGEX REPLACE "(.*)/(.*)/(.*)/(.*)/(.*)" "\\1" PROJECT_INIT_PATH  ${PROJECT_SOURCE_DIR})

if("${SOC}"  STREQUAL "MS1008")
SET(LINKER_SCRIPT  ${PROJECT_INIT_PATH}/linkcfg/gcc_flash_ms1008.ld)
elseif("${SOC}"  STREQUAL "MS1008_V2")
SET(LINKER_SCRIPT  ${PROJECT_INIT_PATH}/linkcfg/gcc_flash_ms1008_v2.ld)
endif( )

message("project init=" ${PROJECT_INIT_PATH})
message("project source=" ${PROJECT_SOURCE_DIR})
message("link file=" ${LINKER_SCRIPT})
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}  -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os  -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections   -g -Wl,-Map=drivertest.map -nostartfiles -Xlinker --gc-sections  --specs=nano.specs --specs=nosys.specs  -u _isatty -u _write -u _sbrk -u _read -u _close -u _fstat -u _lseek ")
message("link flags=" ${CMAKE_EXE_LINKER_FLAGS})
add_definitions(-DFREERTOS_SUPPORT)
add_definitions(-Dcpu_zeroriscy)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_INIT_PATH}/components/platform/include)
include_directories(${PROJECT_INIT_PATH}/components/platform/arch)
include_directories(${PROJECT_INIT_PATH}/components/platform/arch/include/arch_nv307)
include_directories(${PROJECT_INIT_PATH}/components/platform/driver/flash)
include_directories(${PROJECT_INIT_PATH}/components/platform/hal/common)
include_directories(${PROJECT_INIT_PATH}/components/unity/include)
include_directories(${PROJECT_INIT_PATH}/components/unity/unity/src)
include_directories(${PROJECT_INIT_PATH}/components/log)
include_directories(${PROJECT_INIT_PATH}/components/freertos/portable)
include_directories(${PROJECT_INIT_PATH}/components/freertos/include)
include_directories(${PROJECT_INIT_PATH}/components/bt/include)
include_directories(${PROJECT_INIT_PATH}/components/third_party)

if("${SOC}"  STREQUAL "MS1008")
include_directories(
      ${PROJECT_INIT_PATH}/components/platform/hal/ms1008
)
elseif("${SOC}"  STREQUAL "MS1008_V2")
include_directories(
       ${PROJECT_INIT_PATH}/components/platform/hal/ms1008_v2
)
endif( )

add_subdirectory(${PROJECT_INIT_PATH}/components/unity unity)
add_subdirectory(${PROJECT_INIT_PATH}/components/log log)
add_subdirectory(${PROJECT_INIT_PATH}/components/freertos freertos)

file(GLOB_RECURSE platform_arch_cs  ${PROJECT_INIT_PATH}/components/platform/arch/*.c)
file(GLOB_RECURSE platform_arch_asms ${PROJECT_INIT_PATH}/components/platform/arch/*.S)
file(GLOB_RECURSE platform_arch_cpps  ${PROJECT_INIT_PATH}/components/platform/arch/*.cpp)

list (APPEND DIR_SRCS ${platform_arch_cs})
list (APPEND DIR_SRCS ${platform_arch_asms})
list (APPEND DIR_SRCS ${platform_arch_cpps})

#file(GLOB_RECURSE platform_driver_cs ${PROJECT_INIT_PATH}/components/platform/driver/*.c)
#file(GLOB_RECURSE platform_driver_asms ${PROJECT_INIT_PATH}/components/platform/driver/*.S)
#file(GLOB_RECURSE platform_driver_cpps ${PROJECT_INIT_PATH}/components/platform/driver/*.cpp)

#list (APPEND DIR_SRCS ${platform_driver_cs})
#list (APPEND DIR_SRCS ${platform_driver_asms})
#list (APPEND DIR_SRCS ${platform_driver_cpps})

file(GLOB platform_driver_cs ${PROJECT_INIT_PATH}/components/platform/driver/*.c)
file(GLOB platform_driver_asms ${PROJECT_INIT_PATH}/components/platform/driver/*.S)
file(GLOB platform_driver_cpps ${PROJECT_INIT_PATH}/components/platform/driver/*.cpp)

list (APPEND DIR_SRCS ${platform_driver_cs})
list (APPEND DIR_SRCS ${platform_driver_asms})
list (APPEND DIR_SRCS ${platform_driver_cpps})

file(GLOB_RECURSE platform_driver_cs ${PROJECT_INIT_PATH}/components/platform/driver/flash/*.c)
file(GLOB_RECURSE platform_driver_asms ${PROJECT_INIT_PATH}/components/platform/driver/flash/*.S)
file(GLOB_RECURSE platform_driver_cpps ${PROJECT_INIT_PATH}/components/platform/driver/flash/*.cpp)

list (APPEND DIR_SRCS ${platform_driver_cs})
list (APPEND DIR_SRCS ${platform_driver_asms})
list (APPEND DIR_SRCS ${platform_driver_cpps})

message("board=" ${BOARD})

if("${BOARD}"  STREQUAL "FPGA")
    file(GLOB_RECURSE platform_driver_cs ${PROJECT_INIT_PATH}/components/platform/driver/FPGA_evb_board/*.c)
    file(GLOB_RECURSE platform_driver_asms ${PROJECT_INIT_PATH}/components/platform/driver/FPGA_evb_board/*.S)
    file(GLOB_RECURSE platform_driver_cpps ${PROJECT_INIT_PATH}/components/platform/driver/FPGA_evb_board/*.cpp)
    list (APPEND DIR_SRCS ${platform_driver_cs})
    list (APPEND DIR_SRCS ${platform_driver_asms})
    list (APPEND DIR_SRCS ${platform_driver_cpps})
elseif("${BOARD}" STREQUAL "1008")
    file(GLOB_RECURSE platform_driver_cs ${PROJECT_INIT_PATH}/components/platform/driver/1008_evb_board/*.c)
    file(GLOB_RECURSE platform_driver_asms ${PROJECT_INIT_PATH}/components/platform/driver/1008_evb_board/*.S)
    file(GLOB_RECURSE platform_driver_cpps ${PROJECT_INIT_PATH}/components/platform/driver/1008_evb_board/*.cpp)
    list (APPEND DIR_SRCS ${platform_driver_cs})
    list (APPEND DIR_SRCS ${platform_driver_asms})
    list (APPEND DIR_SRCS ${platform_driver_cpps})
endif( )


if("${SOC}"  STREQUAL "MS1008")
    file(GLOB_RECURSE platform_ms_cs ${PROJECT_INIT_PATH}/components/platform/hal/ms1008/*.c)
    file(GLOB_RECURSE platform_ms_asms ${PROJECT_INIT_PATH}/components/platform/hal/ms1008/*.S)
    file(GLOB_RECURSE platform_ms_cpps ${PROJECT_INIT_PATH}/components/platform/hal/ms1008/*.cpp)

    list (APPEND DIR_SRCS ${platform_ms_cs})
    list (APPEND DIR_SRCS ${platform_ms_asms})
    list (APPEND DIR_SRCS ${platform_ms_cpps})
elseif("${SOC}"  STREQUAL "MS1008_V2")
    file(GLOB_RECURSE platform_ms_cs ${PROJECT_INIT_PATH}/components/platform/hal/ms1008_v2/*.c)
    file(GLOB_RECURSE platform_ms_asms ${PROJECT_INIT_PATH}/components/platform/hal/ms1008_v2/*.S)
    file(GLOB_RECURSE platform_ms_cpps ${PROJECT_INIT_PATH}/components/platform/hal/ms1008_v2/*.cpp)

    list (APPEND DIR_SRCS ${platform_ms_cs})
    list (APPEND DIR_SRCS ${platform_ms_asms})
    list (APPEND DIR_SRCS ${platform_ms_cpps})  
    add_definitions(-D__MS1008_V2)
    add_definitions(-DKEYPAD_TO2)	
endif( )


file(GLOB platform_mshal_cs ${PROJECT_INIT_PATH}/components/platform/hal/*.c)
file(GLOB platform_mshal_asms ${PROJECT_INIT_PATH}/components/platform/hal/*.S)
file(GLOB platform_mshal_cpps ${PROJECT_INIT_PATH}/components/platform/hal/*.cpp)

list (APPEND DIR_SRCS ${platform_mshal_cs})
list (APPEND DIR_SRCS ${platform_mshal_asms})
list (APPEND DIR_SRCS ${platform_mshal_cpps})

file(GLOB platform_mshal_cs ${PROJECT_INIT_PATH}/components/platform/hal/common/*.c)
file(GLOB platform_mshal_asms ${PROJECT_INIT_PATH}/components/platform/hal/common/*.S)
file(GLOB platform_mshal_cpps ${PROJECT_INIT_PATH}/components/platform/hal/common/*.cpp)

list (APPEND DIR_SRCS ${platform_mshal_cs})
list (APPEND DIR_SRCS ${platform_mshal_asms})
list (APPEND DIR_SRCS ${platform_mshal_cpps})




file(GLOB_RECURSE platform_ms_cs ${PROJECT_INIT_PATH}/components/platform/hal/ms1008verify/*.c)
file(GLOB_RECURSE platform_ms_asms ${PROJECT_INIT_PATH}/components/platform/hal/ms1008verify/*.S)
file(GLOB_RECURSE platform_ms_cpps ${PROJECT_INIT_PATH}/components/platform/hal/ms1008verify/*.cpp)

list (APPEND DIR_SRCS ${platform_ms_cs})
list (APPEND DIR_SRCS ${platform_ms_asms})
list (APPEND DIR_SRCS ${platform_ms_cpps})




add_executable(${PROJECT_NAME} ${DIR_SRCS})
message("main file =" ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} -Wl,--whole-archive unity log freertos -Wl,--no-whole-archive)



