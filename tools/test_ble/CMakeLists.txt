cmake_minimum_required (VERSION 3.1)

project(test_ble_sdk)

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
#SET(CMAKE_FIND_ROOT_PATH /cygdriver/c/RISCV)
#SET(CMAKE_FIND_ROOT_MODE_LIBRARY ONLY)
 

add_compile_options(-march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -std=gnu11 -DDOWNLOAD_MODE=DOWNLOAD_MODE_ILM  -std=gnu11 -MMD -MP)
#SET(CMAKE_C_FLAGS "-march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DDOWNLOAD_MODE=DOWNLOAD_MODE_ILM  -std=gnu11 -MMD -MP")
#SET(CMAKE_CXX_FLAGS "-march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DDOWNLOAD_MODE=DOWNLOAD_MODE_ILM  -std=gnu11 -MMD -MP")

string(REGEX REPLACE "(.*)/(.*)/(.*)" "\\1" PROJECT_INIT_PATH  ${PROJECT_SOURCE_DIR})

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
include_directories(${PROJECT_INIT_PATH}/components/bt/app)
include_directories(${PROJECT_INIT_PATH}/components/pm)
include_directories(${PROJECT_INIT_PATH}/components/third_party)

if("${BT_SUPPORT}"  STREQUAL "YES")
add_definitions(-DBT_SUPPORT)
endif( )


if("${SOC}"  STREQUAL "MS1008")
include_directories(
      ${PROJECT_INIT_PATH}/components/platform/hal/ms1008
)
elseif("${SOC}"  STREQUAL "MS1008_V2")
add_definitions(-DKEYPAD_TO2)
add_definitions(-DIR1_TO2)
#add_definitions(-DUSE_IR_NEC)
add_definitions(-DUSE_IR_RCA)

include_directories(
       ${PROJECT_INIT_PATH}/components/platform/hal/ms1008_v2
)
endif( )

add_subdirectory(${PROJECT_INIT_PATH}/components/platform platform)
add_subdirectory(${PROJECT_INIT_PATH}/components/unity unity)
add_subdirectory(${PROJECT_INIT_PATH}/components/log log)
add_subdirectory(${PROJECT_INIT_PATH}/components/freertos freertos)
add_subdirectory(${PROJECT_INIT_PATH}/components/pm pm)
add_subdirectory(${PROJECT_INIT_PATH}/components/third_party third_party)

if("${BT_SUPPORT}"  STREQUAL "YES")
add_subdirectory(${PROJECT_INIT_PATH}/components/bt bt)
endif( )


file(GLOB root_cs ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
file(GLOB root_asms ${CMAKE_CURRENT_SOURCE_DIR}/*.S)
file(GLOB root_cpps ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

list (APPEND DIR_SRCS ${root_cs})
list (APPEND DIR_SRCS ${root_asms})
list (APPEND DIR_SRCS ${root_cpps})


#aux_source_directory(. DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})
message("main file =" ${DIR_SRCS})

if("${BT_SUPPORT}"  STREQUAL "YES")
target_link_libraries(${PROJECT_NAME} -Wl,--whole-archive platform unity log third_party freertos ceva_ble ceva_host pm -Wl,--no-whole-archive)
else()
target_link_libraries(${PROJECT_NAME} -Wl,--whole-archive platform unity log third_party freertos pm -Wl,--no-whole-archive)
endif( )




