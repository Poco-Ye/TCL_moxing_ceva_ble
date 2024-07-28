

include_directories(
       ${CMAKE_CURRENT_SOURCE_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR}/include
       ${CMAKE_CURRENT_SOURCE_DIR}/portable
       ${CMAKE_CURRENT_SOURCE_DIR}/../bt/modules/rwip/api
       ${CMAKE_CURRENT_SOURCE_DIR}/../bt/plf/include
)

file(GLOB_RECURSE freertos_cs ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
file(GLOB_RECURSE freertos_asms ${CMAKE_CURRENT_SOURCE_DIR}/*.S)
file(GLOB_RECURSE freertos_cpps ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

list (APPEND DIR_FREERTOS_SRCS ${freertos_cs})
list (APPEND DIR_FREERTOS_SRCS ${freertos_asms})
list (APPEND DIR_FREERTOS_SRCS ${freertos_cpps})

message("freertos compiled file=" ${DIR_FREERTOS_SRCS})

add_library (freertos STATIC ${DIR_FREERTOS_SRCS})
