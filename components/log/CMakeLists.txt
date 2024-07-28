

include_directories(
       ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE log_cs ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
file(GLOB_RECURSE log_asms ${CMAKE_CURRENT_SOURCE_DIR}/*.S)
file(GLOB_RECURSE log_cpps ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

list (APPEND DIR_LOG_SRCS ${log_cs})
list (APPEND DIR_LOG_SRCS ${log_asms})
list (APPEND DIR_LOG_SRCS ${log_cpps})

message("log compiled file=" ${DIR_LOG_SRCS})

add_library (log STATIC ${DIR_LOG_SRCS})