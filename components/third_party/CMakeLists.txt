

include_directories(
       ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE sbc_cs ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
file(GLOB_RECURSE sbc_asms ${CMAKE_CURRENT_SOURCE_DIR}/*.S)
file(GLOB_RECURSE sbc_cpps ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

list (APPEND DIR_SBC_SRCS ${sbc_cs})
list (APPEND DIR_SBC_SRCS ${sbc_asms})
list (APPEND DIR_SBC_SRCS ${sbc_cpps})

message("third_party compiled file=" ${DIR_SBC_SRCS})

add_library (third_party STATIC ${DIR_SBC_SRCS})