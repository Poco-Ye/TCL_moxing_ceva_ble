

include_directories(
       ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE pm_cs ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
file(GLOB_RECURSE pm_asms ${CMAKE_CURRENT_SOURCE_DIR}/*.S)
file(GLOB_RECURSE pm_cpps ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

list (APPEND DIR_PM_SRCS ${pm_cs})
list (APPEND DIR_PM_SRCS ${pm_asms})
list (APPEND DIR_PM_SRCS ${pm_cpps})

message("pm compiled file=" ${DIR_PM_SRCS})

add_library (pm STATIC ${DIR_PM_SRCS})