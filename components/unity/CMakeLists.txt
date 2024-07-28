

include_directories(
       ${CMAKE_CURRENT_SOURCE_DIR}/include
       ${CMAKE_CURRENT_SOURCE_DIR}/unity/src
)

add_definitions(-DUNITY_EXCLUDE_STDLIB_MALLOC)

aux_source_directory(. DIR_UNITY_SRCS)
aux_source_directory(./unity/src DIR_UNITY_CORE_SRCS)

list (APPEND DIR_UNITY_SRCS ${DIR_UNITY_CORE_SRCS})
add_library (unity ${DIR_UNITY_SRCS})
message("unity compiled file=" ${DIR_UNITY_SRCS})