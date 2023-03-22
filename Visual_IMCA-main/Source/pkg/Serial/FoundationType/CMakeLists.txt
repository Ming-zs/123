
# 功能包名
project("FoundationType")
# 搜索.cpp文件的所有路径
FILE(GLOB_RECURSE CPP_SET ${PROJECT_SOURCE_DIR}/src *.cpp)
# 搜索.h文件的所有路径
FILE(GLOB_RECURSE H_SET ${PROJECT_SOURCE_DIR}/inc *.h)

message("${Red}FoundationType Header:${ColourReset}" ${H_SET})

set(FoundationType
        .
        CACHE INTERNAL "THE FoundationType")

set(GLOBAL_VAR_DIR ${PROJECT_SOURCE_DIR}/inc)
# 添加头文件路径
include_directories(${PROJECT_SOURCE_DIR}/inc)
# 添加子包
add_library(${PROJECT_NAME} SHARED ${CPP_SET} ${H_SET})
