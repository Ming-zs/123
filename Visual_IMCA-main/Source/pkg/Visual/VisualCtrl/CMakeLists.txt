
# 功能包名
project("VisualCtrl")
# 搜索.cpp文件的所有路径
FILE(GLOB_RECURSE CPP_SET ${PROJECT_SOURCE_DIR}/src *.cpp)
# 搜索.h文件的所有路径
FILE(GLOB_RECURSE H_SET ${PROJECT_SOURCE_DIR}/inc *.h)
# 添加头文件路径
include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(${GLOBAL_VAR_DIR})

message("${BoldWhite}VisualCtrl Header:${ColourReset}" ${H_SET})

find_package(OpenCV REQUIRED)

set(VISUAL_CTRL
        ${PROJECT_NAME}
        CACHE INTERNAL "THE ARMOR_OPTION")

# 添加子包
add_library(${PROJECT_NAME} SHARED ${CPP_SET} ${H_SET})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
