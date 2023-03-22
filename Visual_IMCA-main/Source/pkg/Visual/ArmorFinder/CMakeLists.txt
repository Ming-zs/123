
# 功能包名
project("ArmorFinder")
# 搜索.cpp文件的所有路径
FILE(GLOB_RECURSE CPP_SET ${PROJECT_SOURCE_DIR}/src *.cpp)
# 搜索.h文件的所有路径
FILE(GLOB_RECURSE H_SET ${PROJECT_SOURCE_DIR}/inc *.h)

# 打印信息
message("${BoldWhite}ArmorFinder Header:${ColourReset}" ${H_SET})

set(ARMOR_FINDER
        ${CMAKE_SOURCE_DIR}
        CACHE INTERNAL "THE ARMOR_FINDER")

# 添加头文件路径
include_directories(${PROJECT_SOURCE_DIR}/inc)

# 搜索opencv包
find_package(OpenCV REQUIRED)

# 添加子包
add_library(${PROJECT_NAME} ${CPP_SET} ${H_SET})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# 查看当前OpenCV信息
#message("OpenCV_DIR -----> ${OpenCV_DIR}")
#message("OpenCV_Version -----> ${OpenCV_VERSION}")