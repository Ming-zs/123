
# 功能包名
project("Experi")
# 搜索.cpp文件的所有路径
FILE(GLOB_RECURSE CPP_SET ../../.. *.cpp)
# 搜索.h文件的所有路径
FILE(GLOB_RECURSE H_SET ../../.. *.h)
message(NOTICE "Camera " ${CPP_SET})
# 添加头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include/)

set(EXPERI
        ${CMAKE_SOURCE_DIR}
        CACHE INTERNAL "THE EXPERI")

find_package(OpenCV REQUIRED)

link_directories(3rdLibrary)
# 添加子包
add_library(${PROJECT_NAME} SHARED ${CPP_SET} ${H_SET})
target_link_libraries(${PROJECT_NAME} libMVSDK.so ${OpenCV_LIBS})
