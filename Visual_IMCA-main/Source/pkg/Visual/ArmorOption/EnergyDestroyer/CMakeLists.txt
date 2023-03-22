
# 功能包名
project("EnergyDestroy")
# 搜索.cpp文件的所有路径
FILE(GLOB_RECURSE CPP_SET ${PROJECT_SOURCE_DIR}/src *.cpp)
# 搜索.h文件的所有路径
FILE(GLOB_RECURSE H_SET ${PROJECT_SOURCE_DIR}/inc *.h)
# 搜索接口文件的所有路径
FILE(GLOB_RECURSE INTERFACE ../Interface *.h)

message("${BoldWhite}EnergyDestroy Header:${ColourReset}" ${H_SET})

set(ENERGY_DESTROY
        ${CMAKE_SOURCE_DIR}
        CACHE INTERNAL "THE ENERGY_DESTROY")

# 添加头文件路径
include_directories(${PROJECT_SOURCE_DIR}/inc)

find_package(OpenCV REQUIRED)
# 添加子包
add_library(${PROJECT_NAME} ${CPP_SET} ${H_SET} ${INTERFACE})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
