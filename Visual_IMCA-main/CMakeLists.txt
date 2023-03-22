# Root
# cmake版本
cmake_minimum_required(VERSION 3.12)
# 项目名
project(ROOT_IMCA)
# C++ 版本
set(CMAKE_CXX_STANDARD 17)
# 编译模式
set(CMAKE_BUILD_TYPE Debug)
# clangd使用的
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# 添加头文件目录
include_directories(Source/pkg/)
# 读取功能包列表
file(READ ${PROJECT_SOURCE_DIR}/script/ModuleList.txt read_file_var)
# Set Submodule BEGIN
set(Module_Lists ${read_file_var})
separate_arguments(Module_Lists)
# Set Submodule END
# 添加子类
add_subdirectory(Source/pkg)

include_directories(${CMAKE_SOURCE_DIR}/Source/3rdPartyLib)

FILE(GLOB_RECURSE Lib_HPP_SET ${CMAKE_SOURCE_DIR}/3rdPartyLib *.hpp)

message(LIB_H_SET ${Lib_HPP_SET})

# 查找当前目录下的SRC文件
aux_source_directory(${CMAKE_SOURCE_DIR}/Source DIR_SRCS)
# 添加到可执行文件
add_executable(${PROJECT_NAME} ${DIR_SRCS} ${Lib_HPP_SET})
# Third Model BEGIN
foreach(sub_name IN LISTS Module_Lists)
	target_link_libraries(${PROJECT_NAME} ${sub_name})
endforeach()
# Third Model END
# 从此处自定义连接第三方库
target_link_libraries(${PROJECT_NAME} pthread)