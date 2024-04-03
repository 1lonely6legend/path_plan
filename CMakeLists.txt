cmake_minimum_required(VERSION 3.10)

project(path_plan)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# 定义一个变量‘PACKAGE_NAME’，将其值设置为‘path_plan’
set(PACKAGE_NAME path_plan)
# 设置可执行文件的输出目录。这里指定了构建（编译）后的可执行文件应该放在构建目录下的bin子目录中
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# 创建了一个名为path_plan的接口库，接口库不编译为二进制文件，而是可以用来传递使用要求（包含目录，编译选项等）给其他目标（库/可执行文件）
add_library(path_plan INTERFACE)

find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
#将python相关的库连接到path_plan接口库
target_link_libraries(path_plan INTERFACE
        Python::Python
        Python::Module
        Python::NumPy
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Dijkstra,创建可执行目标并指定源文件
add_executable(dijkstra_demo dijkstra/main.cpp dijkstra/dijkstra.cpp)
# 将dijkstra_demo可执行文件与path_plan接口库链接。PRIVATE指定了链接范围，这里意味着链接的库仅用于dijkstra_demo，不会传递给其他链接了dijkstra_demo的目标。
target_link_libraries(dijkstra_demo PRIVATE path_plan)

