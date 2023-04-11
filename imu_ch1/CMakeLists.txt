cmake_minimum_required(VERSION 3.0)
project(useSophus)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
ADD_COMPILE_OPTIONS(-std=c++14 )
# Eigen
Find_Package(Eigen3 REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(useSophus useSophus.cpp )
target_link_libraries(useSophus Sophus::Sophus)

