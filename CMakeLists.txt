# ORB-SLAM 项目管理

# 确定最低版本CMake。
cmake_minimum_required(VERSION 2.8)
project(IMU_Preintegration)

# 设置项目编译方式 Release。
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release )
ENDIF()

MESSAGE("Build Type: " ${CMAKE_BUILD_TYPE})

# 检查C++11或C++0x 特性支持。
add_definitions(-DCOMPILEDWITHC11)
if( CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -msse3 -std=c++11 -pthread -g -march=native")
else( CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -msse3 -std=c++11 -pthread -O3 -march=native")
endif( CMAKE_BUILD_TYPE MATCHES "Debug")

# CMake文件列表。
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 查找依赖库。
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)

# include工程目录。
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${CHOLMOD_INCLUDE_DIR}
)

# 设置CMake库输出目录。
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 添加库 
add_library(${PROJECT_NAME} SHARED
        src/so3.cpp
        src/IMU_Data.cpp
        src/IMU_Preintegrator.cpp
)

# 链接库。 
target_link_libraries(${PROJECT_NAME}
${EIGEN3_LIBS}
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
cholmod
${CHOLMOD_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
)


# 编译单目KITTI数据集可执行文件
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})








  
