cmake_minimum_required(VERSION 3.9)
cmake_policy(SET CMP0091 NEW)

project(LevialdiShrinking LANGUAGES CXX CUDA)

set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -gencode arch=compute_52,code=sm_52
)

find_package(Thrust REQUIRED CONFIG)
thrust_create_target(Thrust)

find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} s)
find_package(Eigen3 CONFIG REQUIRED)

set(SRCS
    "src/main.cpp"
    "src/CCLabeler.cpp"
)

add_executable(label ${SRCS})
target_include_directories(label PUBLIC "include/" )
target_compile_features(label PRIVATE cxx_std_17)
target_link_libraries(label Thrust fmt::fmt-header-only ${OpenCV_LIBS} Eigen3::Eigen)