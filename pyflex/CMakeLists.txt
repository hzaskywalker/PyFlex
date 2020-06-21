cmake_minimum_required(VERSION 2.8.12)
project(pyflex)


if (NOT DEFINED FleX_ROOT)
    set(FleX_ROOT /home/hza/fluid/PyFlex)
endif()
if (NOT DEFINED CUDA_ROOT)
    set(CUDA_ROOT /usr/local/cuda-9.2)
endif()


######################################### Find Dependencies ############################################
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(OpenGL REQUIRED)
find_package(Eigen REQUIRED)
find_package(FleX REQUIRED)
find_package(CUDA REQUIRED)

set(FleX_DEPENDENCIES
    ${FleX_LIBRARIES}
    ${CUDA_LIBRARIES}
    rt
    )

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${FleX_INCLUDE_DIR})

link_directories("${CMAKE_BINARY_DIR}/lib")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${FleX_LIB_DIR})


######################################### Link via Pybind ############################################
add_subdirectory(pybind11)
pybind11_add_module(pyflex src/pyflex.cpp ${FleX_INCLUDE_SOURCE} ${FleX_INCLUDE_HEADER})
target_compile_definitions(pyflex PRIVATE FleX_ROOT="${FleX_ROOT}")
target_compile_options(pyflex PRIVATE -I/home/hza/anaconda3/include/python3.7m)
target_include_directories(pyflex PRIVATE "${PYTHON_INCLUDE_DIRS}" -fPIC)
target_link_libraries(pyflex PRIVATE ${FleX_DEPENDENCIES} -fPIC)

