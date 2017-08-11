project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

#set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)

#added includes for plotting
#adapted from https://github.com/lava/matplotlib-cpp
find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARIES})
include_directories(src/matplotlib-cpp)

## original given by readme
#https://github.com/lava/matplotlib-cpp
#find_package(PythonLibs 2.7)
#target_include_directories(MPC PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(MPC ${PYTHON_LIBRARIES})

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS ${PYTHON_LIBRARIES} pthread)