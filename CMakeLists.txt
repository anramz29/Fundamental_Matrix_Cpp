cmake_minimum_required(VERSION 3.10)
project(Fundamental_Matrix_Cpp)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

# step 0 — just loads + shows images, no src/ deps yet
add_executable(00_load apps/00_load.cpp)
target_link_libraries(00_load ${OpenCV_LIBS})

add_executable(01_match apps/01_match.cpp)
target_link_libraries(01_match ${OpenCV_LIBS})

add_executable(02_opencv_f apps/02_opencv_f.cpp)
target_link_libraries(02_opencv_f ${OpenCV_LIBS})

add_executable(03_my_f apps/03_my_f.cpp)
target_link_libraries(03_my_f ${OpenCV_LIBS})

# include the executables so they can find the header files
target_include_directories(02_opencv_f PRIVATE include)
target_include_directories(03_my_f PRIVATE include)

# later, when src/ files exist, you'll add a library and link it:
# add_library(stereo src/matching.cpp src/eight_point.cpp)
# target_include_directories(stereo PUBLIC include)
# target_link_libraries(stereo ${OpenCV_LIBS})
# target_link_libraries(01_match stereo ${OpenCV_LIBS})
