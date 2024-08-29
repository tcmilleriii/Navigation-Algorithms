cmake_minimum_required(VERSION 3.27.9)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(main VERSION 0.1.0 LANGUAGES C CXX)

add_executable(main 
    main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iterative_closest_point_helper.cpp
)
target_include_directories(main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
target_link_libraries(main Eigen3::Eigen)

