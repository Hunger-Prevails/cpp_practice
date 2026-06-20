find_package(Eigen3 CONFIG REQUIRED)
find_package(indicators CONFIG REQUIRED)

add_library(practices ring_buffer.cpp unique_pointer.cpp rigid_transform.cpp reprojection.cpp)
target_include_directories(practices PUBLIC ${CMAKE_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(practices PRIVATE /EHsc)
endif()

target_link_libraries(
    practices PUBLIC Eigen3::Eigen indicators::indicators
)
