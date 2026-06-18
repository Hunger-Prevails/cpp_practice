find_package(cxxopts CONFIG REQUIRED)
find_package(nlohmann_json CONFIG 3.12.0 REQUIRED)

add_executable(main main.cpp)
if(MSVC)
    target_compile_options(main PRIVATE /EHsc)
endif()
target_link_libraries(main PRIVATE practices cxxopts::cxxopts nlohmann_json::nlohmann_json)
