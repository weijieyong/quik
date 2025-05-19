cmake_minimum_required(VERSION 3.15)
project(quik LANGUAGES CXX)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Eigen as dependency
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG master
)
FetchContent_MakeAvailable(Eigen)

# Library target (header + source)
add_library(quik STATIC
    src/geometry.cpp
    # Add more sources here if needed
)
set_target_properties(quik PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(quik
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(quik
    PUBLIC Eigen3::Eigen
)
# Install headers
install(DIRECTORY include/ DESTINATION include)

# Example executable (not installed by default, for local dev/tests)
add_executable(sample_cpp_usage src/sample_cpp_usage.cpp src/geometry.cpp)
target_link_libraries(sample_cpp_usage PRIVATE quik)

# Install library (export for downstream usage)
install(TARGETS quik
    EXPORT quikTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
# install(EXPORT quikTargets
#     FILE quikTargets.cmake
#     NAMESPACE quik::
#     DESTINATION lib/cmake/quik
# )

# Optional: install extra folders if they exist
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)
