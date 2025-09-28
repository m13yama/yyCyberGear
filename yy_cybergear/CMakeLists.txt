cmake_minimum_required(VERSION 3.10)
project(yy_cybergear)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# cybergear library ==========================
set(TARGET cybergear)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp
  src/data_frame_handler.cpp
  src/logging.cpp
)
unset(TARGET)

# exmp_01_monitor_status ============================
set(TARGET exmp_01_monitor_status)
ament_auto_add_executable(${TARGET} example/${TARGET}.cpp)
unset(TARGET)

# # exmp_03_operation_sin_wave ============================
# set(TARGET exmp_03_operation_sin_wave)
# ament_auto_add_executable(${TARGET} example/${TARGET}.cpp)
# unset(TARGET)

# # exmp_04_position_sin_wave ============================
# set(TARGET exmp_04_position_sin_wave)
# ament_auto_add_executable(${TARGET} example/${TARGET}.cpp)
# unset(TARGET)

# exmp_05_speed_constant ============================
set(TARGET exmp_05_speed_constant)
ament_auto_add_executable(${TARGET} example/${TARGET}.cpp)
unset(TARGET)

# # exmp_06_current_constant ============================
# set(TARGET exmp_06_current_constant)
# ament_auto_add_executable(${TARGET} example/${TARGET}.cpp)
# unset(TARGET)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   # GoogleTest
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_data_frame_handler
#     test/test_data_frame_handler.cpp
#   )
#   if(TARGET test_data_frame_handler)
#     target_include_directories(test_data_frame_handler PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
#     target_link_libraries(test_data_frame_handler cybergear)
#   endif()
# endif()

ament_auto_package()
