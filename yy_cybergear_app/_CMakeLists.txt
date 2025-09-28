cmake_minimum_required(VERSION 3.10)
project(yy_cybergear_app)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Qt5 setup
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)

# cybergear_gui_app ============================
set(TARGET cybergear_gui_app)
ament_auto_add_executable(${TARGET}
  src/main.cpp
  src/main_window.cpp
  include/yy_cybergear_app/main_window.hpp
)

target_link_libraries(${TARGET}
  Qt5::Core
  Qt5::Widgets
)

unset(TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()