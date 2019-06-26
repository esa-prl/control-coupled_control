# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)

find_package(PkgConfig)
pkg_check_modules(BASE_TYPES REQUIRED "base-types" "base-lib")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

rock_init(coupled_control 0.1)
rock_standard_layout()
