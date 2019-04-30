rock_library(coupled_control
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(coupled_control_bin Main.cpp
    DEPS coupled_control)

