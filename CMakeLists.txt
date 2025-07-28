cmake_minimum_required(VERSION 3.10)
project(drone_msp_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

include_directories(src)

set(SOURCES
    src/main.cpp
    src/msp_connection.cpp
    src/msp_protocol.cpp
    src/drone_interface.cpp
    src/config_manager.cpp
)

set(HEADERS
    src/msp_connection.h
    src/msp_protocol.h
    src/drone_interface.h
    src/config_manager.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} Threads::Threads)

# Set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2)