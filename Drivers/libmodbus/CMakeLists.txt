cmake_minimum_required(VERSION 3.22)

project(
    libmodbus
    DESCRIPTION "Modbus library"
    LANGUAGES C
)

# Add source files
set(LIBRARY_SOURCES
    src/modbus.c
    src/modbus-data.c
    # src/modbus-rtu.c  # Exclude RTU - requires termios not available on embedded
    # src/modbus-tcp.c  # Exclude TCP for embedded systems
)

# Create static library
add_library(libmodbus STATIC ${LIBRARY_SOURCES})

# Add include directories
target_include_directories(libmodbus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# Add compile definitions
target_compile_definitions(libmodbus PRIVATE __EMBEDDED__)

# Set library properties
set_target_properties(libmodbus PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
)