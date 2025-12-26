zephyr_library()

zephyr_include_directories(include)

zephyr_library_include_directories(src)

zephyr_library_sources(
    src/mavwrap_core.c
)

zephyr_library_sources_ifdef(CONFIG_MAVWRAP_TRANSPORT_UART 
    src/mavwrap_uart.c
)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/mavlink/)
