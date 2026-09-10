idf_component_register(SRCS "freddie_main.c"
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_ledc esp_driver_rmt esp_driver_uart)

# Which Freddie this firmware is for: selects the per-build hardware block
# in freddie_main.c (motor wiring and calibration, INA219, shelf LED).
# Pass it on the command line: idf.py -DFREDDIE_BUILD=2 build. It sticks in
# that build directory's CMake cache, so give each robot its own -B dir.
if(NOT DEFINED FREDDIE_BUILD)
    set(FREDDIE_BUILD 1)
endif()
set(FREDDIE_BUILD ${FREDDIE_BUILD} CACHE STRING "Which Freddie: selects the per-build hardware block")
message(STATUS "FREDDIE_BUILD=${FREDDIE_BUILD}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE FREDDIE_BUILD=${FREDDIE_BUILD})
