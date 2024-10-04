zephyr_include_directories(include)

message("HELLO GOOD SIR MAY I HAVE SOME PMW3389:${CONFIG_PMW3389}")

add_subdirectory_ifdef(CONFIG_PMW3389 drivers/sensor/pmw3389)
