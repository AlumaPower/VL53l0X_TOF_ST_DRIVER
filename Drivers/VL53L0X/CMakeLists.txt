add_library(VL53L0X INTERFACE)

target_sources(VL53L0X INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/Api/core/src/vl53l0x_api_calibration.c
        ${CMAKE_CURRENT_LIST_DIR}/Api/core/src/vl53l0x_api_core.c
        ${CMAKE_CURRENT_LIST_DIR}/Api/core/src/vl53l0x_api_ranging.c
        ${CMAKE_CURRENT_LIST_DIR}/Api/core/src/vl53l0x_api_strings.c
        ${CMAKE_CURRENT_LIST_DIR}/Api/core/src/vl53l0x_api.c
        ${CMAKE_CURRENT_LIST_DIR}/Api/platform/src/vl53l0x_i2c_platform.c
        # ${CMAKE_CURRENT_LIST_DIR}/Api/platform/src/vl53l0x_i2c_win_serial_comms.c
        # ${CMAKE_CURRENT_LIST_DIR}/Api/platform/src/vl53l0x_platform_log.c
        ${CMAKE_CURRENT_LIST_DIR}/Api/platform/src/vl53l0x_platform.c
        )
        
target_include_directories(VL53L0X INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Api/core/inc ${CMAKE_CURRENT_LIST_DIR}/Api/platform/inc )
target_link_libraries(VL53L0X INTERFACE pico_stdlib hardware_i2c)