idf_component_register(
  SRCS
    "main.c"
    "red.c"
    "sensores/us_vs_laser/vl53l0x_drv.c"
    "sensores/us_vs_laser/hc_sr04.c"
  INCLUDE_DIRS
    "."
    "sensores/us_vs_laser"
  PRIV_REQUIRES
    micro_ros_espidf_component
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_rmt
  REQUIRES
    esp_timer
    esp_rom
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
