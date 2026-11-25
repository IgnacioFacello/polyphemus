idf_component_register(
  SRCS "main.c" "red.c" "sensores/us vs laser/vl53l0x_drv.c"
  INCLUDE_DIRS "." "sensores/us vs laser"
  PRIV_REQUIRES
    micro_ros_espidf_component
    esp_driver_gpio
    esp_driver_i2c
  REQUIRES
    esp_timer
    esp_rom
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
