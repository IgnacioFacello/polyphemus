idf_component_register(
  SRCS
    "main.c"
    "driver/as5600.c"
    "driver/potentiometer.c"

  INCLUDE_DIRS
    "."
    "driver"

  PRIV_REQUIRES
    micro_ros_espidf_component
    esp_driver_gpio

  REQUIRES
    driver
    esp_adc
    esp_timer
    esp_rom
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)