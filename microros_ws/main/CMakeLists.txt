idf_component_register(
  SRCS
    "main.c"
    "driver/mpu6050.c"

  INCLUDE_DIRS
    "."
    "driver"

  PRIV_REQUIRES
    esp_driver_gpio


  REQUIRES
    driver
    esp_adc
    esp_timer
    esp_rom
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
