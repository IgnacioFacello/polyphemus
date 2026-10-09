idf_component_register(
  SRCS
    "main.c"
    "driver/mpu6050.c"
    "accelerometer.c"
    "gyroscope.c"
    "quaternion.c"
    "complementary_filter.c"


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
