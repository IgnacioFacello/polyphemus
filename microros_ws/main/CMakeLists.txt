idf_component_register(
  SRCS
    "main.c"
    "driver/mpu6050.c"
    "imu_aux/accelerometer.c"
    "imu_aux/gyroscope.c"
    "imu_aux/quaternion.c"
    "imu_aux/complementary_filter.c"

  INCLUDE_DIRS
    "."
    "driver"
    "imu_aux"

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
