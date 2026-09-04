idf_component_register(
  SRCS
    main.c
    driver/as5600.c
    driver/potentiometer.c
    esp32_serial_transport.c
  INCLUDE_DIRS "."
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
