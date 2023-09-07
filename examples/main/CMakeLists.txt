idf_component_register(SRCS "main.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino driver esp_netif esp_eth fatfs 
    vfs sdmmc 
    sx1509 ds3231 DFRobot_RGBLCD1602
    )
