set(COMPONENT_REQUIRES "seesaw")
idf_component_register(SRCS
    "main.c" 
    "i2c_driver.c"
    "display.c"
    INCLUDE_DIRS ".")    