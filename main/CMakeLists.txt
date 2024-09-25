idf_component_register(
    SRCS "main.c" "i2s_par_rec.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio
)
