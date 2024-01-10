set(component_srcs)

idf_component_register(SRCS "mpu-6500.c" "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS ".")

target_compile_definitions(${COMPONENT_LIB} PRIVATE timegm=mktime)
