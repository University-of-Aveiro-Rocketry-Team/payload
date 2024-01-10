set(component_srcs "../minmea/minmea.c" "neo-7m.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "../minmea" ".")

target_compile_definitions(${COMPONENT_LIB} PRIVATE timegm=mktime)
