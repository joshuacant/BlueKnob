set(srcs "main.c")
set(includes ".")

idf_component_register(SRCS ${srcs}
                       PRIV_REQUIRES nvs_flash ulp
                       INCLUDE_DIRS ${includes})
