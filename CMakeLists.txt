cmake_minimum_required(VERSION 3.13)

project(Forte_Cloud_Project)

#add_executable(my_app ${SOURCES})

file(GLOB SOURCES "src/*.c")

# Add workshop code and files.
#include_directories(my_app PRIVATE include)


# Tell IDF build to link against this target.
set(IDF_PROJECT_EXECUTABLE my_app)

#set(IDF_SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")




#get_filename_component(
#    IDF_EXECUTABLE_SRCS
#    "src/main.c" ABSOLUTE
#    )



# As of now there's no offical way to redefine config files outside of Amazon FreeRTOS source tree.
# This is a temporary approach to inject an include path so that this takes precedence over the
# config file directory inside Amazon FreeRTOS.
#include_directories(BEFORE amazon-freertos-configs)
include_directories(BEFORE freertos-configs)


# Add amazon freertos as an subdirectory. AFR_BOARD tells which board to target.
set(AFR_BOARD espressif.esp32_devkitc CACHE INTERNAL "")
add_subdirectory(amazon-freertos)



target_link_libraries(my_app 
    PRIVATE 
        AFR::core_mqtt
        AFR::common_io
        AFR::core_json
        AFR::pkcs11
        AFR::pkcs11_helpers
        AFR::pkcs11_implementation
        AFR::pkcs11_utils
        AFR::wifi
        AFR::secure_sockets
        AFR::platform
        AFR::logging
        AFR::demo_core_mqtt
		AFR::backoff_algorithm
	        AFR::transport_interface_secure_sockets
		AFR::core_mqtt_demo_dependencies
		AFR::greengrass
		AFR::demo_greengrass_connectivity
		AFR::tls
		AFR::kernel
#		AFR::mbedtls
		AFR::pkcs11
)

