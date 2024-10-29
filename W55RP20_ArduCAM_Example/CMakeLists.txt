if (NOT TARGET ArduCAM)
    add_library(ArduCAM INTERFACE)
    target_sources(ArduCAM INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/ArduCAM/ArduCAM.c
            ${CMAKE_CURRENT_LIST_DIR}/ArduCAM/ov2640_regs.c
    )
    target_link_libraries(ArduCAM INTERFACE pico_stdlib hardware_i2c hardware_spi hardware_irq)
endif()
   

add_executable(W55RP20_ArduCAM_Example
		W55RP20_ArduCAM_Example.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
include_directories(ArduCAM)
target_link_libraries(W55RP20_ArduCAM_Example
	pico_stdlib
	hardware_dma
	hardware_i2c
	hardware_spi
	hardware_pwm
	ArduCAM
)

pico_enable_stdio_usb(W55RP20_ArduCAM_Example 1)
pico_enable_stdio_uart(W55RP20_ArduCAM_Example 0)
# create map/bin/hex file etc.
pico_add_extra_outputs(W55RP20_ArduCAM_Example)


