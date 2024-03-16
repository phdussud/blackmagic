# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)



# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picoblackmagic C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# project options
add_definitions(-DNO_LIBOPENCM3 -DPC_HOSTED=0)

# Add executable. Default name is the project name, version 0.1
add_executable(picoblackmagic
        src/command.c
        src/crc32.c
        src/exception.c
        src/gdb_main.c
        src/gdb_packet.c
        src/hex_utils.c
        src/main.c
        src/maths_utils.c
        src/morse.c
        src/remote.c
        src/rtt.c
        src/timing.c

        src/platforms/rp/cdc_uart.c
        src/platforms/rp/gdb_if.c
        src/platforms/rp/get_serial.c
        src/platforms/rp/jtagtap.c
        src/platforms/rp/led.c
        src/platforms/rp/pio_jtag.c
        src/platforms/rp/platform.c
        src/platforms/rp/swdptap.c
        src/platforms/rp/timing.c
        src/platforms/rp/traceswoasync.c
        src/platforms/rp/traceswodecode.c
        src/platforms/rp/usb_descriptors.c

        src/target/adiv5.c
        src/target/adiv5_jtag.c
        src/target/adiv5_swd.c
        src/target/at32f43x.c
        src/target/ch32f1.c
        src/target/cortex.c
        src/target/cortexar.c
        src/target/cortexm.c
        src/target/efm32.c
        src/target/flashstub/efm32.c
        src/target/flashstub/lmi.c
        src/target/flashstub/rp.c
        src/target/gdb_reg.c
        src/target/hc32l110.c
        src/target/imxrt.c
        src/target/jtagtap_generic.c
        src/target/jtag_devs.c
        src/target/jtag_scan.c
        src/target/kinetis.c
        src/target/lmi.c
        src/target/lpc11xx.c
        src/target/lpc15xx.c
        src/target/lpc17xx.c
        src/target/lpc40xx.c
        src/target/lpc43xx.c
        src/target/lpc546xx.c
        src/target/lpc55xx.c
        src/target/lpc_common.c
        src/target/msp432e4.c
        src/target/msp432p4.c
        src/target/nrf51.c
        src/target/nrf91.c
        src/target/nxpke04.c
        src/target/renesas_ra.c
        src/target/renesas_rz.c
        src/target/riscv32.c
        src/target/riscv64.c
        src/target/riscv_debug.c
        src/target/riscv_jtag_dtm.c
        src/target/rp.c
        src/target/s32k3xx.c
        src/target/sam3x.c
        src/target/sam4l.c
        src/target/samd.c
        src/target/samx5x.c
        src/target/semihosting.c
        src/target/sfdp.c
        src/target/spi.c
        src/target/stm32f1.c
        src/target/stm32f4.c
        src/target/stm32g0.c
        src/target/stm32h5.c
        src/target/stm32h7.c
        src/target/stm32l0.c
        src/target/stm32l4.c
        src/target/stm32mp15.c
        # src/target/swdptap_generic.c
        src/target/target.c
        src/target/target_flash.c
        src/target/target_probe.c
        src/target/zynq7000.c
)

target_include_directories(picoblackmagic PRIVATE
        src/platforms/rp
        src/platforms/common
        src/target
        src/include
)

pico_set_program_name(picoblackmagic "picoblackmagic")
pico_set_program_version(picoblackmagic "0.1")

pico_enable_stdio_uart(picoblackmagic 0)
pico_enable_stdio_usb(picoblackmagic 0)

target_compile_definitions(picoblackmagic PRIVATE
PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
)

pico_generate_pio_header(picoblackmagic ${CMAKE_CURRENT_LIST_DIR}/src/platforms/rp/swdp.pio)
pico_generate_pio_header(picoblackmagic ${CMAKE_CURRENT_LIST_DIR}/src/platforms/rp/jtag.pio)

# Add the standard library to the build
target_link_libraries(picoblackmagic pico_stdlib)

# Add any user requested libraries
target_link_libraries(picoblackmagic
        hardware_dma
        hardware_pio
        hardware_timer
        hardware_resets
        hardware_irq
        tinyusb_device
        tinyusb_board
        pico_unique_id
        pico_multicore
)

pico_add_extra_outputs(picoblackmagic)

