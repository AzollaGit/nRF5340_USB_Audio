# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unicast_audio_server)

target_sources(app PRIVATE
  src/app_main.c
  src/hal_uart.c
  src/hal_gpio.c
  # src/hal_i2s.c
  src/audio_usb.c
  src/audio_i2s.c
)

 
zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
zephyr_library_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/boards)

# Flag which defines whether application is compiled as gateway/dongle or headset
add_compile_definitions(HEADSET=1)
add_compile_definitions(GATEWAY=2)

if (CONFIG_BT_LL_ACS_NRF53)
      file(TO_CMAKE_PATH ${ZEPHYR_NRF_MODULE_DIR}/lib/bin/bt_ll_acs_nrf53/bin CMAKE_STYLE_LL_ACS_NRF53_BIN)
 
      add_custom_target(
          post_bin ALL
          DEPENDS
          ${BLE5_CTR_SIGN_STEP}
      )
 
endif()