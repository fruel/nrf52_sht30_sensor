cmake_minimum_required(VERSION 3.14)
project(nrf52_sht30_application LANGUAGES C ASM)

set(NRF5_TARGET "nrf52832_xxaa" CACHE STRING "" FORCE)
set(NRF5_SOFTDEVICE_VARIANT "s132" CACHE STRING "" FORCE)
set(NRF5_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/gcc_nrf52.ld  CACHE STRING "" FORCE)
set(NRF5_SDKCONFIG_PATH ${CMAKE_CURRENT_LIST_DIR} CACHE STRING "" FORCE)

include("nrf5")
add_executable(${CMAKE_PROJECT_NAME}
  "main.c"
)

nrf5_target(${CMAKE_PROJECT_NAME})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  "."
)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
  nrf5_nrfx_common
  nrf5_nrfx_gpiote
  nrf5_nrfx_saadc
  nrf5_nrfx_twim
  nrf5_ble_advertising
  nrf5_app_timer
  nrf5_delay
  nrf5_pwr_mgmt
  nrf5_app_util_platform
  nrf5_sdh
  nrf5_nrfx_prs
  ${NRF5_LIBRARY_NRF5_BLE_ADVERTISING_DEPENDENCIES}
  ${NRF5_LIBRARY_NRF5_NRFX_SAADC_DEPENDENCIES}
  ${NRF5_LIBRARY_NRF5_NRFX_TWIM_DEPENDENCIES}
)