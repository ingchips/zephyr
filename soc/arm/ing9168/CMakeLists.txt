# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  reboot.S
  sys_arch_reboot.c
  power.c
)

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/arm/include
)
zephyr_library_compile_definitions(fabs=__builtin_fabs)

set(SOC_LINKER_SCRIPT
    ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld
    CACHE INTERNAL "")

set(ING_SDK_BASE "${ZEPHYR_BASE}/../modules/hal/ingchips")
set(ING_BUNDLE_DIR "${ING_SDK_BASE}/bundles/noos_mini/ING9168xx")
set(ING_APIS_JSON "${ING_BUNDLE_DIR}/apis.json")
set(ING_SYMDEFS "${CMAKE_CURRENT_BINARY_DIR}/symdefs.g")

if(NOT EXISTS "${ING_APIS_JSON}")
  message(FATAL_ERROR
    "INGCHIPS SDK is missing at ${ING_SDK_BASE}; run 'west update hal_ingchips'")
endif()

file(READ "${ING_APIS_JSON}" ing_apis)
string(JSON ing_api_count LENGTH "${ing_apis}")
math(EXPR ing_api_last "${ing_api_count} - 1")
file(WRITE "${ING_SYMDEFS}" "/* Generated from ${ING_APIS_JSON}. */\n")
foreach(index RANGE 0 ${ing_api_last})
  string(JSON symbol MEMBER "${ing_apis}" ${index})
  string(JSON address GET "${ing_apis}" "${symbol}")
  file(APPEND "${ING_SYMDEFS}" "${symbol} = ${address};\n")
endforeach()

target_link_options(app PUBLIC
  "-Wl,--just-symbols=${ING_SYMDEFS}")

zephyr_include_directories(
  ${ING_SDK_BASE}/src/BSP
  ${ING_SDK_BASE}/src/FWlib
  ${ING_SDK_BASE}/src/Tools
  ${ING_SDK_BASE}/src/hal_zephyr
  ${ING_SDK_BASE}/src/StartUP/ing916
  ${ING_SDK_BASE}/bundles/noos_mini/inc
)

zephyr_library_sources(
  hal_zephyr.c
  gen_os_impl.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_adc.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_gpio.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_i2c.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_pinctrl.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_pwm.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_rtc.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_ssp.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_sysctrl.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_timer.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_uart.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_comparator.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_dma.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_efuse.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_i2s.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_ir.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_keyscan.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_pdm.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_pte.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_qdec.c
  ${ING_SDK_BASE}/src/FWlib/peripheral_usb.c
  ${ING_SDK_BASE}/src/Tools/trace.c
  ${ING_SDK_BASE}/src/BSP/eflash.c
)

add_definitions(-DUSE_STDPERIPH_DRIVER -DPOWER_SAVING)
