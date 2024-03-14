# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(external_lib)

# target_sources(app PRIVATE src/main_zephyr.c)

# The external static library that we are linking with does not know
# how to build for this platform so we export all the flags used in
# this zephyr build to the external build system.
#
# Other external build systems may be self-contained enough that they
# do not need any build information from zephyr. Or they may be
# incompatible with certain zephyr options and need them to be
# filtered out.
zephyr_get_include_directories_for_lang_as_string(       C includes)
zephyr_get_system_include_directories_for_lang_as_string(C system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       C definitions)
zephyr_get_compile_options_for_lang_as_string(           C options)

set(external_project_cflags
  "${includes} ${definitions} ${options} ${system_includes}"
  )

include(ExternalProject)

# Add an external project to be able download and build the third
# party library. In this case downloading is not necessary as it has
# been committed to the repository.
set(mylib_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/mylib)
set(mylib_build_dir ${CMAKE_CURRENT_SOURCE_DIR}/mylib)

set(MYLIB_LIB_DIR     ${mylib_build_dir}/lib)
set(MYLIB_INCLUDE_DIR ${mylib_src_dir}/include)
set(ING_REL ${mylib_src_dir}/ING918XX_SDK_SOURCE)
set(ING_BUNDLE ${mylib_src_dir}/ING918XX_SDK_SOURCE/bundles/noos_typical/inc)
set(ING_BUNDLE_G ${mylib_src_dir}/ING918XX_SDK_SOURCE/bundles/noos_typical/ING9168xx)

include_directories(${MYLIB_INCLUDE_DIR}/../src)

include_directories(${mylib_src_dir}/ING918XX_SDK_SOURCE/src/BSP)
include_directories(${ING_REL}/src/FWlib)

include_directories(${ING_REL}/src/StartUP/ing916)
include_directories(${ING_REL}/src/Tools)
include_directories(${ING_BUNDLE})
include_directories(${ING_BUNDLE}/os)
message("zyx debug includir ${ING_REL}/src/FWlib")

target_sources(app PRIVATE ${mylib_src_dir}/src/main.c)
target_sources(app PRIVATE ${mylib_src_dir}/src/profile.c)
target_sources(app PRIVATE ${mylib_src_dir}/src/h4_ingchips.c)
# target_sources(app PRIVATE ${mylib_src_dir}/src/uart_console.c)
# target_sources(app PRIVATE ${mylib_src_dir}/src/service_console.c)
target_sources(app PRIVATE ${mylib_src_dir}/portable/gen_os_impl.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_adc.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_gpio.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_i2c.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_pinctrl.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_pwm.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_rtc.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_ssp.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_sysctrl.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_timer.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_uart.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_comparator.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_dma.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_efuse.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_i2s.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_ir.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_keyscan.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_pdm.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_pte.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_qdec.c)
target_sources(app PRIVATE ${ING_REL}/src/FWlib/peripheral_usb.c)
target_sources(app PRIVATE ${ING_REL}/src/Tools/trace.c)
target_sources(app PRIVATE ${ING_REL}/src/BSP/eflash.c)



# zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
# zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/mesh)
# zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/audio)
# zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/host)
# zephyr_library_include_directories(${ZEPHYR_BASE}/tests/bluetooth/tester/src)
# target_sources(app PRIVATE
#     # ${ZEPHYR_BASE}/tests/bluetooth/tester/src/main.c
#     ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp.c
#     ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_core.c
#     ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_gap.c
#     # ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_gatt.c
#     )
# if(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL)
# target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_l2cap.c)
# endif()

# if(CONFIG_BT_MESH)
# target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_mesh.c)
# endif()

# target_sources_ifdef(CONFIG_BT_VCP_VOL_REND app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_vcp.c)

# if(CONFIG_BT_IAS OR CONFIG_BT_IAS_CLIENT)
# target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_ias.c)
# endif()

# if(CONFIG_BT_BAP_UNICAST)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_bap.c)
# endif()

# if(CONFIG_BT_HAS)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_has.c)
# endif()

# if (CONFIG_BT_CSIP_SET_MEMBER)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_csis.c)
# endif()

# if(CONFIG_BT_MICP_MIC_DEV)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_micp.c)
# endif()

# if(CONFIG_BT_AICS)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_aics.c)
# endif()

# if (CONFIG_BT_TBS_CLIENT)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_ccp.c)
# endif()

# if (CONFIG_BT_CAP_ACCEPTOR)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_cas.c)
# endif()

# if(CONFIG_BT_MCC)
#     target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/tester/src/btp_mcp.c)
# endif()








add_definitions(-DUSE_STDPERIPH_DRIVER)
add_definitions(-DPOWER_SAVING)
# add_definitions(-DLISTEN_TO_POWER_SAVING)
add_definitions(-DLISTEN_TO_POWER_SAVING)
add_definitions(-DINGCHIPS_FAMILY_916=1)
add_definitions(-DINGCHIPS_FAMILY=1)



# if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
# # https://www.gnu.org/software/make/manual/html_node/MAKE-Variable.html
# set(submake "$(MAKE)")
# else() # Obviously no MAKEFLAGS. Let's hope a "make" can be found somewhere.
# set(submake "make")
# endif()

# ExternalProject_Add(
#   mylib_project                 # Name for custom target
#   PREFIX     ${mylib_build_dir} # Root dir for entire project
#   SOURCE_DIR ${mylib_src_dir}
#   BINARY_DIR ${mylib_src_dir} # This particular build system is invoked from the root
#   CONFIGURE_COMMAND ""    # Skip configuring the project, e.g. with autoconf
#   BUILD_COMMAND
#   ${submake}
#   PREFIX=${mylib_build_dir}
#   CC=${CMAKE_C_COMPILER}
#   AR=${CMAKE_AR}
#   CFLAGS=${external_project_cflags}
#   INSTALL_COMMAND ""      # This particular build system has no install command
#   BUILD_BYPRODUCTS ${MYLIB_LIB_DIR}/libmylib.a
#   )

# # Create a wrapper CMake library that our app can link with
add_library(mylib_lib STATIC IMPORTED GLOBAL)
add_dependencies(
  mylib_lib
  mylib_project
)

# set_target_properties(mylib_lib PROPERTIES IMPORTED_LOCATION             ${MYLIB_LIB_DIR}/libmylib.a)
# set_target_properties(mylib_lib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MYLIB_INCLUDE_DIR})
# include_directories(${CMSIS_INC})
target_link_options(app PUBLIC -Wl,--just-symbols=${ING_BUNDLE_G}/symdefs.g)
message("zyx debug2 ${ING_BUNDLE_G}")
zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
# target_link_options(app PUBLIC ${ING_BUNDLE_G}/platform.bin)
# target_link_libraries(app PUBLIC mylib_lib)