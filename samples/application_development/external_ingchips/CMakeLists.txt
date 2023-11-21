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
set(ING_BUNDLE_G ${mylib_src_dir}/ING918XX_SDK_SOURCE/bundles/noos_typical/ING9187xx)
include_directories(${MYLIB_INCLUDE_DIR}/../src)

include_directories(${mylib_src_dir}/ING918XX_SDK_SOURCE/src/BSP)
include_directories(${ING_REL}/src/FWlib)

include_directories(${ING_REL}/src/StartUP/ing918)
include_directories(${ING_REL}/src/Tools)
include_directories(${ING_BUNDLE})
include_directories(${ING_BUNDLE}/os)
message("zyx debug includir ${ING_REL}/src/FWlib")
# target_sources(app PRIVATE ${mylib_src_dir}/src/gstartup_ing91800.s)
target_sources(app PRIVATE ${mylib_src_dir}/src/main.c)
target_sources(app PRIVATE ${mylib_src_dir}/src/profile.c)
target_sources(app PRIVATE ${mylib_src_dir}/src/service_console.c)
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
target_sources(app PRIVATE ${ING_REL}/src/Tools/trace.c)
target_sources(app PRIVATE ${ING_REL}/src/BSP/eflash.c)
add_definitions(-DUSE_STDPERIPH_DRIVER)
add_definitions(-DPOWER_SAVING)
# add_definitions(-DLISTEN_TO_POWER_SAVING)
add_definitions(-DLISTEN_TO_POWER_SAVING)
add_definitions(-DINGCHIPS_FAMILY_918=1)
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