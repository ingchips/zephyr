CMSIS_BASE=./wizard/CMSIS
CMSIS_INC="${CMSIS_BASE}/Include"
KERNEL_INC= ./kernel/include
ZEPHYR_INC= ./include
ING_REL=./ING918XX_SDK_SOURCE
ING_BUNDLE=./ING918XX_SDK_SOURCE/bundles/noos_typical/inc
SYMDEFS=-Wl,--just-symbols=./ING918XX_SDK_SOURCE/bundles/noos_typical/ING9187xx/symdefs.g

APPNAME=my_project_noos

STARTUP=src/gstartup_ing91800.s

APPDEFS=-D USE_STDPERIPH_DRIVER

PROJECT_FILES  = src/main.c
PROJECT_FILES += src/profile.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_adc.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_gpio.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_i2c.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_pinctrl.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_pwm.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_rtc.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_ssp.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_sysctrl.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_timer.c
PROJECT_FILES += ${ING_REL}/src/FWlib/peripheral_uart.c
PROJECT_FILES += ${ING_REL}/src/Tools/trace.c
PROJECT_FILES += ${ING_REL}/src/BSP/eflash.c
include makefile.conf
