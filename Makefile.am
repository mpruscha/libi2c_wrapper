

AUTOMAKE_OPTIONS  = subdir-objects

MACHINE = -DSTM32F4

#FP_FLAGS        = -mfloat-abi=hard -mfpu=fpv4-sp-d16
#PREFIX          = arm-none-eabi

#CC              = $(PREFIX)-gcc
#AR              = $(PREFIX)-ar
#CFLAGSx          = -Wall -Wextra -Wimplicit-function-declaration \
#		  -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes \
#		  -Wundef -Wshadow \
#		  -Iinclude -fno-common \
#		  -mcpu=cortex-m4 -mthumb $(FP_FLAGS) \
#		  -Wstrict-prototypes \
#		  -ffunction-sections -fdata-sections -MD $(MACHINE)



AM_CPPFLAGS = -I../include $(CXXFLAGS)

lib_LIBRARIES = libi2c_wrapper.a

include_HEADERS = ../include/i2c_wrapper.h

libi2c_wrapper_a_SOURCES = ../src/i2c_wrapper.cpp
