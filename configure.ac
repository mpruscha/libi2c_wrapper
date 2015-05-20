#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([FULL-PACKAGE-NAME], [VERSION], [BUG-REPORT-ADDRESS])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_HEADERS([config.h])



#CXXFLAGS="-Os -mthumb -mcpu=cortex-m4 -lrdimon -specs=rdimon.specs"


#MACHINE="-DSTM32F4"

#FP_FLAGS="-mfloat-abi=hard -mfpu=fpv4-sp-d16"

#CXXFLAGS="-Wall -Wextra -Wredundant-decls -Wundef -Wshadow  -Iinclude -fno-common  -mcpu=cortex-m4 -mthumb $FP_FLAGS -ffunction-sections -fdata-sections -MD $MACHINE"


CXXFLAGS="$CXXFLAGS -I/usr/local/arm-none-eabi/include -mthumb -mcpu=cortex-m4 --specs=nosys.specs -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lrdimon -specs=rdimon.specs"
LDFLAGS="$LDFLAGS -L/usr/local/arm-none-eabi/lib/ -mfloat-abi=hard -Wl,--gc-sections"



# Checks for programs.
AC_PROG_CXX #([arm-none-eabi-g++])
AC_PROG_CC #([arm-none-eabi-gcc])


AC_PROG_RANLIB

AM_PROG_AR





# Checks for libraries.
AC_SEARCH_LIBS([rcc_periph_clock_enable], [opencm3_stm32f4], [], [
		  AC_MSG_ERROR([unable to find the dlopen() function])
		  ])


# Checks for header files.
AC_CHECK_HEADERS([inttypes.h stdint.h stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.
#AC_CHECK_HEADER_STDBOOL

# Checks for library functions.
#AC_FUNC_MALLOC
#AC_CHECK_FUNCS([memset sqrt])

AC_CONFIG_FILES([
		 Makefile
		 ])

AC_OUTPUT
