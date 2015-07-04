
AC_PREREQ([2.69])
AC_INIT([libI2CWrapper], [0.1], [])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_HEADERS([config.h])



MACHINE="-DSTM32F4"

CXXFLAGS="$CXXFLAGS -I/usr/local/arm-none-eabi/include -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g $MACHINE -nostdlib -fno-exceptions -fno-rtti "

CFLAGS="$CFLAGS -I/usr/local/arm-none-eabi/include -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g  $MACHINE -nostdlib"

LDFLAGS="$LDFLAGS -L/usr/local/arm-none-eabi/lib/ -mfloat-abi=hard -nostdlib -Wl,--gc-sections"



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


AC_CONFIG_FILES([
		 Makefile
		 ])

AC_OUTPUT
