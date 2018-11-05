#/**************************************************************************
# * Kevin 2014-02-24                                                       *
# **************************************************************************/

#---------------------------------------------------------------------------
# Constant Variable definition
#---------------------------------------------------------------------------

ifeq ($(COMPILER_OS_CYGWIN),1)
TOPDIR=../..
endif

include $(TOPDIR)/Tools/toolchain.def

#---------------------------------------------------------------------------
# Target definition (User)
#---------------------------------------------------------------------------
GOAL  = $(LIB_DIR)/wmdriver.$(LIBTYPE)

#---------------------------------------------------------------------------
# Source section (User)
#---------------------------------------------------------------------------

#---------------------------------------------------------------------------
# Link Library Section
#---------------------------------------------------------------------------
ASM_SRC := 
C_SRC := adc/wm_adc.c \
	 cpu/wm_cpu.c \
	 dma/wm_dma.c \
	 efuse/wm_efuse.c \
	 flash/wm_fls.c \
	 flash/wm_fls_gd25qxx.c \
	 gpio/wm_gpio.c \
	 gpio/wm_gpio_afsel.c \
	 hspi/wm_hspi.c \
	 i2c/wm_i2c.c \
	 irq/wm_irq.c \
	 pwm/wm_pwm.c \
	 pmu/wm_pmu.c \
	 rtc/wm_rtc.c \
	 spi/wm_hostspi.c \
	 timer/wm_timer.c \
	 uart/wm_uart.c \
	 watchdog/wm_watchdog.c 
#special for venus
ifeq ($(ARCH_M3),1)
C_SRC += internalflash/wm_internal_fls.c \
	 i2s/wm_i2s.c \
	 io/wm_io.c \
	 lcd/wm_lcd.c \
	 7816/wm_7816.c
endif

#---------------------------------------------------------------------------
# Implicit rules
#---------------------------------------------------------------------------
.c.o:
	$(CC) $(CFLAGS) -c -o $*.o $< $(INCLUDES)

.s.o:
	$(ASM) $(ASMFLAGS) -o $*.o $< $(INCLUDES)

#---------------------------------------------------------------------------
# Explicit ruls
#---------------------------------------------------------------------------
OBJ_FILES	= $(C_SRC:%.c=%.o) $(ASM_SRC:%.s=%.o)

all: $(GOAL)

$(GOAL): $(OBJ_FILES) $(LIB_SRC)
ifeq ($(TOOL_GNU),1)
		$(AR) $(ARFLAGS) $@ $(OBJ_FILES)
else
		$(AR) --create $@ $(OBJ_FILES) $(LIB_SRC)
endif

.PHONY: clean
clean:
	$(RM) -f $(GOAL)
	$(RM) -f $(OBJ_FILES:.o=.d) $(OBJ_FILES)
	

