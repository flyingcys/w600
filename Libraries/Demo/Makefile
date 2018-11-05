#/**************************************************************************
# * Kevin 2014-02-24                                                       *
# **************************************************************************/

#---------------------------------------------------------------------------
# Constant Variable definition
#---------------------------------------------------------------------------

ifeq ($(COMPILER_OS_CYGWIN),1)
TOPDIR=..
endif

include $(TOPDIR)/Tools/toolchain.def

#---------------------------------------------------------------------------
# Target definition (User)
#---------------------------------------------------------------------------
GOAL  = $(LIB_DIR)/wmdemo.$(LIBTYPE)

#---------------------------------------------------------------------------
# Source section (User)
#---------------------------------------------------------------------------
ASM_SRC			+= 
      
C_SRC := wm_crypt_demo.c \
	 wm_apsta_demo.c  \
	 wm_mcast_demo.c  \
	 wm_raw_socket_server_demo.c \
	 wm_socket_client_demo.c \
	 wm_socket_fwup_demo.c \
	 wm_socket_server_demo.c \
	 wm_socket_server_sel_demo.c \
	 wm_softap_demo.c \
	 wm_udp_demo.c \
	 wm_i2c_demo.c \
	 wm_i2s_demo.c \
	 wm_adc_demo.c \
	 wm_7816_demo.c \
	 wm_uart_demo.c \
	 ./console/wm_demo_console_task.c \
	 wm_gpio_demo.c \
	 wm_flash_demo.c \
	 wm_pwm_demo.c \
	 wm_slave_spi_demo.c \
	 wm_master_spi_demo.c \
	 wm_rtc_demo.c  \
	 wm_timer_demo.c \
	 wm_pmu_demo.c \
	 wm_crypt_hard_demo.c \
	 wm_rsa_demo.c \
	 wm_raw_socket_client_demo.c \
	 wm_connect_net_demo.c \
	 wm_http_demo.c \
	 wm_wps_demo.c \
	 wm_websockets_demo.c \
	 wm_ssl_server_demo.c \
	 wm_ntp_demo.c
#C_SRC				+= wm_dlna_dmr_demo.c
#C_SRC				+= wm_spi_codec_demo.c 
#---------------------------------------------------------------------------
# Implicit rules
#---------------------------------------------------------------------------
.c.o:
	$(CC) $(CFLAGS) -c -o $*.o $< $(INCLUDES)

.s.o:
	$(ASM) $(ASMFLAGS) -c -o $*.o $< $(INCLUDES)

#---------------------------------------------------------------------------
# Explicit ruls
#---------------------------------------------------------------------------
OBJ_FILES	= $(C_SRC:%.c=%.o) $(ASM_SRC:%.s=%.o)

all: $(GOAL)

$(GOAL): $(OBJ_FILES)
	$(AR) $(ARFLAGS) $@ $(OBJ_FILES)
	
.PHONY: clean
clean:
	$(RM) -f $(GOAL)
	$(RM) -f $(OBJ_FILES:.o=.d) $(OBJ_FILES)
	
