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
GOAL  = $(LIB_DIR)/wmcommon.$(LIBTYPE)

#---------------------------------------------------------------------------
# Source section (User)
#---------------------------------------------------------------------------
ASM_SRC :=
C_SRC := fwup/wm_fwup.c \
	 mem/wm_mem.c \
	 Params/wm_param.c \
	 task/wm_wl_mbox.c \
	 task/wm_wl_task.c \
	 task/wm_wl_timers.c \
	 utils/utils.c \
	 crypto/digest/hmac.c \
	 crypto/digest/md2.c \
	 crypto/digest/md4.c \
	 crypto/digest/sha224.c \
	 crypto/digest/sha384.c \
	 crypto/digest/sha512.c \
	 crypto/keyformat/asn1.c \
	 crypto/keyformat/base64.c \
	 crypto/keyformat/x509.c \
	 crypto/math/pstm_mul_comba.c \
	 crypto/prng/prng.c \
	 crypto/prng/yarrow.c \
	 crypto/pubkey/dh.c \
	 crypto/pubkey/ecc.c \
	 crypto/pubkey/pkcs.c \
	 crypto/pubkey/pubkey.c \
	 crypto/pubkey/rsa.c \
	 crypto/symmetric/aesGCM.c \
	 crypto/symmetric/des3.c \
	 crypto/symmetric/idea.c \
	 crypto/symmetric/rc2.c \
	 crypto/symmetric/seed.c 
#special for venus
ifeq ($(ARCH_M3),1)
C_SRC += crypto/wm_crypto_hard.c
endif

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
	
