# This is the Part of original library 

MYLIB_INC = $(MYLIB)
MYLIB_SRC = $(MYLIB)

LIBINCDIRS += $(MYLIB_INC)

CFILES += \
 $(MYLIB_SRC)/timer.c 				\
 $(MYLIB_SRC)/rc_io.c				\
 $(MYLIB_SRC)/i2c.c					\
 $(MYLIB_SRC)/fram.c


