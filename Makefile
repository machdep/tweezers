APP = 		tweezers
MACHINE =	mips

CC =		${CROSS_COMPILE}gcc
LD =		${CROSS_COMPILE}ld
OBJCOPY =	${CROSS_COMPILE}objcopy

LDSCRIPT =	${CURDIR}/ldscript

OSDIR =		mdepx
OBJDIR =	obj

export CFLAGS =								\
	-O -pipe -g -nostdinc -fno-pic -mno-abicalls -G0		\
	-mmicromips -march=mips32r2 -EL -msoft-float -ffreestanding	\
	-fwrapv	-gdwarf-2 -fno-common -fms-extensions			\
	-finline-limit=8000 -std=iso9899:1999 -fno-builtin-printf	\
	-Wall -Wredundant-decls -Wnested-externs			\
	-Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith	\
	-Winline -Wcast-qual -Wundef -Wno-pointer-sign			\
	-Wmissing-include-dirs -fdiagnostics-show-option		\
	-Wno-unknown-pragmas -Wno-uninitialized -Werror			\
	-D__mips_o32							\

export AFLAGS = ${CFLAGS}

all:
	@python3 -B mdepx/tools/emitter.py -j mdepx.conf
	@${OBJCOPY} -O srec obj/tweezers.elf obj/tweezers.srec
	@${CROSS_COMPILE}size obj/tweezers.elf

clean:
	@rm -f ${OBJECTS} ${OBJDIR}/${APP}.*

include ${OSDIR}/mk/user.mk
