APP = 		tweezers
MACHINE =	mips

CC =		${CROSS_COMPILE}gcc
LD =		${CROSS_COMPILE}ld
OBJCOPY =	${CROSS_COMPILE}objcopy

LDSCRIPT =	${CURDIR}/ldscript

OSDIR =		mdepx
OBJDIR =	obj

all:
	@python3 -B mdepx/tools/emitter.py -j mdepx.conf
	@${OBJCOPY} -O srec obj/tweezers.elf obj/tweezers.srec
	@${CROSS_COMPILE}size obj/tweezers.elf

clean:
	@rm -f ${OBJECTS} ${OBJDIR}/${APP}.*

include ${OSDIR}/mk/user.mk
