DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

HAVE_BANKING=1
UIP_CONF_IPV6=1
UIP_CONF_RPL=1
OFFSET_FIRMWARE=1
STACK_CONF_DEBUGGING=1

PROJECT_SOURCEFILES += mqtt-sn.c

CONTIKI_PROJECT = example

APPS += mqtts

all: example

CONTIKI = ../../..

include $(CONTIKI)/Makefile.include
