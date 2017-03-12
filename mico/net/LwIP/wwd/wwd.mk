#
# Copyright 2013, Broadcom Corporation
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Broadcom Corporation.
#

NAME := WWD_LwIP_Interface_$(RTOS)

GLOBAL_INCLUDES := .

$(NAME)_SOURCES := wwd_network.c \
                   wwd_buffer.c

$(NAME)_COMPONENTS := Wiced/Network/LwIP/wwd/$(RTOS)

$(NAME)_CFLAGS  = $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)

