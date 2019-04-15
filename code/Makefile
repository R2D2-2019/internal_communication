#############################################################################
#
# Project Makefile
#
# (c) Wouter van Ooijen (www.voti.nl) 2016
#
# This file is in the public domain.
# 
#############################################################################

# source files in this project (main.cpp is automatically assumed)
SOURCES := $(wildcard src/*.cpp)

# header files in this project
HEADERS := $(wildcard headers/*.hpp)

# other places to look for files for this project
SEARCH  := ./headers ./srca

# set REATIVE to the next higher directory 
# and defer to the Makefile.due there
RELATIVE := $(RELATIVE)../
RESULTS += main.lss main.lst
include $(RELATIVE)Makefile.due

