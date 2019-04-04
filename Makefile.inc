#############################################################################
#
# Project Makefile
#
# (c) Wouter van Ooijen (www.voti.nl) 2016
#
# This file is in the public domain.
# 
#############################################################################

SEARCH  += $(internal_comm)/code/headers
SEARCH  += $(internal_comm)/code/src


# source files in this library
SOURCES += $(wildcard $(internal_comm)/code/src/*.cpp)

# header files in this library
HEADERS += $(wildcard $(internal_comm)/code/headers/*.hpp)

# include datastructures
SEARCH += ../datastructures/code/headers