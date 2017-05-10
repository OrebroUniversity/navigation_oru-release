## Time-stamp: <2012-06-04 16:03:59 (drdv)>

##
## definition of compilation constants
##

## ---------------------------------------------
## compiler to use 
## ---------------------------------------------
CC = g++
AR = ar

## ---------------------------------------------
## store system type
## ---------------------------------------------
SYSTYPE = $(shell uname -s)

## ---------------------------------------------
## compilation flags
## ---------------------------------------------

# to profile using gprof add -pg 
#CFLAGS = -DLINUX -Wall -pedantic -Wfloat-equal -Wshadow -Winline -D__DEBUG__ -g -O3 -finline-functions -Wno-parentheses

CFLAGS = -DLINUX -Wall -pedantic -Wfloat-equal -Wshadow -Winline -O3 -finline-functions -Wno-parentheses

## ---------------------------------------------
## define object and library directories
## as seen from the local folders
## ---------------------------------------------
MAIN_DIR = ..

LDIR = $(MAIN_DIR)/lib
MAPLE_DIR = $(MAIN_DIR)/Maple
MATLAB_DIR = $(MAIN_DIR)/Matlab
DOC_DIR = $(MAIN_DIR)/doc

ifeq ($(SYSTYPE), Linux)
ACADO_DIR = $(MAIN_DIR)/../../external_libraries/ACADOtoolkit-1.0.2613beta
endif

ifeq ($(SYSTYPE), Darwin)
ACADO_DIR = /Users/mitko/Desktop/ACADOtoolkit-1.0.2613beta
endif

## ---------------------------------------------
## includes and libraries
## ---------------------------------------------
INCLUDES = \
	-I$(ACADO_DIR)/include \
	-I$(ACADO_DIR)/new_features/include \
	-I$(ACADO_DIR)/external_packages \
	-I$(ACADO_DIR)/external_packages/qpOASES-2.0/INCLUDE \
	-I$(MAIN_DIR)/include

LIBS = \
	-L$(LDIR) -lLHD \
	-L$(ACADO_DIR)/libs \
	-lacado_toolkit \
	-lqpOASESextras2.0 \
	-lcsparse \
	../Maple/get_abc.o ../Maple/get_parameters_xy.o

TARGET = libLHD.a

## ---------------------------------------------
## other
## ---------------------------------------------
CURRENT_DIR = $(PWD)

###EOF
