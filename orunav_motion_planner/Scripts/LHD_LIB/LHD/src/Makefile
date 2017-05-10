# Time-stamp: <2012-06-04 18:42:34 (drdv)>

include ../const_def.mk

#===============================
CODE = \
	OptionsLHD.cpp \
	State.cpp \
	Polynomial.cpp \
	SimpleBounds.cpp \
	StageParameters.cpp \
	ViaRegion.cpp \
	StageOutput.cpp \
	LHD_utility.cpp \
	Maneuver.cpp \
	Maneuver_ACADO.cpp
#===============================

all: 
	@echo Compiling on $(SYSTYPE)
	@echo ====================================================================
	$(CC) $(CFLAGS) $(INCLUDES) -c $(CODE)
	${AR} -r $(LDIR)/$(TARGET) *.o
	@echo ====================================================================
	cd $(MAPLE_DIR); make
	@echo ====================================================================

clean:
	rm -f *.o
	cd $(LDIR); rm -f *.a 
	cd $(MAPLE_DIR); make clean

clean-all:
	make clean
	cd $(MAIN_DIR)/examples; make clean
	cd $(MATLAB_DIR); rm -f *.txt
	cd $(MATLAB_DIR)/figures; make clean
	cd $(MAPLE_DIR); make clean
	cd $(DOC_DIR); make clean 


# dummy targets
.PHONY: clean

###EOF