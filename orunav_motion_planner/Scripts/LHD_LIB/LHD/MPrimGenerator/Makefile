# Time-stamp: <2012-07-10 08:28:22 (drdv)>

include ../const_def.mk

all:
	g++ -o SingleStage SingleStage.cpp $(CFLAGS) $(INCLUDES) $(LIBS) 

clean:
	rm -f *.o
	rm -f a
	cd $(MATLAB_DIR); rm -f *.txt 
	rm -rf a.dSYM # MAC OS related


# dummy targets
.PHONY: clean

###EOF
