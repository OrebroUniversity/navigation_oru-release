# Time-stamp: <2012-07-10 08:28:22 (drdv)>

include ../const_def.mk

all:
#	g++ -o a example_M1BB.cpp $(CFLAGS) $(INCLUDES) $(LIBS) 
#	g++ -o a example_initial_guess.cpp $(CFLAGS) $(INCLUDES) $(LIBS) 
#	g++ -o a example_M1S3.cpp $(CFLAGS) $(INCLUDES) $(LIBS)
#	g++ -o a example_M1S2.cpp $(CFLAGS) $(INCLUDES) $(LIBS)
#	g++ -o a example_M1S1.cpp $(CFLAGS) $(INCLUDES) $(LIBS)
#	g++ -o a example_M1FBF_1.cpp $(CFLAGS) $(INCLUDES) $(LIBS)
#	g++ -o a example_M1_test.cpp $(CFLAGS) $(INCLUDES) $(LIBS)
#	g++ -o a demo.cpp $(CFLAGS) $(INCLUDES) $(LIBS)
	g++ -o a SingleStage.cpp $(CFLAGS) $(INCLUDES) $(LIBS)



#	rm -rf a.dSYM # something related to MAC OS

clean:
	rm -f *.o
	rm -f a
	cd $(MATLAB_DIR); rm -f *.txt 
	rm -rf a.dSYM # MAC OS related


# dummy targets
.PHONY: clean

###EOF
