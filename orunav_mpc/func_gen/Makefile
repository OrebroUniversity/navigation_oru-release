MAPLE=~/maple16/bin/maple

ifdef NLEN
maple:
	${MAPLE} -c N:=${NLEN} form_car_T_Sx0.mpl > out.txt
endif

clean:
	rm -f *.a *.o *.txt *.log

# dummy targets
.PHONY: clean maple
