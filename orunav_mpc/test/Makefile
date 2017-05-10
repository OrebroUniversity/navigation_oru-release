#WARN_FLAGS=-Wall -Wfloat-equal -Wshadow -pedantic
WARN_FLAGS=-Wall -Wshadow -pedantic
IFLAGS+=-I../include/ -I../qpOASES/include/ -I../func_gen
LDFLAGS+=-lm -lpthread -lcanlib -L../../../../../devel/lib/ -lfuncgen -lqpOASES
#LIB_DIR=../lib/

ifdef DEBUG
	LDFLAGS+=-g -pg
	CXXFLAGS=-g -pg ${WARN_FLAGS} ${IFLAGS}
	CFLAGS=-g -pg ${WARN_FLAGS} ${IFLAGS}
else
	CXXFLAGS=-O3 -DNDEBUG ${WARN_FLAGS} ${IFLAGS}
	CFLAGS=-O3 -DNDEBUG ${WARN_FLAGS} ${IFLAGS}
endif


all: telnet can maple qp qpc canread chpos

telnet:
	${CXX} ${CXXFLAGS} -c test_01_telnet.cpp
	${CXX} -o test_01_telnet.a test_01_telnet.o

can:
	${CXX} ${CXXFLAGS} -c test_02_can.cpp
	${CXX} -o test_02_can.a test_02_can.o ${LDFLAGS}

maple:
	${CXX} ${CXXFLAGS} -c test_03_maple.cpp
	${CXX} -o test_03_maple.a test_03_maple.o ${LDFLAGS}

qp:
	${CXX} ${CXXFLAGS} -c test_04_qp.cpp
	${CXX} -o test_04_qp.a test_04_qp.o ${LDFLAGS}

qpc:
	${CXX} ${CXXFLAGS} -c test_05_qpc.cpp
	${CXX} -o test_05_qpc.a test_05_qpc.o ${LDFLAGS}

canread:
	${CXX} ${CXXFLAGS} -c test_06_canread.cpp
	${CXX} -o test_06_canread.a test_06_canread.o ${LDFLAGS}

chpos:
	${CXX} ${CXXFLAGS} -c test_07_chpos.cpp
	${CXX} -o test_07_chpos.a test_07_chpos.o ${LDFLAGS}

clean:
	rm -f *.a *.o *.txt

# dummy targets
.PHONY: clean telnet can maple qp
