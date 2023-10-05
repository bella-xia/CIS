CXX_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp main.cpp
CXX_OBJS = $(CXX_SRCS:%.cpp=%.o)

CXX_MATRIX_SRCS = matrix.cpp  _matrix_test.cpp
CXX_MATRIX_OBJS = $(CXX_MATRIX_SRCS:%.cpp=%.o)

CXX_FRAME_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp _frame_test.cpp
CXX_FRAME_OBJS = $(CXX_FRAME_SRCS:%.cpp=%.o)

CXX_REGIS_SRCS = registration.cpp matrix.cpp rotation.cpp position.cpp frame.cpp _registration_test.cpp
CXX_REGIS_OBJS = $(CXX_REGIS_SRCS:%.cpp=%.o)

CXX_IO_SRCS = io_read.cpp _io_test.cpp
CXX_IO_OBJS = $(CXX_IO_SRCS:%.cpp=%.o)

CXX = g++ 
CXXFLAGS =  -g -Wall -std=c++17 -I./eigen-3.4.0

%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<

all : cis

matrix : $(CXX_MATRIX_OBJS)
	$(CXX) -o $@ $(CXX_MATRIX_OBJS)

frame : $(CXX_FRAME_OBJS)
	$(CXX) -o $@ $(CXX_FRAME_OBJS)

regis:  $(CXX_REGIS_OBJS)
	$(CXX) -o $@ $(CXX_REGIS_OBJS)

io:  $(CXX_IO_OBJS)
	$(CXX) -o $@ $(CXX_IO_OBJS)

cis : $(CXX_OBJS)
	$(CXX) -o $@ $(CXX_OBJS)

clean :
	rm -f *.o matrix frame regis cis io depend.mak

depend :
	$(CXX) $(CXXFLAGS) -M $(CXX_SRCS) >> depend.mak

depend.mak :
	touch $@

solution.zip : 
	zip -9r solution.zip *.h *.cpp Makefile README.txt

include depend.mak