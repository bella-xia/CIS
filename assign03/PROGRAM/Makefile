CXX_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp triangle_mesh.cpp bounding_sphere.cpp bounding_box_tree_node.cpp mesh.cpp main_pa3.cpp
CXX_OBJS = $(CXX_SRCS:%.cpp=%.o)

CXX_PA3_STATS_SRCS = pa3_comp_stats.cpp
CXX_PA3_STATS_OBJS = $(CXX_PA3_STATS_SRCS:%.cpp=%.o)

CXX_PA3_TEST_SRCS = matrix.cpp triangle_mesh.cpp bounding_sphere.cpp bounding_box_tree_node.cpp mesh.cpp io_read.cpp pa3_result_without_regis.cpp
CXX_PA3_TEST_OBJS = $(CXX_PA3_TEST_SRCS:%.cpp=%.o)

CXX_PA1_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp main_pa1.cpp
CXX_PA1_OBJS = $(CXX_PA1_SRCS:%.cpp=%.o)

CXX_PA2_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp interpolation.cpp main_pa2.cpp
CXX_PA2_OBJS = $(CXX_PA2_SRCS:%.cpp=%.o)

CXX_MESH_SRCS = matrix.cpp triangle_mesh.cpp bounding_sphere.cpp bounding_box_tree_node.cpp mesh.cpp _mesh_test.cpp
CXX_MESH_OBJS = $(CXX_MESH_SRCS:%.cpp=%.o)

CXX_MATRIX_SRCS = matrix.cpp  _matrix_test.cpp
CXX_MATRIX_OBJS = $(CXX_MATRIX_SRCS:%.cpp=%.o)

CXX_FRAME_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp _frame_test.cpp
CXX_FRAME_OBJS = $(CXX_FRAME_SRCS:%.cpp=%.o)

CXX_REGIS_SRCS = registration.cpp matrix.cpp rotation.cpp position.cpp frame.cpp _registration_test.cpp
CXX_REGIS_OBJS = $(CXX_REGIS_SRCS:%.cpp=%.o)

CXX_ERROR_SRCS = registration.cpp matrix.cpp rotation.cpp position.cpp frame.cpp _error_test.cpp
CXX_ERROR_OBJS = $(CXX_ERROR_SRCS:%.cpp=%.o)

CXX_CALONE_SRCS = registration.cpp matrix.cpp rotation.cpp position.cpp frame.cpp _calibration_test_1.cpp
CXX_CALONE_OBJS = $(CXX_CALONE_SRCS:%.cpp=%.o)

CXX_CALTWO_SRCS = registration.cpp matrix.cpp rotation.cpp position.cpp frame.cpp _calibration_test_2.cpp
CXX_CALTWO_OBJS = $(CXX_CALTWO_SRCS:%.cpp=%.o)

CXX_COMP_SRCS = registration.cpp matrix.cpp rotation.cpp position.cpp frame.cpp io_read.cpp output_comp.cpp
CXX_COMP_OBJS = $(CXX_COMP_SRCS:%.cpp=%.o)

CXX_IO_SRCS = io_read.cpp _io_test.cpp
CXX_IO_OBJS = $(CXX_IO_SRCS:%.cpp=%.o)

CXX_INTERP_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp interpolation.cpp _interpolation_test.cpp
CXX_INTERP_OBJS = $(CXX_INTERP_SRCS:%.cpp=%.o)

CXX_CT_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp interpolation.cpp _ct_frame_test.cpp
CXX_CT_OBJS = $(CXX_CT_SRCS:%.cpp=%.o)

CXX_CTTWO_SRCS = matrix.cpp rotation.cpp position.cpp frame.cpp registration.cpp io_read.cpp interpolation.cpp _ct_frame_test_2.cpp
CXX_CTTWO_OBJS = $(CXX_CTTWO_SRCS:%.cpp=%.o)

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

calone:  $(CXX_CALONE_OBJS)
	$(CXX) -o $@ $(CXX_CALONE_OBJS)

caltwo:  $(CXX_CALTWO_OBJS)
	$(CXX) -o $@ $(CXX_CALTWO_OBJS)

err:  $(CXX_ERROR_OBJS)
	$(CXX) -o $@ $(CXX_ERROR_OBJS) 

cis : $(CXX_OBJS)
	$(CXX) -o $@ $(CXX_OBJS)

comp: $(CXX_COMP_OBJS)
	$(CXX) -o $@ $(CXX_COMP_OBJS)

interp: $(CXX_INTERP_OBJS)
	$(CXX) -o $@ $(CXX_INTERP_OBJS)

ct: $(CXX_CT_OBJS)
	$(CXX) -o $@ $(CXX_CT_OBJS)

ct2: $(CXX_CTTWO_OBJS)
	$(CXX) -o $@ $(CXX_CTTWO_OBJS)

mesh: $(CXX_MESH_OBJS)
	$(CXX) -o $@ $(CXX_MESH_OBJS)

pa1: $(CXX_PA1_OBJS)
	$(CXX) -o $@ $(CXX_PA1_OBJS)

pa2: $(CXX_PA2_OBJS)
	$(CXX) -o $@ $(CXX_PA2_OBJS)

pa3-test: $(CXX_PA3_TEST_OBJS)
	$(CXX) -o $@ $(CXX_PA3_TEST_OBJS)

pa3-stats: $(CXX_PA3_STATS_OBJS)
	$(CXX) -o $@ $(CXX_PA3_STATS_OBJS)
clean :
	rm -f *.o matrix frame regis cis calone caltwo io err comp pa1 pa2 pa3-test pa3-stats interp mesh ct ct2 depend.mak

depend :
	$(CXX) $(CXXFLAGS) -M $(CXX_SRCS) >> depend.mak

depend.mak :
	touch $@

solution.zip : 
	zip -9r solution.zip ../PROGRAM ../PA1\ Student\ Data ../OUTPUT ../TEST ../README.txt

include depend.mak