#include "registration.h"

int main(int argc, char **argv)
{
    std::vector<Frame> frames;
    Registration reg1 = Registration();
    reg1.add_matrix_a(Matrix(0, 11, 0));
    reg1.add_matrix_a(Matrix(1, 10, 0));
    reg1.add_matrix_a(Matrix(0, 9, 0));
    reg1.add_matrix_a(Matrix(-1, 10, 0));

    reg1.add_matrix_b(Matrix(0,1,0));
    reg1.add_matrix_b(Matrix(1,0,0));
    reg1.add_matrix_b(Matrix(0,-1,0));
    reg1.add_matrix_b(Matrix(-1,0,0));
    Frame f1 = reg1.point_cloud_registration();
    Position pos1 = f1.get_pos();
    std::cout<<"pos1:"<<std::endl;
    pos1.get_pos().print_str();
    frames.push_back(f1);
    reg1.clean_matrix_b();

    reg1.add_matrix_b(Matrix(11,-10,0));
    reg1.add_matrix_b(Matrix(10,-11,0));
    reg1.add_matrix_b(Matrix(9,-10,0));
    reg1.add_matrix_b(Matrix(10,-9,0));
    Frame f2 = reg1.point_cloud_registration();
    Position pos2 = f2.get_pos();
    std::cout<<"pos2:"<<std::endl;
    pos2.get_pos().print_str();
    frames.push_back(f2);
    reg1.clean_matrix_b();

    reg1.add_matrix_b(Matrix(0,-21,0));
    reg1.add_matrix_b(Matrix(-1,-20,0));
    reg1.add_matrix_b(Matrix(0,-19,0));
    reg1.add_matrix_b(Matrix(1,-20,0));
    Frame f3 = reg1.point_cloud_registration();
    Position pos3 = f3.get_pos();
    std::cout<<"pos3:"<<std::endl;
    pos3.get_pos().print_str();
    frames.push_back(f3);
    reg1.clean_matrix_b();
    
    reg1.add_matrix_b(Matrix(-11,-10,0));
    reg1.add_matrix_b(Matrix(-10,-9,0));
    reg1.add_matrix_b(Matrix(-9,-10,0));
    reg1.add_matrix_b(Matrix(-10,-11,0));
    Frame f4 = reg1.point_cloud_registration();
    Position pos4 = f4.get_pos();
    std::cout<<"pos4:"<<std::endl;
    pos4.get_pos().print_str();
    frames.push_back(f4);
    reg1.clean_matrix_b();
    Matrix a = reg1.pivot_calibration(frames);
    a.print_str();

    return 0;
}