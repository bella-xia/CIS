#include "registration.h"
#include <iostream>

int main(int argc, char **argv)
{
    // test 1 : point cloud registration
    Matrix eig1 = Matrix(1, 2, 3);
    Matrix eig2 = Matrix(4, 5, 6);
    Matrix eig3 = Matrix(7, 8, 9);
    Matrix eig4 = Matrix(1, 11, 8);

    Registration reg = Registration();
    reg.add_matrix_a(eig1);
    reg.add_matrix_a(eig2);
    reg.add_matrix_a(eig3);
    reg.add_matrix_a(eig4);
    reg.add_matrix_b(eig1);
    reg.add_matrix_b(eig2);
    reg.add_matrix_b(eig3);
    reg.add_matrix_b(eig4);
    Frame f = reg.point_cloud_registration();
    std::cout << f.get_pos().get_pos().as_str() << std::endl;
    std::cout << f.get_rot().get_rot().as_str() << std::endl;

    // test 2 : pivot calibration
    Rotation rot1 = Rotation(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1));
    Rotation rot2 = Rotation(Matrix(2, 0, 0, 0, 2, 0, 0, 0, 2));
    Rotation rot3 = Rotation(Matrix(3, 0, 0, 0, 3, 0, 0, 0, 3));
    Rotation rot4 = Rotation(Matrix(4, 0, 0, 0, 4, 0, 0, 0, 4));

    std::vector<Frame> f_vec;
    f_vec.push_back(Frame(rot1, Position(Matrix(0, 0, 0))));
    f_vec.push_back(Frame(rot2, Position(Matrix(25, 25, 25))));
    f_vec.push_back(Frame(rot3, Position(Matrix(50, 50, 50))));
    f_vec.push_back(Frame(rot4, Position(Matrix(75, 75, 75))));
    Matrix p_ts = reg.pivot_calibration(f_vec);
    p_ts.print_str();
}