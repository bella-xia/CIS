#include "registration.h"
#include <iostream>

int main(int argc, char **argv)
{
    Eigen::MatrixXf eig1 = Eigen::MatrixXf(3, 1);
    Eigen::MatrixXf eig2 = Eigen::MatrixXf(3, 1);
    Eigen::MatrixXf eig3 = Eigen::MatrixXf(3, 1);
    Eigen::MatrixXf eig4 = Eigen::MatrixXf(3, 1);
    eig1 << 1,
        2,
        3;
    eig2 << 4,
        5,
        6;
    eig3 << 7,
        8,
        9;
    eig4 << 1,
        11,
        8;

    Registration reg = Registration();
    reg.add_matrix_a(eig1);
    reg.add_matrix_a(eig2);
    reg.add_matrix_a(eig3);
    reg.add_matrix_a(eig4);
    reg.add_matrix_b(eig1);
    reg.add_matrix_b(eig2);
    reg.add_matrix_b(eig3);
    reg.add_matrix_b(eig4);
    Eigen::MatrixXf m = reg.best_plane_from_points().transpose();
    std::cout << m << std::endl;
    auto svd = m.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf r = svd.matrixU() * svd.matrixV().transpose();
    std::cout << r << std::endl;
}