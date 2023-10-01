#include "matrix.h"
#include "function.h"
#include <iostream>

int main(int argc, char **argv)
{
    Matrix m3_1 = Matrix(3, 1);
    m3_1.assign(0, 0, 1.0);
    m3_1.assign(1, 0, 2.0);
    m3_1.assign(2, 0, 3.0);

    Matrix m1_3 = Matrix(1, 3);
    m1_3.assign(0, 0, 3.0);
    m1_3.assign(0, 1, 2.0);
    m1_3.assign(0, 2, 1.0);

    Matrix m1_1 = m1_3 * m3_1;
    Matrix m3_3 = m3_1 * m1_3;
    Matrix m3_3_T = m3_3.transpose();

    std::cout << m3_3.asstr() << std::endl;
    std::cout << m3_3_T.asstr() << std::endl;
    std::cout << (m3_3 * m3_3_T).asstr() << std::endl;
    std::cout << (m3_3_T * m3_3).asstr() << std::endl;
    std::cout << (m3_3_T + m3_3).asstr() << std::endl;

    Eigen::MatrixXf eigen = convert_to_eigen(m3_3_T + m3_3);
    std::cout << eigen << std::endl;

    Matrix mat_return = convert_from_eigen(eigen);
    std::cout << mat_return.asstr() << std::endl;
    return 0;
}