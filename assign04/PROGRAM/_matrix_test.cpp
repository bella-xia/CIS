#include "matrix.h"
#include <iostream>

int main(int argc, char **argv)
{

    // create a 3 * 1 matrix
    Matrix m3_1 = Matrix(1.0, 2.0, 3.0);
    // create a 1 * 3 matrix
    Matrix m1_3 = Matrix(1, 3);

    // test assignment of particular index in matrices
    m1_3.assign(0, 0, 3.0);
    m1_3.assign(0, 1, 2.0);
    m1_3.assign(0, 2, 1.0);

    // test matrix multiplication and transpose
    Matrix m1_1 = m1_3 * m3_1;
    Matrix m3_3 = m3_1 * m1_3;
    Matrix m3_3_T = m3_3.transpose();

    // ensure the outcome is as expected
    m3_3.print_str();
    m3_3_T.print_str();
    (m3_3 * m3_3_T).print_str();
    (m3_3_T * m3_3).print_str();
    (m3_3_T + m3_3).print_str();
    (m3_3_T * -1).print_str();
    return 0;
}