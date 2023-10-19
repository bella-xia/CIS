#include "matrix.h"
#include <iostream>

int main(int argc, char **argv)
{
    Matrix m3_1 = Matrix(1.0, 2.0, 3.0);

    Matrix m1_3 = Matrix(1, 3);
    m1_3.assign(0, 0, 3.0);
    m1_3.assign(0, 1, 2.0);
    m1_3.assign(0, 2, 1.0);

    Matrix m1_1 = m1_3 * m3_1;
    Matrix m3_3 = m3_1 * m1_3;
    Matrix m3_3_T = m3_3.transpose();

    m3_3.print_str();
    m3_3_T.print_str();
    (m3_3 * m3_3_T).print_str();
    (m3_3_T * m3_3).print_str();
    (m3_3_T + m3_3).print_str();
    (m3_3_T * -1).print_str();
    return 0;
}