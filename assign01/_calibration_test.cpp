#include "registration.h"

int main(int argc, char **argv)
{
    Frame f = Frame(Rotation(Matrix(1, 0, 0, 0, 0, -1, 0, 1, 0)), Position(Matrix(34, 52, -10)));
    Matrix fm1 = f * Matrix(100, 200, 300);
    Matrix fm2 = f * Matrix(50, 45, 128);
    Matrix fm3 = f * Matrix(85, 40, 30);
    Matrix fm4 = f * Matrix(65, 20, 10);
    Matrix fm5 = f * Matrix(50, 50, -50);
    Matrix fm6 = f * Matrix(0, 0, 0);
    Matrix fm7 = f * Matrix(1, 12, 123);
    Registration regis = Registration();
    regis.add_matrix_b(fm1);
    regis.add_matrix_b(fm2);
    regis.add_matrix_b(fm3);
    regis.add_matrix_b(fm4);
    return 0;
}