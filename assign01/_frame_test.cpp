#include "frame.h"
#include <iostream>

int main(int argc, char **argv)
{
    Matrix m_r = Matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    std::cout << m_r.as_str() << std::endl;
    Matrix alpha = Matrix(3, 0, 1);
    std::cout << alpha.as_str() << std::endl;
    Matrix m_p = Matrix(7, 1, 3);
    Matrix epsilon = Matrix(3, 5, 8);
    Rotation r = Rotation(m_r, alpha);
    Position p = Position(m_p, epsilon);
    Frame f = Frame(r, p);

    Matrix pointer = Matrix(2, 4, 5);

    std::cout << "skew of " << alpha.skew().as_str() << std::endl;
    std::cout << "delta rot of " << r.get_delta_rot().as_str() << std::endl;

    std::cout << (f * pointer).as_str() << std::endl;

    return 0;
}