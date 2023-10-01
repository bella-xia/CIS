#include "frame.h"
#include "function.h"
#include <iostream>

int main(int argc, char **argv)
{
    Matrix m_r = create_3_3(1, 2, 3, 4, 5, 6, 7, 8, 9);
    std::cout << m_r.asstr() << std::endl;
    Matrix alpha = create_3_1(3, 0, 1);
    std::cout << alpha.asstr() << std::endl;
    Matrix m_p = create_3_1(7, 1, 3);
    Matrix epsilon = create_3_1(3, 5, 8);
    Rotation r = Rotation(m_r, alpha);
    Position p = Position(m_p, epsilon);
    Frame f = Frame(r, p);

    Matrix pointer = create_3_1(2, 4, 5);

    std::cout << "skew of " << alpha.skew().asstr() << std::endl;
    std::cout << "delta rot of " << r.get_delta_rot().asstr() << std::endl;

    std::cout << (f * pointer).asstr() << std::endl;

    return 0;
}