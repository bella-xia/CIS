#include "frame.h"
#include <iostream>

int main(int argc, char **argv)
{
    // create a 3 * 3 matrix as "fake" rotational matrix
    Matrix m_r = Matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    std::cout << m_r.as_str() << std::endl;
    // create its rotational error vector
    Matrix alpha = Matrix(3, 0, 1);
    std::cout << alpha.as_str() << std::endl;
    // create a 3 * 1 matrix as translation matrix, altogether with its
    // err vector
    Matrix m_p = Matrix(7, 1, 3);
    Matrix epsilon = Matrix(3, 5, 8);

    // construct the frame transformation based on the four terms
    Rotation r = Rotation(m_r, alpha);
    Position p = Position(m_p, epsilon);
    Frame f = Frame(r, p);

    // choose a random point
    Matrix pointer = Matrix(2, 4, 5);

    // test the skew function gives the same result as the error rotation (should be)
    std::cout << "skew of " << alpha.skew().as_str() << std::endl;
    std::cout << "delta rot of " << r.get_delta_rot().as_str() << std::endl;

    // test the result of frame transformation on pointer
    std::cout << (f * pointer).as_str() << std::endl;

    return 0;
}