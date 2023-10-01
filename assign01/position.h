#ifndef POSITION_H
#define POSITION_H

#include "matrix.h"

class Position
{

private:
    Matrix m_pos;
    Matrix delta_pos;

public:
    Position() : m_pos(Matrix(3, 1)), delta_pos(Matrix(3, 1)) {}
    Position(Matrix pos) : m_pos(pos), delta_pos(Matrix(3, 1)) {}
    Position(Matrix pos, Matrix err) : m_pos(pos), delta_pos(err) {}

    void assign_err(Matrix err) { delta_pos = err; }

    Matrix get_pos() const { return m_pos; }
    Matrix get_delta_pos() const { return delta_pos; }
};

#endif