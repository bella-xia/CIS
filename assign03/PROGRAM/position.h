#ifndef POSITION_H
#define POSITION_H

#include "matrix.h"

class Position
{
/**
 * This class aim to create a position object to store the translation of a frame. It contains:
 * The basic information of the translation: p and p_err
 * The basic get and set functions.
*/

private:
    Matrix m_pos; // p
    Matrix delta_pos; // p_err

public:
    // Default constructor
    Position() : m_pos(Matrix(3, 1)), delta_pos(Matrix(3, 1)) {}

    // Consructor with only translation input but no error input
    Position(Matrix pos) : m_pos(pos), delta_pos(Matrix(3, 1)) {}

    // Constructor basing on the translation input and error input.
    Position(Matrix pos, Matrix err) : m_pos(pos), delta_pos(err) {}

    ~Position();

    // set the error of the translation
    void assign_err(Matrix err)
    {
        delta_pos = err;
    }
    // return only the translation value
    Matrix get_pos() const { return m_pos; }

    // return only the error value.
    Matrix get_delta_pos() const { return delta_pos; }
};

#endif