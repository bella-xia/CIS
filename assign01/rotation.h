#ifndef ROTATION_H
#define ROTATION_H

#include "matrix.h"

enum rotKind
{
    angle = 100,
    matrix

};

class Rotation
{

private:
    enum rotKind m_kind;
    enum rotKind delta_kind;
    Matrix m_rot;
    Matrix delta_rot;
    float m_angle;
    float delta_angle;
    Matrix m_axis;
    Matrix delta_axis;

public:
    Rotation() : m_kind(matrix), delta_kind(angle), m_rot(Matrix(3, 3)), delta_angle(0.0), delta_axis(Matrix(3, 1)) {}
    Rotation(Matrix m_rot, Matrix err) : m_kind(matrix), delta_kind(angle), m_rot(m_rot), delta_axis(err), delta_angle(err.magnitude()) {}

    void assign_err(Matrix err)
    {
        delta_axis = err;
        delta_angle = err.magnitude();
    }

    void assign_err(Matrix err, float ferr)
    {
        delta_axis = err;
        delta_angle = ferr;
    }

    Matrix get_rot() const { return m_rot; }
    Matrix get_delta_rot() const;
};

#endif