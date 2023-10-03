#ifndef ROTATION_H
#define ROTATION_H

#include "matrix.h"

// enum rotKind
// {
//     angle = 100,
//     matrix

// };

class Ang_rep
{
private:
    Matrix m_axis;
    float m_angle;

public:
    Ang_rep() : m_axis(Matrix(3, 1)), m_angle(0.0) {}
    Ang_rep(Matrix axis, float ang) : m_axis(axis), m_angle(ang) {}
    Matrix get_axis() const { return m_axis; }
    float get_angle() const { return m_angle; }
    void assign_axis(Matrix axis) { m_axis = axis; }
    void assign_angle(float ang) { m_angle = ang; }
};

class Rotation
{

private:
    Matrix m_rot;

    Ang_rep delta_rot;

public:
    Rotation() : m_rot(Matrix(3, 3)), delta_rot(Ang_rep())
    {
    }

    Rotation(Matrix m_rot, Matrix err) : m_rot(m_rot), delta_rot(Ang_rep(err, err.magnitude()))
    {
    }

    void assign_err(Matrix err)
    {
        delta_rot = Ang_rep(err, err.magnitude());
    }

    void assign_err(Matrix err, float ferr)
    {
        delta_rot = Ang_rep(err, ferr);
    }

    Matrix get_rot() const { return m_rot; }
    Matrix get_delta_rot() const { return Matrix(3) + delta_rot.get_axis().skew(); }
    ~Rotation();
};

#endif