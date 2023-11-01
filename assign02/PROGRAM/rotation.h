#ifndef ROTATION_H
#define ROTATION_H

#include "matrix.h"

class Ang_rep
{
    /**
     * This class aim to create a rotation object in angle/axis form to store the rotation of a frame. It contains:
     * The basic information of the rotation: axis, angle
     * The basic get and set functions.
     */

private:
    Matrix m_axis;
    float m_angle;

public:
    // default constructor of the rotation
    Ang_rep() : m_axis(Matrix(3, 1)), m_angle(0.0) {}

    // construct a rotation given axis and angle
    Ang_rep(Matrix axis, float ang) : m_axis(axis), m_angle(ang) {}

    // return the axis of the rotation
    Matrix get_axis() const { return m_axis; }

    // return the angle of the rotation
    float get_angle() const { return m_angle; }

    // set the axis of the rotation
    void assign_axis(Matrix axis) { m_axis = axis; }

    // set the angle of the rotation
    void assign_angle(float ang) { m_angle = ang; }
};

class Rotation
{
    /**
     * This class aim to create a rotation object in matrix form to store the rotation of a frame. It contains:
     * The basic information of the rotation: rotational matrix, error of rotation
     * The basic get and set functions.
     */
private:
    Matrix m_rot; // rotational matrix

    Ang_rep delta_rot; // error matrix

public:
    // default constructor
    Rotation() : m_rot(Matrix(3)), delta_rot(Ang_rep())
    {
    }

    // construct a rotation object given rotation matrix but no error
    Rotation(Matrix m_rot) : m_rot(m_rot), delta_rot(Ang_rep())
    {
    }

    // construct a rotation object given rotation and error in matrix form
    Rotation(Matrix m_rot, Matrix err) : m_rot(m_rot), delta_rot(Ang_rep(err, err.magnitude()))
    {
    }

    // set the error of the rotation given an error of matrix form
    void assign_err(Matrix err)
    {
        delta_rot = Ang_rep(err, err.magnitude());
    }

    // set the error of the rotation given an error of axis/angle form
    void assign_err(Matrix err, float ferr)
    {
        delta_rot = Ang_rep(err, ferr);
    }

    // get the rotation matrix
    Matrix get_rot() const { return m_rot; }

    // get the error matrix
    Matrix get_delta_rot() const { return Matrix(3) + delta_rot.get_axis().skew(); }

    ~Rotation();
};

#endif