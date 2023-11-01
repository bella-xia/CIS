#ifndef FRAME_H
#define FRAME_H

#include "position.h"
#include "rotation.h"
#include <string>

class Frame
{

private:
    Rotation rot;
    Position pos;

public:
    //default constructor: assign an empty rotation and empty translation.
    Frame() : rot(Rotation()), pos(Position()) {}

    //constructor basing on a given rotation and translation
    Frame(Rotation r, Position p) : rot(r), pos(p) {}

    //set rotation
    void assign_rot(Rotation rot) { this->rot = rot; }

    //set translation
    void assign_pos(Position pos) { this->pos = pos; }

    //get rotation
    Rotation get_rot() const { return rot; }

    //get translation
    Position get_pos() const { return pos; }

    //define dot product operator of the Frame * Matrix
    Matrix operator*(const Matrix &mat) const;

    //define dot product operator of the Frame * Frame
    Frame operator*(const Frame &f) const;

    //calculate the inverse of a frame.
    Frame inverse() const;
};

#endif