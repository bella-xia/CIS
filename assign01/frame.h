#ifndef FRAME_H
#define FRAME_H

#include "position.h"
#include "rotation.h"
#include <string>

class Frame
{

private:
    std::string from;
    std::string to;
    Rotation rot;
    Position pos;

public:
    Frame() : from("hisashi"), to("taylor"), rot(Rotation()), pos(Position()) {}
    Frame(Rotation r, Position p) : from("taylor"), to("hisashi"), rot(r), pos(p) {}

    void assign_from(std::string from) { this->from = from; }
    void assign_to(std::string to) { this->to = to; }
    void assign_from_to(std::string from, std::string to)
    {
        this->from = from;
        this->to = to;
    }
    void assign_rot(Rotation rot) { this->rot = rot; }
    void assign_pos(Position pos) { this->pos = pos; }

    Rotation get_rot() const { return rot; }
    Position get_pos() const { return pos; }

    Matrix operator*(const Matrix &mat) const;
    Frame operator*(const Frame &f) const;
    Frame inverse() const;
};

#endif