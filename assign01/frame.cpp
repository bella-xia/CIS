#include "frame.h"

Matrix Frame::operator*(const Matrix &mat) const
{
    return rot.get_rot() * (rot.get_delta_rot() * mat + pos.get_delta_pos()) + pos.get_pos();
}

Frame Frame::inverse() const
{
    Rotation inverse_rot = Rotation(rot.get_rot().inverse());
    Position inverse_pos = Position(inverse_rot.get_rot() * pos.get_pos() * -1);
    return Frame(inverse_rot, inverse_pos);
}