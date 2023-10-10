#include "frame.h"

Matrix Frame::operator*(const Matrix &mat) const
{
    return rot.get_rot() * (rot.get_delta_rot() * mat + pos.get_delta_pos()) + pos.get_pos();
}

Frame Frame::operator*(const Frame &f) const
{

    Rotation mul_rot = Rotation(rot.get_rot() * f.get_rot().get_rot());
    Position mul_pos = Position(pos.get_pos() + rot.get_rot() * f.get_pos().get_pos());
    return Frame(mul_rot, mul_pos);
}

Frame Frame::inverse() const
{
    Rotation inverse_rot = Rotation(rot.get_rot().inverse());
    Position inverse_pos = Position(inverse_rot.get_rot() * pos.get_pos() * -1);

    Rotation inverse_rot_err = Rotation(rot.get_delta_rot().inverse());
    Position inverse_pos_err = Position(inverse_rot.get_delta_rot() * pos.get_delta_pos() * -1);
    return Frame(inverse_rot_err, inverse_pos_err) * Frame(inverse_rot, inverse_pos);
}