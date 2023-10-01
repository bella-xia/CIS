#include "frame.h"

Matrix Frame::operator*(const Matrix &mat) const
{
    return rot.get_rot() * (rot.get_delta_rot() * mat + pos.get_delta_pos()) + pos.get_pos();
}