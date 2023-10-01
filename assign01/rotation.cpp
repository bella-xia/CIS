#include "rotation.h"

Matrix Rotation::get_delta_rot() const
{
    if (delta_kind == angle)
    {
        return Matrix(3) + delta_axis.skew();
    }
    else
    {
        return delta_rot;
    }
}