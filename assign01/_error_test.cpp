#include "registration.h"

int main(int argc, char **argv)
{

    Matrix rot = Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    Matrix pos = Matrix(100, 50, 75);
    Matrix err = Matrix(0.3, 0.3, 0.3);

    Rotation rot_rot = Rotation(rot, err);
    Position pos_pos = Position(pos, err);

    Rotation neg_rot_rot = Rotation(rot, err * -1);
    Position neg_pos_pos = Position(pos, err * -1);

    Frame f_pr_np = Frame(rot_rot, neg_pos_pos);
    Frame f_nr_pp = Frame(neg_rot_rot, pos_pos);

    Matrix vec(329.483, 339.539, 335.146);

    Rotation delta_rot = Rotation(Matrix(3) + err.skew());
    Rotation neg_delta_rot = Rotation(Matrix(3) + (err * -1).skew());

    Frame f_no_err = Frame(Rotation(rot), Position(pos));
    Frame f_pos_rot_neg_pos_delta = Frame(Rotation(delta_rot), Position(err * -1));
    Frame f_neg_rot_pos_pos_delta = Frame(Rotation(neg_delta_rot), Position(err));

    std::cout
        << "original matrix" << std::endl;
    vec.print_str();

    // common frame transformation
    std::cout << "pos rot neg pos frame transformation through manual" << std::endl;
    (f_no_err * f_pos_rot_neg_pos_delta * vec).print_str();
    std::cout << "common frame transformation through program" << std::endl;
    (f_pr_np * vec).print_str();

    // inverse frame transformation
    std::cout << "inverse pos rot neg pos frame transformation through manual" << std::endl;
    (f_pos_rot_neg_pos_delta.inverse() * f_no_err.inverse() * vec).print_str();
    std::cout << "inverse frame transformation through program" << std::endl;
    (f_pr_np.inverse() * vec).print_str();

    // negative error frame transformation
    std::cout << "neg rot pos pos frame transformation through manual" << std::endl;
    (f_no_err * f_neg_rot_pos_pos_delta * vec).print_str();
    std::cout << "neg rot pos pos frame transformation through program" << std::endl;
    (f_nr_pp * vec).print_str();

    // negative error inverse frame transformation
    std::cout << "neg rot pos pos inverse frame transformation through manual" << std::endl;
    (f_neg_rot_pos_pos_delta.inverse() * f_no_err.inverse() * vec).print_str();
    std::cout << "neg rot pos pos inverse frame transformation through program" << std::endl;
    (f_nr_pp.inverse() * vec).print_str();

    return 0;
}