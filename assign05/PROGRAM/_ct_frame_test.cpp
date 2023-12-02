#include <iostream>
#include "registration.h"
#include "interpolation.h"
#include "io_read.h"
#include <algorithm>
#include <random>
#include <chrono>

int main(int argc, char **argv)
{
    Registration f_g;
    Registration f_b;
    // construct b in CT frame
    std::vector<Matrix> b;
    b.push_back(Matrix(1, 0, 0));
    b.push_back(Matrix(0, -1, 0));
    b.push_back(Matrix(-1, 0, 0));
    b.push_back(Matrix(0, 1, 0));

    for (int i = 0; i < 4; i++)
    {
        f_b.add_matrix_b(b[i]);
    }
    Matrix ptr = Matrix(0, -1, 0);

    // construct g in pivot frame
    std::vector<Matrix> g;
    g.push_back(Matrix(1, 0, 0));
    g.push_back(Matrix(0, -1, 0));
    g.push_back(Matrix(-1, 0, 0));
    g.push_back(Matrix(0, 1, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_a(g[i]);
    }
    std::vector<Matrix> B;
    /*no rotation*/
    // construct G in EM frame (EM center: (10, 10, 0))

    // Frame 1
    std::vector<Matrix> G_f1;
    G_f1.push_back(Matrix(-8, -9, 0));
    G_f1.push_back(Matrix(-9, -10, 0));
    G_f1.push_back(Matrix(-10, -9, 0));
    G_f1.push_back(Matrix(-9, -8, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f1[i]);
    }

    Frame fg1 = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B.push_back(fg1 * ptr);

    // Frame 2
    std::vector<Matrix> G_f2;
    G_f2.push_back(Matrix(-9, -10, 0));
    G_f2.push_back(Matrix(-10, -11, 0));
    G_f2.push_back(Matrix(-11, -10, 0));
    G_f2.push_back(Matrix(-10, -9, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f2[i]);
    }

    Frame fg2 = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B.push_back(fg2 * ptr);

    // Frame 3
    std::vector<Matrix> G_f3;
    G_f3.push_back(Matrix(-10, -9, 0));
    G_f3.push_back(Matrix(-11, -10, 0));
    G_f3.push_back(Matrix(-12, -9, 0));
    G_f3.push_back(Matrix(-11, -8, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f3[i]);
    }

    Frame fg3 = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B.push_back(fg3 * ptr);

    // Frame 4

    std::vector<Matrix> G_f4;
    G_f4.push_back(Matrix(-9, -8, 0));
    G_f4.push_back(Matrix(-10, -9, 0));
    G_f4.push_back(Matrix(-11, -8, 0));
    G_f4.push_back(Matrix(-10, -7, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f4[i]);
    }

    Frame fg4 = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B.push_back(fg4 * ptr);

    // CT frame registration
    for (int i = 0; i < 4; i++)
    {
        f_b.add_matrix_a(B[i]);
    }
    Frame f_ct = f_b.point_cloud_registration();

    // nav G

    std::vector<Matrix> G_fnav;
    G_fnav.push_back(Matrix(-9, -9, 0));
    G_fnav.push_back(Matrix(-10, -10, 0));
    G_fnav.push_back(Matrix(-11, -9, 0));
    G_fnav.push_back(Matrix(-10, -8, 0));

    for (int i = 0; i < 4; i++)
    {

        f_g.add_matrix_b(G_fnav[i]);
    }

    Frame fgnav = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    Matrix navB = fgnav * ptr;
    Matrix result = f_ct * navB;
    result.print_str(); // expected: (0, 0, 0)

    /*test rotation*/
    std::vector<Matrix> B_rot;

    // Frame 1
    std::vector<Matrix> G_f1_rot;
    G_f1_rot.push_back(Matrix(-8, -11, 0));
    G_f1_rot.push_back(Matrix(-9, -10, 0));
    G_f1_rot.push_back(Matrix(-8, -9, 0));
    G_f1_rot.push_back(Matrix(-7, -10, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f1_rot[i]);
    }

    Frame fg1_rot = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B_rot.push_back(fg1_rot * ptr);

    // Frame 2

    std::vector<Matrix> G_f2_rot;
    G_f2_rot.push_back(Matrix(-11, -12, 0));
    G_f2_rot.push_back(Matrix(-10, -11, 0));
    G_f2_rot.push_back(Matrix(-9, -12, 0));
    G_f2_rot.push_back(Matrix(-10, -13, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f2_rot[i]);
    }

    Frame fg2_rot = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B_rot.push_back(fg2_rot * ptr);

    // Frame 3

    std::vector<Matrix> G_f3_rot;
    G_f3_rot.push_back(Matrix(-12, -9, 0));
    G_f3_rot.push_back(Matrix(-11, -10, 0));
    G_f3_rot.push_back(Matrix(-12, -11, 0));
    G_f3_rot.push_back(Matrix(-13, -10, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f3_rot[i]);
    }

    Frame fg3_rot = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B_rot.push_back(fg3_rot * ptr);

    // frame 4

    std::vector<Matrix> G_f4_rot;
    G_f4_rot.push_back(Matrix(-9, -8, 0));
    G_f4_rot.push_back(Matrix(-10, -9, 0));
    G_f4_rot.push_back(Matrix(-11, -8, 0));
    G_f4_rot.push_back(Matrix(-10, -7, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_f4_rot[i]);
    }

    Frame fg4_rot = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    B_rot.push_back(fg4_rot * ptr);
    f_b.clean_matrix_a();

    for (int i = 0; i < 4; i++)
    {
        f_b.add_matrix_a(B_rot[i]);
    }

    Frame f_ct_rot = f_b.point_cloud_registration();

    // nav G

    std::vector<Matrix> G_fnav_rot;
    G_fnav_rot.push_back(Matrix(-9, -9, 0));
    G_fnav_rot.push_back(Matrix(-10, -10, 0));
    G_fnav_rot.push_back(Matrix(-11, -9, 0));
    G_fnav_rot.push_back(Matrix(-10, -8, 0));

    for (int i = 0; i < 4; i++)
    {
        f_g.add_matrix_b(G_fnav_rot[i]);
    }

    Frame fgnav_rot = f_g.point_cloud_registration();
    f_g.clean_matrix_b();
    Matrix navB_rot = fgnav_rot * ptr;
    Matrix result_rot = f_ct_rot * navB_rot;
    result_rot.print_str(); // expected: (0, 0, 0)

    // add rotation between patient and ct form
    f_b.clean_matrix_b();
    std::vector<Matrix> b_rot;
    b_rot.push_back(Matrix(0, -1, 0));
    b_rot.push_back(Matrix(-1, 0, 0));
    b_rot.push_back(Matrix(0, 1, 0));
    b_rot.push_back(Matrix(1, 0, 0));

    for (int i = 0; i < 4; i++)
    {
        f_b.add_matrix_b(b_rot[i]);
    }

    Frame f_ct_rot_2 = f_b.point_cloud_registration();
    Matrix result_rot_2 = f_ct_rot_2 * navB_rot;
    result_rot_2.print_str(); // expected: (0, 0, 0)

    return 0;
}