
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
        f_b.add_matrix_a(b[i]);
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

    G_f3.push_back(Matrix(-11, -8, 0));

    G_f3.push_back(Matrix(-12, -9, 0));

    G_f3.push_back(Matrix(-11, -10, 0));

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
        f_b.add_matrix_b(B[i]);
    }

    Frame f_ct = f_b.point_cloud_registration();

    Matrix result = f_ct * Matrix(-10, -10, 0);

    result.print_str();

    return 0;
}