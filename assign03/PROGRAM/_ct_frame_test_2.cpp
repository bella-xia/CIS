#include "registration.h"
#include <cmath>

float make_noise(float noise_level = 0.0001)
{
    float random = rand();
    int sign = (rand() % 2 == 1) ? 1 : -1;
    while (random >= 1)
    {
        random = random / 10;
    }
    return sign * random * noise_level;
}

Matrix create_noise_vector(float noise_level = 0.0001)
{
    return Matrix(make_noise(noise_level), make_noise(noise_level), make_noise(noise_level));
}

Matrix generate_randomized_rotation(int dim = 1)
{
    int axis = rand() % 3;
    int sine = rand() % 2;
    int sign = (rand() % 2 == 1) ? 1 : -1;
    double divisor = (float)(rand() % 10) * 3 + 1;

    switch (axis)
    {
    case 0:
        return (dim == 1) ? Matrix(0,
                                   sine ? sin(sign * M_PI / divisor) : cos(sign * M_PI / divisor),
                                   sine ? cos(sign * M_PI / divisor) : sin(sign * M_PI / divisor))
                          : Matrix(1, 0, 0, 0, cos(sign * M_PI / divisor), -sin(sign * M_PI / divisor),
                                   0, sin(sign * M_PI / divisor), cos(sign * M_PI / divisor));
    case 1:
        return (dim == 1) ? Matrix(sine ? sin(sign * M_PI / divisor) : cos(sign * M_PI / divisor),
                                   0,
                                   sine ? cos(sign * M_PI / divisor) : sin(sign * M_PI / divisor))
                          : Matrix(cos(sign * M_PI / divisor), 0, sin(sign * M_PI / divisor),
                                   0, 1, 0,
                                   -sin(sign * M_PI / divisor), 0, cos(sign * M_PI / divisor));
    case 2:
        return (dim == 1) ? Matrix(sine ? sin(sign * M_PI / divisor) : cos(sign * M_PI / divisor),
                                   sine ? cos(sign * M_PI / divisor) : sin(sign * M_PI / divisor),
                                   0)
                          : Matrix(cos(sign * M_PI / divisor), -sin(sign * M_PI / divisor), 0,
                                   sin(sign * M_PI / divisor), cos(sign * M_PI / divisor), 0,
                                   0, 0, 1);
    default:
        return (dim == 1) ? Matrix(3, 1) : Matrix(3);
    }
}

Matrix generate_randomized_translation()
{
    // srand(time(0));
    return Matrix(rand() % 10, rand() % 10, rand() % 10);
}

int main(int argc, char **argv)
{
    srand(time(0));
    Matrix pos = Matrix(10, 17, 28);
    double radius = 10;
    double noise = 0.001;
    std::vector<std::vector<Matrix>> Mats;
    std::vector<Matrix> Mat_zero;
    for (int i = 1; i <= 20; ++i)
    {
        // all on the y axis
        Mat_zero.push_back(Matrix(0, radius / (float)i, 0) + pos + create_noise_vector(noise));
    }
    Mats.push_back(Mat_zero);
    for (int i = 0; i < 10; ++i)
    {
        std::vector<Matrix> Mat;
        Matrix rot = generate_randomized_rotation();
        for (int j = 1; j <= 20; ++j)
        {
            Mat.push_back(rot * (radius / (float)j) + pos);
        }
        Mats.push_back(Mat);
    }
    Registration Regis = Registration();
    std::vector<Frame> frames;
    for (int frame = 0; frame < (int)Mats.size(); ++frame)
    {
        for (int ele = 0; ele < (int)Mats[frame].size(); ++ele)
        {
            Regis.add_matrix_b(Mats[frame][ele]);
        }
        if (frame == 0)
        {
            Regis.get_matrix_a_from_b();
        }
        Frame f = Regis.point_cloud_registration();
        // f.get_rot().get_rot().print_str();
        // f.get_pos().get_pos().print_str();
        frames.push_back(f);
        Regis.clean_matrix_b();
    }
    Matrix m = Regis.pivot_calibration(frames);
    Matrix b_ptr(Matrix(m.get_pos(0, 0), m.get_pos(1, 0), m.get_pos(2, 0)));
    Matrix EM_ptr(Matrix(m.get_pos(3, 0), m.get_pos(4, 0), m.get_pos(5, 0)));

    std::cout << "calibrated EM ptr:  " << std::endl;
    EM_ptr.print_str();

    // create a new frame transformation
    Rotation rotation(generate_randomized_rotation(3));
    Position position(generate_randomized_translation());
    Frame generated_frame(rotation, position);
    std::cout << "randomly generated rotation and translation: " << std::endl;
    rotation.get_rot().print_str();
    position.get_pos().print_str();

    std::vector<Matrix> Bs;
    std::vector<Matrix> ct_bs;
    for (int i = 0; i < 20; ++i)
    {
        Frame local_to_EM(generate_randomized_rotation(3), generate_randomized_translation());
        for (int j = 1; j <= 20; ++j)
        {
            Regis.add_matrix_b(local_to_EM * (Matrix(0, radius / j, 0)) + create_noise_vector(noise));
        }
        Frame estimated_frame = Regis.point_cloud_registration();
        Regis.clean_matrix_b();
        // std::cout << "expected frame: " << std::endl;
        // local_to_EM.get_rot().get_rot().print_str();
        // local_to_EM.get_pos().get_pos().print_str();
        // std::cout << "estimated frame: " << std::endl;
        // estimated_frame.get_rot().get_rot().print_str();
        // estimated_frame.get_pos().get_pos().print_str();
        Bs.push_back(estimated_frame * b_ptr);
        ct_bs.push_back(generated_frame * local_to_EM.get_pos().get_pos());
    }

    // do point cloud registration in reference to the new frame
    Registration Regis_b = Registration();
    for (int i = 0; i < (int)Bs.size(); ++i)
    {
        Regis_b.add_matrix_a(Bs[i]);
        Regis_b.add_matrix_b(ct_bs[i]);
    }
    Frame calculated_f = Regis_b.point_cloud_registration();
    std::cout << "calculated rotation and translation: " << std::endl;
    calculated_f.get_rot().get_rot().print_str();
    calculated_f.get_pos().get_pos().print_str();

    std::vector<Matrix> navigationBs;
    std::vector<Matrix> navigationbs_expected;
    for (int i = 0; i < 5; ++i)
    {
        Matrix new_mat(rand() % 100, rand() % 100, rand() % 100);
        navigationBs.push_back(new_mat);
        navigationbs_expected.push_back(generated_frame * new_mat);
    }

    for (int i = 0; i < (int)navigationBs.size(); ++i)
    {
        std::cout << "Expected: " << std::endl;
        navigationbs_expected[i].transpose().print_str();
        std::cout << "Calculated: " << std::endl;
        (calculated_f * navigationBs[i]).transpose().print_str();
    }

    return 0;
}