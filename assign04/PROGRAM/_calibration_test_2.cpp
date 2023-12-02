#include "registration.h"
#include <cmath>

/**
 * helper function used to generate random noises in the data
 */
float make_noise(float noise_level = 0.0001)
{

    srand(time(0));
    float random = rand();
    int sign = (rand() % 2 == 1) ? 1 : -1;
    while (random >= 1)
    {
        random = random / 10;
    }
    return sign * random * noise_level;
}

/**
 * helper function used to construct a 3 * 1 matrix of random noise
 */
Matrix create_noise_vector(float noise_level = 0.0001)
{
    return Matrix(make_noise(noise_level), make_noise(noise_level), make_noise(noise_level));
}

int main(int argc, char **argv)
{
    srand(time(0));

    // define all the constants here
    const Matrix pos = Matrix(10, 17, 28);
    const double radius = 10;
    const float noise = 0.001;

    // initiate the matrix vectors to store the matrices at each frame in the global frame (Mats)
    // and the local frame matrices (Mat_zero)
    std::vector<std::vector<Matrix>> Mats;
    std::vector<Matrix> Mat_zero;

    // push back all the local frame points
    for (int i = 1; i <= 20; ++i)
    {
        // all on the y axis
        Mat_zero.push_back(Matrix(0, radius / i, 0) + pos);
    }
    Mats.push_back(Mat_zero);

    // construct rotations around the tip of the probe, then
    // recalculate each points in respect to the new position in a position
    // that has a translation transformation of pos from the probe tip.
    for (int i = 0; i < 20; ++i)
    {
        // axis 0 = x, 1 = y, 2 = z
        int axis = rand() % 3;
        int sine = rand() % 2;
        int sign = (rand() % 2 == 1) ? 1 : -1;
        // divisor from 1 to 5
        double divisor = rand() % 5 + 1;
        std::vector<Matrix> Mat;
        for (int j = 1; j <= 20; ++j)
        {
            switch (axis)
            {
            case 0:
                Mat.push_back(Matrix(0,
                                     sine ? sin(sign * M_PI / divisor) * (radius / j) : cos(sign * M_PI / divisor) * (radius / j),
                                     sine ? cos(sign * M_PI / divisor) * (radius / j) : sin(sign * M_PI / divisor) * (radius / j)) +
                              pos + create_noise_vector(noise));
                break;
            case 1:
                Mat.push_back(Matrix(sine ? sin(sign * M_PI / divisor) * (radius / j) : cos(sign * M_PI / divisor) * (radius / j),
                                     0,
                                     sine ? cos(sign * M_PI / divisor) * (radius / j) : sin(sign * M_PI / divisor) * (radius / j)) +
                              pos + create_noise_vector(noise));
                break;
            case 2:
                Mat.push_back(Matrix(
                                  sine ? sin(sign * M_PI / divisor) * (radius / j) : cos(sign * M_PI / divisor) * (radius / j),
                                  sine ? cos(sign * M_PI / divisor) * (radius / j) : sin(sign * M_PI / divisor) * (radius / j),
                                  0) +
                              pos + create_noise_vector(noise));
                break;
            default:
                break;
            }
        }
        Mats.push_back(Mat);
    }
    // for_each(mats.begin(),
    //          mats.end(),
    //          [](const auto &mat)
    //          {
    //              for_each(mat.begin(),
    //                       mat.end(),
    //                       [](const auto &element)
    //                       {
    //                           // printing one by one element
    //                           // separated with space
    //                           element.print_str();
    //                       });
    //          });

    // push in all the global frame matrices, and use it to calculate an intermediate local
    // frame
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
        // get the frame transformation estimate for each frame of points corresponding to
        // the constructed local frame
        Frame f = Regis.point_cloud_registration();
        // f.get_rot().get_rot().print_str();
        // f.get_pos().get_pos().print_str();
        frames.push_back(f);
        Regis.clean_matrix_b();
    }
    // perform pivot calibration based on all the frame estimations
    Matrix m = Regis.pivot_calibration(frames);
    m.print_str();
    return 0;
}