#include "registration.h"
#include <cmath>

int main(int argc, char **argv)
{
    Matrix pos = Matrix(10, 17, 28);
    double radius = 10;
    std::vector<std::vector<Matrix>> Mats;
    std::vector<Matrix> Mat_zero;
    for (int i = 1; i <= 8; ++i)
    {
        // all on the y axis
        Mat_zero.push_back(Matrix(0, radius / i, 0) + pos);
    }
    Mats.push_back(Mat_zero);
    for (int i = 0; i < 10; ++i)
    {
        // axis 0 = x, 1 = y, 2 = z
        int axis = rand() % 3;
        int sine = rand() % 2;
        // divisor from 1 to 5
        double divisor = rand() % 5 + 1;
        std::vector<Matrix> Mat;
        for (int j = 1; j <= 8; ++j)
        {
            switch (axis)
            {
            case 0:
                Mat.push_back(Matrix(0,
                                     sine ? sin(M_PI / divisor) * (radius / j) : cos(M_PI / divisor) * (radius / j),
                                     sine ? cos(M_PI / divisor) * (radius / j) : sin(M_PI / divisor) * (radius / j)) +
                              pos);
                break;
            case 1:
                Mat.push_back(Matrix(sine ? sin(M_PI / divisor) * (radius / j) : cos(M_PI / divisor) * (radius / j),
                                     0,
                                     sine ? cos(M_PI / divisor) * (radius / j) : sin(M_PI / divisor) * (radius / j)) +
                              pos);
                break;
            case 2:
                Mat.push_back(Matrix(
                                  sine ? sin(M_PI / divisor) * (radius / j) : cos(M_PI / divisor) * (radius / j),
                                  sine ? cos(M_PI / divisor) * (radius / j) : sin(M_PI / divisor) * (radius / j),
                                  0) +
                              pos);
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
    m.print_str();
    return 0;
}