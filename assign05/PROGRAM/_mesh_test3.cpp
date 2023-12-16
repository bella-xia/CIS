#include "mesh.h"
#include "io_read.h"
#include <cmath>
#include <random>

float make_mode()
{
    int sign = (rand() % 2 == 1) ? 1 : -1;
    float rand_choice = ((float)(rand() % 5)) * 0.01;
    return sign * rand_choice;
}
/**
 * helper function to generate random rotational matrix
 */
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
/**
 * helper function to generate random translation matrix
 */
Matrix generate_randomized_translation()
{
    // srand(time(0));
    return Matrix(rand() % 10, rand() % 10, rand() % 10);
}

/**
 * helper function to generate random point on an ellipsoid based on given
 * x, y, z radii
 */
Matrix generate_random_point(const int radiusX, const int radiusY, const int radiusZ)
{

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> phi_dist(-M_PI, M_PI);
    std::uniform_real_distribution<float> theta_dist(0.0, 2.0 * M_PI);

    float phi = phi_dist(gen);
    float theta = theta_dist(gen);

    float x = radiusX * sin(phi) * cos(theta);
    float y = radiusY * sin(phi) * sin(theta);
    float z = radiusZ * cos(phi);

    return Matrix(x, y, z);
}

/**
 * helper function to generate a 3D surface containing
 * 2 * divisions^2 number of vertices and triangular meshes
 * on the ellipsoid with the given radii x, y, z, calculate
 * the point after a specified frame transformation rand_frame,
 * then store all the vertices and triangular meshes into the mesh class
 */
std::vector<std::vector<Matrix>> generateSphereMeshModes(const Frame &rand_frame, const int divisions,
                                                         const int radiusX, const int radiusY, const int radiusZ,
                                                         int n_m)
{
    std::vector<std::vector<Matrix>> modes;
    for (int j = 0; j <= n_m; ++j)
    {
        modes.push_back(std::vector<Matrix>());
    }

    // insert mode 0 matrices
    for (int i = 0; i <= divisions; ++i)
    {
        float phi = static_cast<float>(i) / divisions * M_PI;
        for (int j = 0; j <= 2 * divisions; ++j)
        {
            float theta = static_cast<float>(j) / (2 * divisions) * (2 * M_PI);

            float x = radiusX * sin(phi) * cos(theta);
            float y = radiusY * sin(phi) * sin(theta);
            float z = radiusZ * cos(phi);

            Matrix mat = Matrix(x, y, z);
            modes.at(0).push_back(mat);
            for (int k = 1; k <= n_m; ++k)
            {
                modes.at(k).push_back(Matrix(make_mode(), make_mode(), make_mode()));
            }
        }
    }
    return modes;
}

int main(int argc, char **argv)
{
    // initialize mesh
    const float radiusX = 15.0;
    const float radiusY = 10.0;
    const float radiusZ = 20.0;
    const int divisions = 50;
    const int n_m = 5;
    std::vector<Matrix> mode_nm;
    for (int i = 0; i < n_m; ++i)
    {
        mode_nm.push_back(Matrix(make_mode(), make_mode(), make_mode()));
    }

    std::vector<Matrix> random_points;
    std::vector<float> lambdas{-5.0, 10.0, -20.0, 15.0, -25.0};
    srand(time(0));

    // create randomized frame transformation
    Frame rand_frame = Frame(Rotation(generate_randomized_rotation(2)),
                             Position(generate_randomized_translation()));

    std::vector<std::vector<Matrix>> modes(generateSphereMeshModes(rand_frame, divisions, radiusX, radiusY,
                                                                   radiusZ, n_m));
    Mesh mesh(n_m, modes);
    int prob = (divisions * 2 * divisions) / 100;
    int nums = 0;
    for (int i = 0; i < divisions; ++i)
    {
        for (int j = 0; j < 2 * divisions; ++j)
        {
            int p0 = i * (2 * divisions + 1) + j;
            int p1 = p0 + 1;
            int p2 = (i + 1) * (2 * divisions + 1) + j;
            int p3 = p2 + 1;

            mesh.insert_triangle(modes.at(0).at(p0), modes.at(0).at(p1), modes.at(0).at(p2),
                                 -1, -1, -1, p0, p1, p2);
            mesh.insert_triangle(modes.at(0).at(p1), modes.at(0).at(p3), modes.at(0).at(p2),
                                 -1, -1, -1, p1, p3, p2);
            int first_or_sec = rand() % 2;
            int point_or_not = rand() % prob;

            if (point_or_not == 0)
            {
                nums++;
                if (first_or_sec == 0)
                {
                    Matrix rand_point = modes.at(0).at(p0) * 0.3 + modes.at(0).at(p1) * 0.3 + modes.at(0).at(p2) * 0.4;
                    for (int k = 1; k <= n_m; ++k)
                    {
                        rand_point = rand_point + (modes.at(k).at(p0) * 0.3 + modes.at(k).at(p1) * 0.3 +
                                                   modes.at(k).at(p2) * 0.4) *
                                                      lambdas.at(k - 1);
                    }
                    random_points.push_back(rand_frame.inverse() * rand_point);
                }
                else
                {
                    Matrix rand_point = modes.at(0).at(p1) * 0.3 + modes.at(0).at(p3) * 0.3 + modes.at(0).at(p2) * 0.4;
                    for (int k = 1; k <= n_m; ++k)
                    {
                        rand_point = rand_point + (modes.at(k).at(p1) * 0.3 + modes.at(k).at(p3) * 0.3 +
                                                   modes.at(k).at(p2) * 0.4) *
                                                      lambdas.at(k - 1);
                    }
                    random_points.push_back(rand_frame.inverse() * rand_point);
                }
            }
        }
    }
    std::cout << "a total of " << nums << " numbers are chosen " << std::endl;

    /*
        Matrix offset(0, 0, 0);
        for (int i = 0; i < n_m; i++)
        {
            offset = offset + mode_nm.at(i) * lambdas.at(i);
        }

        for (int i = 0; i < 100; ++i)
        {
            Matrix random_point = generate_random_point(radiusX, radiusY, radiusZ);
            random_points.push_back(random_point + offset);
        }
        */

    auto output = mesh.deformed_find_optimum_transformation(random_points, 0.95);
    // std::vector<Matrix> closest_points = std::get<1>(output);
    Frame estimate = std::get<0>(output);
    std::vector<float> lambdas_est = mesh.get_lambdas();

    // check result
    std::cout << "randomly generated frame transformation: " << std::endl;
    rand_frame.get_rot().get_rot().print_str();
    rand_frame.get_pos().get_pos().print_str();
    std::cout << "estimated frame transformation: " << std::endl;
    estimate.get_rot().get_rot().print_str();
    estimate.get_pos().get_pos().print_str();

    std::cout << "customized lambdas: " << std::endl;
    std::cout << lambdas.at(0) << " " << lambdas.at(1) << " " << lambdas.at(2) << " ";
    std::cout << lambdas.at(3) << " " << lambdas.at(4) << std::endl;
    std::cout << "estimated lambdas: " << std::endl;
    std::cout << lambdas_est.at(0) << " " << lambdas_est.at(1) << " " << lambdas_est.at(2) << " ";
    std::cout << lambdas_est.at(3) << " " << lambdas_est.at(4) << std::endl;

    return 0;
}