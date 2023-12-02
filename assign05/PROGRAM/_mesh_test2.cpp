#include "mesh.h"
#include <cmath>
#include <random>

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
void generateSphereMesh(Mesh &mesh, const Frame &rand_frame,
                        const int divisions, const int radiusX, const int radiusY, const int radiusZ)
{
    std::vector<Matrix> vertices;

    // construct vertices, add frame transformation in the meantime
    for (int i = 0; i <= divisions; ++i)
    {
        float phi = static_cast<float>(i) / divisions * M_PI;
        for (int j = 0; j <= 2 * divisions; ++j)
        {
            float theta = static_cast<float>(j) / (2 * divisions) * (2 * M_PI);

            float x = radiusX * sin(phi) * cos(theta);
            float y = radiusY * sin(phi) * sin(theta);
            float z = radiusZ * cos(phi);

            Matrix mat = rand_frame * Matrix(x, y, z);

            vertices.push_back(mat);
            mesh.insert_vertex(mat.get_pos(0, 0), mat.get_pos(1, 0), mat.get_pos(2, 0));
        }
    }

    // based on the verticies, construct the triangular meshes
    for (int i = 0; i < divisions; ++i)
    {
        for (int j = 0; j < 2 * divisions; ++j)
        {
            int p0 = i * (2 * divisions + 1) + j;
            int p1 = p0 + 1;
            int p2 = (i + 1) * (2 * divisions + 1) + j;
            int p3 = p2 + 1;

            mesh.insert_triangle(vertices.at(p0), vertices.at(p1), vertices.at(p2),
                                 p0, p1, p2, -1, -1, -1);
            mesh.insert_triangle(vertices.at(p1), vertices.at(p3), vertices.at(p2),
                                 p1, p3, p2, -1, -1, -1);
        }
    }
}

int main(int argc, char **argv)
{
    // initialize mesh
    Mesh mesh;
    const float radiusX = 15.0;
    const float radiusY = 10.0;
    const float radiusZ = 5.0;
    const int divisions = 50;

    std::vector<Matrix> random_points;
    srand(time(0));

    // create randomized frame transformation
    Frame rand_frame = Frame(Rotation(generate_randomized_rotation(2)),
                             Position(generate_randomized_translation()));

    // push in all the vertices (which constitutes a 4 * 4 square on the yz plane of x = z)
    generateSphereMesh(mesh, rand_frame, divisions, radiusX, radiusY, radiusZ);

    for (int i = 0; i < 100; ++i)
    {
        random_points.push_back(generate_random_point(radiusX, radiusY, radiusZ));
    }

    // find all the closest points and print them together with the corresponding point of query
    std::tuple<Frame, std::vector<Matrix>> output = mesh.find_optimum_transformation(random_points, 0.99);
    // std::vector<Matrix> closest_points = std::get<1>(output);
    Frame estimate = std::get<0>(output);

    // check result
    std::cout << "randomly generated frame transformation: " << std::endl;
    rand_frame.get_rot().get_rot().print_str();
    rand_frame.get_pos().get_pos().print_str();
    std::cout << "estimated frame transformation: " << std::endl;
    estimate.get_rot().get_rot().print_str();
    estimate.get_pos().get_pos().print_str();
    return 0;
}