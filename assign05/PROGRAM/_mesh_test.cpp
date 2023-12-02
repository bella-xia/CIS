#include "mesh.h"

int main(int argc, char **argv)
{
    // initialize mesh
    Mesh mesh;

    std::vector<Matrix> vertices;
    srand(time(0));
    int z = rand() % 10;

    // push in all the vertices (which constitutes a 4 * 4 square on the yz plane of x = z)

    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            mesh.insert_vertex(z, j, i);
            vertices.push_back(Matrix(z, j, i));
        }
    }

    // push in all the triangular meshes
    std::vector<Matrix> random_points;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            int idx1 = i * 5 + j;
            int idx2 = i * 5 + j + 1;
            int idx3 = (i + 1) * 5 + j;
            mesh.insert_triangle(vertices.at(idx1), vertices.at(idx2), vertices.at(idx3),
                                 idx1, idx2, idx3, -1, -1, -1);
            float point1 = ((float)rand() / (RAND_MAX));
            float point2 = ((float)rand() / (RAND_MAX));
            float point3 = (1.0 - point2) / 2.0;
            random_points.push_back(Matrix(point1, j + point2, i + point3));
        }
    }

    // find all the closest points and print them together with the corresponding point of query

    std::vector<Matrix> closest_points = mesh.find_closest_point_advanced(random_points);

    for (int i = 0; i < (int)random_points.size(); i++)
    {
        std::cout << "input point: " << std::endl;
        random_points.at(i).print_str();
        std::cout << "closest point: " << std::endl;
        closest_points.at(i).print_str();
    }

    return 0;
}