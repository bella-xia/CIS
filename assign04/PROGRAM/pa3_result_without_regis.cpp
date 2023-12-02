#include "mesh.h"
#include "registration.h"
#include "io_read.h"
#include <iostream>
#include <iomanip>

std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks);
std::string formatLine(Matrix d, Matrix s);
std::string formatNum(float a);

/**
 * read the given registration from specified filename
 */
void read_post_regis(std::string filename, std::vector<Matrix> &points)
{
    std::ifstream in_file;
    in_file.open(filename);
    std::string in_line;
    // this is used to read the first three integers
    int n_mat;
    float v1, v2, v3;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_mat;
    for (int i = 0; i < n_mat; ++i)
    {
        std::getline(in_file, in_line);
        std::stringstream(in_line) >> v1 >> v2 >> v3;
        points.push_back(Matrix(v1, v2, v3));
    }
}

int main(int argc, char **argv)
{

    std::string idx = argv[1];
    if (argc > 1)
    {
        idx = argv[1];
    }

    Mesh mesh;
    std::vector<Matrix> s_ks;

    // define the path name.
    std::string answer_path_name = "../TEST/PA3-";

    std::string answer_path_name_2 = "-Debug_Output.txt";
    std::string sample_str = "-Debug-Output.txt";
    std::string path_name = "";

    std::string debug_path_name = "../DATA/PA3-";

    // specify the names for each of the files
    std::string body_a_str = "../DATA/Problem3-BodyA.txt";
    std::string body_b_str = "../DATA/Problem3-BodyB.txt";
    std::string mesh_str = "../DATA/Problem3Mesh.sur";

    // read files
    p3_read_mesh(mesh_str, mesh);
    read_post_regis(debug_path_name + idx + sample_str, s_ks);

    // calculate the closest points respect to s.
    std::vector<Matrix> output = matching(mesh, s_ks);

    // output results
    std::string output_filename = answer_path_name + idx + answer_path_name_2;
    std::ofstream out_file(output_filename);
    if (out_file.is_open())
    {
        out_file << (int)s_ks.size() << ", " << output_filename << std::endl;
        for (int frame_num = 0; frame_num < (int)s_ks.size(); ++frame_num)
        {
            std::string output_line = formatLine(s_ks[frame_num], output[frame_num]);
            out_file << output_line << "\n";
        }
        out_file.close();
    }
}

/**
 * helper function to match a set of target point to the 3D mesh
 */
std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks)
{
    // uncomment the block below to perform the simple search
    /* std::vector<Matrix> closest_points;
     for(int i = 0; i < q_ks.size(); i++){
         closest_points.push_back(mesh.find_closest_point(q_ks.at(i)));
     }
 */
    std::vector<Matrix> closest_points = mesh.find_closest_point_advanced(q_ks);
    return closest_points;
}

/**
 * helper function to format the output file
 */
std::string formatLine(Matrix d, Matrix s)
{
    std::string dx = formatNum(d.get_pos(0, 0));
    std::string dy = formatNum(d.get_pos(1, 0));
    std::string dz = formatNum(d.get_pos(2, 0));
    std::string sx = formatNum(s.get_pos(0, 0));
    std::string sy = formatNum(s.get_pos(1, 0));
    std::string sz = formatNum(s.get_pos(2, 0));
    float mg = (d - s).magnitude();
    std::stringstream stream;
    stream << std::fixed << std::setprecision(3) << mg;
    std::string mg_string = stream.str();
    return dx + " " + dy + " " + dz + "  " + sx + " " + sy + " " + sz + "  " + mg_string;
}

/**
 * helper function to format the output file
 */
std::string formatNum(float a)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << a;
    std::string fill = "";
    std::string cur = stream.str();

    for (int i = 0; i < 7 - (int)cur.length(); i++)
    {
        fill += " ";
    }
    return fill + cur;
}