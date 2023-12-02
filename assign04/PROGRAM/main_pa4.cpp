#include "mesh.h"
#include "io_read.h"
#include <iostream>
#include <iomanip>

std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks, bool advanced);
std::string formatLine(Matrix d, Matrix s);
std::string formatNum(float a);

int main(int argc, char **argv)
{

    std::string idx = argv[1];
    if (argc > 1)
    {
        idx = argv[1];
    }

    // create structures to store data
    std::vector<Matrix> body_a = std::vector<Matrix>();
    std::vector<Matrix> body_b = std::vector<Matrix>();
    std::vector<f_data> sample_reading = std::vector<f_data>();
    Mesh mesh;

    // define the path name.
    std::string answer_path_name = "../OUTPUT/PA5-";
    std::string answer_path_name_2;
    std::string sample_str;
    if (idx < "G")
    {
        answer_path_name_2 = "-Debug_Output.txt";
        sample_str = "-Debug-SampleReadingsTest.txt";
    }
    else
    {
        answer_path_name_2 = "-Unknown_Output.txt";
        sample_str = "-Unknown-SampleReadingsTest.txt";
    }
    std::string path_name = "";
    std::string debug_path_name = "../DATA/PA5-";

    // specify the names for each of the files
    std::string body_a_str = "../DATA/Problem5-BodyA.txt";
    std::string body_b_str = "../DATA/Problem5-BodyB.txt";
    std::string mesh_str = "../DATA/Problem5MeshFile.sur";

    // read files
    Matrix ptr_a = p3_read_body(body_a_str, body_a);
    p3_read_body(body_b_str, body_b);
    p3_read_mesh(mesh_str, mesh);
    p3_read_sample(debug_path_name + idx + sample_str, sample_reading, (int)body_a.size(), (int)body_b.size());

    // registration to calculate F_A
    Registration fa_reg = Registration();
    // registration to calculate F_B
    Registration fb_reg = Registration();
    // vectors to store dk
    std::vector<Matrix> d_ks = std::vector<Matrix>();
    // F_A in each frame
    std::vector<Frame> frames_aks = std::vector<Frame>();
    // F_B in each frame
    std::vector<Frame> frames_bks = std::vector<Frame>();

    // pass in a and b vectors in body frame
    for (int i = 0; i < (int)body_a.size(); ++i)
    {
        fa_reg.add_matrix_a(body_a[i]);
    }
    for (int i = 0; i < (int)body_b.size(); ++i)
    {
        fb_reg.add_matrix_a(body_b[i]);
    }
    // perform point cloud registration to find the Fa and Fb
    for (int frame_num = 0; frame_num < (int)sample_reading.size(); ++frame_num)
    {

        for (int i = 0; i < (int)sample_reading[frame_num].data_a.size(); ++i)
        {
            // put all the A and D vectors into their corresponding position
            // because the least square equation takes the form F_a * a = A, F_d * b = D,
            // so A and D would be in matrix b
            fa_reg.add_matrix_b(Matrix(sample_reading[frame_num].data_a[i]));
            fb_reg.add_matrix_b(Matrix(sample_reading[frame_num].data_b[i]));
        }
        Frame fa = fa_reg.point_cloud_registration();
        Frame fb = fb_reg.point_cloud_registration();
        frames_aks.push_back(fa);
        frames_bks.push_back(fb);

        // calculate the corresponding d and store them.
        d_ks.push_back(fb.inverse() * (fa * ptr_a));

        fa_reg.clean_matrix_b();
        fb_reg.clean_matrix_b();
    }

    // Freg = I
    Frame reg = Frame();

    // calculate s = Freg * d
    // std::vector<Matrix> s_ks = std::vector<Matrix>();
    // for (Matrix ele : d_ks)
    // {
    //     s_ks.push_back(reg * ele);
    // }

    // calculate the closest points respect to s.
    time_t start, end;
    start = clock();
    std::ios_base::sync_with_stdio(false);
    std::tuple<Frame, std::vector<Matrix>> output = mesh.find_optimum_transformation(d_ks, 0.95);
    end = clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by program is : " << std::fixed
              << time_taken << std::setprecision(5);
    std::cout << " sec " << std::endl;
    Frame estimate = std::get<0>(output);
    std::vector<Matrix> c_ks = std::get<1>(output);
    std::vector<Matrix> s_ks;
    for (Matrix m : d_ks)
    {
        s_ks.push_back(estimate * m);
    }
    // output results
    std::string output_filename = answer_path_name + idx + answer_path_name_2;
    std::ofstream out_file(output_filename);
    if (out_file.is_open())
    {
        out_file << (int)sample_reading.size() << ", " << output_filename << std::endl;
        for (int frame_num = 0; frame_num < (int)sample_reading.size(); ++frame_num)
        {

            std::string output_line = formatLine(s_ks[frame_num], c_ks[frame_num]);
            out_file << output_line << "\n";
        }
        out_file.close();
    }
}

/**
 * helper function to find the set of closest points to the target points
 */
std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks, bool advanced)
{
    if (!advanced)
    {
        std::vector<Matrix> closest_points;
        for (int i = 0; i < (int)q_ks.size(); i++)
        {
            closest_points.push_back(mesh.find_closest_point(q_ks.at(i)));
        }
        return closest_points;
    }
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