#include "mesh.h"
#include "io_read.h"
#include <iostream>
#include <iomanip>

// std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks, bool advanced);
std::string formatLine(Matrix d, Matrix s);
std::string formatNum(float a);

int main(int argc, char **argv)
{
    std::string idx = "A";
    std::string res = "h";
    if (argc > 1)
    {
        idx = argv[1];
    }
    if (argc > 2)
    {
        res = argv[2];
    }

    // create structures to store data
    std::vector<Matrix> body_a = std::vector<Matrix>();
    std::vector<Matrix> body_b = std::vector<Matrix>();
    std::vector<f_data> sample_reading = std::vector<f_data>();
    std::vector<std::vector<Matrix>> modes;

    std::string answer_path_name_2;
    std::string sample_str;
    std::string data_path_name = "../DATA/PA5-HighRes-";
    std::string answer_path_name = "../OUTPUT/PA5-HighRes-";
    std::string body_a_str = "../DATA/Problem5-HighRes-BodyA.txt";
    std::string body_b_str = "../DATA/Problem5-HighRes-BodyB.txt";
    std::string mesh_str = "../DATA/Problem5-HighRes-MeshFile.sur";
    std::string modes_path = "../DATA/Problem5-HighRes-Modes.txt";
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

    if (res != "h")
    {
        data_path_name = "../DATA/PA5-";
        answer_path_name = "../OUTPUT/PA5-";
        body_a_str = "../DATA/Problem5-BodyA.txt";
        body_b_str = "../DATA/Problem5-BodyB.txt";
        mesh_str = "../DATA/Problem5MeshFile.sur";
        modes_path = "../DATA/Problem5Modes.txt";
    }
    p5_read_modes(modes_path, modes);

    Mesh mesh((int)modes.size() - 1, modes);

    // read files
    Matrix ptr_a = p3_read_body(body_a_str, body_a);
    p3_read_body(body_b_str, body_b);
    p3_read_mesh(mesh_str, mesh);
    p3_read_sample(data_path_name + idx + sample_str, sample_reading, (int)body_a.size(), (int)body_b.size());

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

    auto output = mesh.deformed_find_optimum_transformation(d_ks, 0.95);

    Frame f_estimate = std::get<0>(output);
    std::cout << "frame transformation: " << std::endl;
    f_estimate.get_rot().get_rot().print_str();
    f_estimate.get_pos().get_pos().print_str();
    std::vector<Matrix> s_ks;
    std::vector<Matrix> c_ks;
    auto result = std::get<1>(output);
    for (Matrix m : d_ks)
    {
        s_ks.push_back(f_estimate * m);
    }
    for (std::tuple<TriangleMesh, Matrix> c : result)
    {
        c_ks.push_back(std::get<1>(c));
    }

    // output results
    std::string output_filename = answer_path_name + idx + answer_path_name_2;
    std::ofstream out_file(output_filename);
    if (out_file.is_open())
    {
        out_file << (int)sample_reading.size() << ", " << output_filename << ", " << ((int)modes.size() - 1) << std::endl;
        std::vector<float> lambdas = mesh.get_lambdas();
        for (int i = 0; i < (int)lambdas.size(); i++)
        {
            out_file << lambdas[i] << ", ";
        }
        out_file << std::endl;

        for (int frame_num = 0; frame_num < (int)sample_reading.size(); ++frame_num)
        {
            std::string output_line = formatLine(s_ks[frame_num], c_ks[frame_num]);
            out_file << output_line << "\n";
        }
        out_file.close();
    }

    return 0;
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