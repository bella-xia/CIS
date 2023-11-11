#include "mesh.h"
#include "registration.h"
#include "io_read.h"
#include <iostream>

std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks);

int main(int argc, char **argv)
{

    std::string idx = "A";
    if (argc > 1)
    {
        idx = argv[1];
    }

    std::vector<Matrix> body_a = std::vector<Matrix>();
    std::vector<Matrix> body_b = std::vector<Matrix>();
    std::vector<f_data> sample_reading = std::vector<f_data>();

    Mesh mesh;

    std::string answer_path_name;
    std::string path_name = "";
    std::string debug_path_name = "../DATA/PA3-";

    // specify the names for each of the files
    std::string sample_str = "-Debug-SampleReadingsTest.txt";
    std::string body_a_str = "../DATA/Problem3-BodyA.txt";
    std::string body_b_str = "../DATA/Problem3-BodyB.txt";
    // std::string mesh_str = "../DATA/Problem3-Mesh.sur";
    std::string mesh_str = "../DATA/Problem3Mesh.sur";

    Matrix ptr_a = p3_read_body(body_a_str, body_a);
    p3_read_body(body_b_str, body_b);
    p3_read_mesh(mesh_str, mesh);
    p3_read_sample(debug_path_name + idx + sample_str, sample_reading, (int)body_a.size(), (int)body_b.size());

    Registration fa_reg = Registration();
    Registration fb_reg = Registration();
    std::vector<Matrix> d_ks = std::vector<Matrix>();
    std::vector<Frame> frames_aks = std::vector<Frame>();
    std::vector<Frame> frames_bks = std::vector<Frame>();

    // pass in a and d and c vectors
    for (int i = 0; i < (int)body_a.size(); ++i)
    {
        fa_reg.add_matrix_a(body_a[i]);
    }
    for (int i = 0; i < (int)body_b.size(); ++i)
    {
        fb_reg.add_matrix_a(body_b[i]);
    }
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
        d_ks.push_back(fb.inverse() * fa * ptr_a);
        fa_reg.clean_matrix_b();
        fb_reg.clean_matrix_b();
    }

    Frame reg = Frame();

    std::vector<Matrix> s_ks = std::vector<Matrix>();
    for (Matrix ele : d_ks)
    {
        s_ks.push_back(reg * ele);
    }

    std::vector<Matrix> output = matching(mesh, s_ks);
    for (Matrix ele : s_ks)
    {
        ele.print_str();
    }

    // step 1 : read files
}

std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks)
{

    std::vector<Matrix> closest_points = mesh.find_closest_point_advanced(q_ks);
    return closest_points;
}