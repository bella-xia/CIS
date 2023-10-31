#include <iostream>
#include "registration.h"
#include "io_read.h"
#include <algorithm>
#include <random>
#include <chrono>

int main(int argc, char **argv)
{
#
    std::string idx = "";
    if (argc > 1)
    {
        idx = argv[1];
    }
    // preparing data storage
    // calbody
    std::vector<std::vector<float>> data_d;
    std::vector<std::vector<float>> data_a;
    std::vector<std::vector<float>> data_c;
    std::vector<Matrix> c_mat;
    // calreading
    std::vector<f_data> calreadings_frames;
    // EM pivot calibration
    std::vector<f_data> em_frames;
    // optical pivot calibration
    std::vector<f_data> opt_frames;
    std::string answer_path_name;
    // designate file pathway
    if (idx.compare("other") == 0)
    {
        std::string calbody_string;
        std::string calreading_string;
        std::string empivot_string;
        std::string optpivot_string;
        std::cout << "Please enter calbody file name:" << std::endl;
        std::cin >> calbody_string;
        std::cout << "Please enter calreading file name:" << std::endl;
        std::cin >> calreading_string;
        std::cout << "Please enter empivot file name:" << std::endl;
        std::cin >> empivot_string;
        std::cout << "Please enter optpivot file name:" << std::endl;
        std::cin >> optpivot_string;
        std::cout << "Please enter answer file name:" << std::endl;
        std::cin >> answer_path_name;

        answer_path_name = "../OUTPUT/" + answer_path_name;

        read_calbody(calbody_string, data_d, data_a, data_c);
        read_calreadings(calreading_string, calreadings_frames);
        read_empivot(empivot_string, em_frames);
        read_optpivot(optpivot_string, opt_frames);
    }
    else
    {
        // initialize all file name components
        std::string path_name = "";
        std::string debug_path_name = "../DATA/pa2-debug-";
        std::string debug_answer_path_name = "../OUTPUT/pa2-debug-";
        std::string unknown_answer_path_name = "../OUTPUT/pa2-unknown-";
        std::string unknown_path_name = "../DATA/pa2-unknown-";
        if (idx.compare("h") < 0 && !(idx.compare("a") < 0))
        {
            path_name = debug_path_name;
            answer_path_name = debug_answer_path_name;
        }
        else if (idx.compare("g") > 0 && idx.compare("l") < 0)
        {
            path_name = unknown_path_name;
            answer_path_name = unknown_answer_path_name;
        }
        else
        {
            std::cout << "invalid file name." << std::endl;
            return 1;
        }
        std::string calbody_str = "-calbody.txt";
        std::string calreadings_str = "-calreadings.txt";
        std::string empivot_str = "-empivot.txt";
        std::string optpivot_str = "-optpivot.txt";

        // step 1 : read files
        read_calbody(path_name + idx + calbody_str, data_d, data_a, data_c);
        read_calreadings(path_name + idx + calreadings_str, calreadings_frames);
        read_empivot(path_name + idx + empivot_str, em_frames);
        read_optpivot(path_name + idx + optpivot_str, opt_frames);
    }
    // randomize the sequence of the readings so that the origin of the pivot
    // reference frame is randomized.
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rng = std::default_random_engine{seed};
    std::shuffle(std::begin(em_frames), std::end(em_frames), rng);
    std::shuffle(std::begin(opt_frames), std::end(opt_frames), rng);

    // create F_A and F_D registration
    Registration fa_reg = Registration();
    Registration fd_reg = Registration();
    std::vector<Matrix> C_frame = std::vector<Matrix>();

    // pass in a and d and c vectors
    for (int i = 0; i < (int)data_d.size(); ++i)
    {
        fd_reg.add_matrix_a(Matrix(data_d[i]));
    }
    for (int i = 0; i < (int)data_a.size(); ++i)
    {
        fa_reg.add_matrix_a(Matrix(data_a[i]));
    }
    for (int i = 0; i < (int)data_c.size(); ++i)
    {
        c_mat.push_back(Matrix(data_c[i]));
    }

    std::vector<Frame> fa_frames;
    std::vector<Frame> fd_frames;
    std::vector<std::vector<Matrix>> C_mat;
    // pass in A and D vectors
    for (int frame_num = 0; frame_num < (int)calreadings_frames.size(); ++frame_num)
    {

        for (int i = 0; i < (int)calreadings_frames[frame_num].data_d.size(); ++i)
        {
            fd_reg.add_matrix_b(Matrix(calreadings_frames[frame_num].data_d[i]));
            fa_reg.add_matrix_b(Matrix(calreadings_frames[frame_num].data_a[i]));
        }
        Frame fd = fd_reg.point_cloud_registration();
        Frame fa = fa_reg.point_cloud_registration();
        fd_frames.push_back(fd);
        fa_frames.push_back(fa);
        for (int i = 0; i < (int)c_mat.size(); ++i)
        {
            C_frame.push_back(fd.inverse() * (fa * c_mat[i]));
        }
        C_mat.push_back(C_frame);
        C_frame.clear();
        fd_reg.clean_matrix_b();
        fa_reg.clean_matrix_b();
    }

    // get Gis from EM pivot
    Registration fg_reg = Registration();
    std::vector<Frame> fg_frames = std::vector<Frame>();
    // pass in G vectors, find f_g frames
    for (int frame_num = (int)em_frames.size() - 1; frame_num >= 0; --frame_num)
    {
        for (int i = 0; i < (int)em_frames[frame_num].data_g.size(); ++i)
        {
            fg_reg.add_matrix_b(Matrix(em_frames[frame_num].data_g[i]));
        }
        if (frame_num == (int)em_frames.size() - 1)
        {
            fg_reg.get_matrix_a_from_b();
        }
        Frame fg = fg_reg.point_cloud_registration();
        fg_frames.push_back(fg);
        fg_reg.clean_matrix_b();
    }
    Matrix em_p_ts = fg_reg.pivot_calibration(fg_frames);

    // calibrate optical pivot
    Registration fd_reg_2 = Registration();
    Registration fh_reg = Registration();
    std::vector<Frame> fh_frames = std::vector<Frame>();
    for (int i = 0; i < (int)data_d.size(); ++i)
    {
        fd_reg_2.add_matrix_a(Matrix(data_d[i]));
    }

    for (int frame_num = (int)opt_frames.size() - 1; frame_num >= 0; --frame_num)
    {
        // calculate F_d
        for (int i = 0; i < (int)opt_frames[frame_num].data_d.size(); i++)
        {
            fd_reg_2.add_matrix_b(Matrix(opt_frames[frame_num].data_d[i]));
        }
        Frame fd_2 = fd_reg_2.point_cloud_registration();
        for (int i = 0; i < (int)opt_frames[frame_num].data_h.size(); i++)
        {
            fh_reg.add_matrix_b(Matrix(opt_frames[frame_num].data_h[i]));
        }
        if (frame_num == (int)opt_frames.size() - 1)
        {
            fh_reg.get_matrix_a_from_b();
        }
        Frame fh = fh_reg.point_cloud_registration();
        fh_frames.push_back(fd_2.inverse() * fh);
        fh_reg.clean_matrix_b();
        fd_reg_2.clean_matrix_b();
    }
    Matrix opt_p_ts = fh_reg.pivot_calibration(fh_frames);
    std::string output_filename = answer_path_name + idx + "-output1.txt";
    std::ofstream out_file(output_filename);
    if (out_file.is_open())
    {
        out_file << (int)C_mat[0].size() << ", " << C_mat.size() << ", " << output_filename << std::endl;
        out_file << em_p_ts.get_pos(3, 0) << ", " << em_p_ts.get_pos(4, 0) << ", " << em_p_ts.get_pos(5, 0) << "\n";
        out_file << opt_p_ts.get_pos(3, 0) << ", " << opt_p_ts.get_pos(4, 0) << ", " << opt_p_ts.get_pos(5, 0) << "\n";
        for (int i = 0; i < (int)C_mat.size(); ++i)
        {
            for (int j = 0; j < (int)C_mat[i].size(); ++j)
            {
                out_file << C_mat[i][j].get_pos(0, 0) << ", " << C_mat[i][j].get_pos(1, 0) << ", " << C_mat[i][j].get_pos(2, 0) << "\n";
            }
        }
        out_file.close();
    }

    return 0;
}
