#include <iostream>
#include "registration.h"
#include "io_read.h"

int main(int argc, char **argv)
{

    std::string idx = "";
    if (argc > 1)
    {
        idx = argv[1];
    }
    // initialize all file name components
    std::string path_name = "";
    std::string answer_path_name = "";
    std::vector<std::string> debugs{"a", "b", "c", "d", "e", "f", "g"};
    std::string debug_path_name = "PA1 Student Data/pa1-debug-";
    std::string debug_answer_path_name = "PA1 Student Answer/pa1-debug-";
    std::string unknown_answer_path_name = "PA1 Student Answer/pa1-unknown-";
    std::string unknown_path_name = "PA1 Student Data/pa1-unknown-";
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

    // calbody
    std::vector<std::vector<float>> data_d;
    std::vector<std::vector<float>> data_a;
    std::vector<std::vector<float>> data_c;
    std::vector<Matrix> c_mat;
    read_calbody(path_name + idx + calbody_str, data_d, data_a, data_c);

    // calreadings
    std::vector<f_data> calreadings_frames;
    read_calreadings(path_name + idx + calreadings_str, calreadings_frames);

    std::vector<Frame> fa_frames;
    std::vector<Frame> fd_frames;
    std::vector<std::vector<Matrix>> C_mat;

    // empivot
    std::vector<f_data> em_frames;
    read_empivot(path_name + idx + empivot_str, em_frames);

    // optpivot
    std::vector<f_data> opt_frames;
    read_optpivot(path_name + idx + optpivot_str, opt_frames);

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
    for (int frame_num = 0; frame_num < (int)em_frames.size(); ++frame_num)
    {
        for (int i = 0; i < (int)em_frames[frame_num].data_g.size(); ++i)
        {
            fg_reg.add_matrix_b(Matrix(em_frames[frame_num].data_g[i]));
        }
        if (frame_num == 0)
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

    for (int frame_num = 0; frame_num < (int)opt_frames.size(); ++frame_num)
    {
        // calculate F_d
        for (int i = 0; i < opt_frames[frame_num].data_d.size(); i++)
        {
            fd_reg_2.add_matrix_b(Matrix(opt_frames[frame_num].data_d[i]));
        }
        Frame fd_2 = fd_reg_2.point_cloud_registration();
        for (int i = 0; i < opt_frames[frame_num].data_h.size(); i++)
        {
            fh_reg.add_matrix_b(Matrix(fd_2.inverse() * opt_frames[frame_num].data_h[i]));
        }
        if (frame_num == 0)
        {
            fh_reg.get_matrix_a_from_b();
        }
        Frame fh = fh_reg.point_cloud_registration();
        fh_frames.push_back(fh);
        fh_reg.clean_matrix_b();
        fd_reg_2.clean_matrix_b();
    }
    Matrix opt_p_ts = fh_reg.pivot_calibration(fh_frames);
    std::string output_filename = answer_path_name + idx + "-output.txt";
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