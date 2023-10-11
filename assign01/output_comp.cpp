#include "output_comp.h"

int main(int argc, char **argv)
{
    std::string expected_output_file;
    std::string output_file;
    // if pass in two or more command argument, assume that
    if (argc >= 3)
    {
        expected_output_file = argv[1];
        output_file = argv[2];
        std::tuple<float, float> m_and_std = get_m_and_std(expected_output_file, output_file);
        float mean = std::get<0>(m_and_std);
        float stdev = std::get<1>(m_and_std);
        std::cout << "mean: " << mean << std::endl;
        std::cout << "standard deviation: " << stdev << std::endl;
    }
    else if (argc == 2 && strcmp(argv[1], "C") == 0)
    {
        std::vector<std::string> debugs{"a", "b", "c", "d", "e", "f", "g"};
        std::string debug_path_name = "PA1 Student Data/pa1-debug-";
        std::string debug_answer_path_name = "PA1 Student Answer/pa1-debug-";
        std::string debug_path_end = "-output1.txt";
        std::string debug_answer_path_end = "-output.txt";
        std::tuple<float, float> m_and_std;
        std::string output_filename = "PA1 Student Answer/pa1-debug-output-stats-c-expected.txt";
        std::ofstream out_file(output_filename);
        if (out_file.is_open())
        {
            for (int i = 0; i < (int)debugs.size(); ++i)
            {
                m_and_std = get_m_and_std(debug_path_name + debugs[i] + debug_path_end,
                                          debug_answer_path_name + debugs[i] + debug_answer_path_end);
                out_file << "mean for dataset " << std::string(debugs[i]) << ": " << std::get<0>(m_and_std) << "\n";
                out_file << "standard deviation for dataset " << std::string(debugs[i]) << ": " << std::get<1>(m_and_std) << "\n";
            }
            out_file.close();
        }
    }
    else
    {
        std::vector<std::string> debugs{"a", "b", "c", "d", "e", "f", "g"};
        std::string empivot_str = "-empivot.txt";
        std::string optpivot_str = "-optpivot.txt";
        std::string output_str = "-output1.txt";
        std::string calbody_str = "-calbody.txt";
        std::string file_header = "PA1 Student Data/pa1-debug-";

        std::string output_filename = "PA1 Student Answer/pa1-debug-output-stats-pivots.txt";
        std::ofstream out_file(output_filename);
        if (out_file.is_open())
        {
            std::tuple<float, float> em_x_m_and_std;
            std::tuple<float, float> em_y_m_and_std;
            std::tuple<float, float> em_z_m_and_std;
            std::tuple<float, float> opt_x_m_and_std;
            std::tuple<float, float> opt_y_m_and_std;
            std::tuple<float, float> opt_z_m_and_std;
            for (int i = 0; i < (int)debugs.size(); i++)
            {
                // get the expected numbers from output1.txt
                std::ifstream in_expected_file;
                in_expected_file.open(file_header + debugs[i] + output_str);
                std::vector<float> expected_vals;
                char garbage;
                float f1, f2, f3;
                std::string in_line;
                std::getline(in_expected_file, in_line);
                for (int j = 0; j < 2; ++j)
                {
                    std::getline(in_expected_file, in_line);
                    std::stringstream(in_line) >> f1 >> garbage >> f2 >> garbage >> f3;
                    expected_vals.push_back(f1);
                    expected_vals.push_back(f2);
                    expected_vals.push_back(f3);
                }
                in_expected_file.close();
                std::vector<float> em_x_diffs;
                std::vector<float> em_y_diffs;
                std::vector<float> em_z_diffs;
                std::vector<float> opt_x_diffs;
                std::vector<float> opt_y_diffs;
                std::vector<float> opt_z_diffs;

                std::vector<f_data> em_frames;
                std::vector<f_data> opt_frames;
                std::vector<std::vector<float>> data_d;
                std::vector<std::vector<float>> data_a;
                std::vector<std::vector<float>> data_c;

                read_calbody(file_header + debugs[i] + calbody_str, data_d, data_a, data_c);
                read_empivot(file_header + debugs[i] + empivot_str, em_frames);
                read_optpivot(file_header + debugs[i] + optpivot_str, opt_frames);

                for (int epoch = 0; epoch < 100; epoch++)
                {
                    std::tuple<Matrix, Matrix> mats = get_pos(em_frames, opt_frames, data_d);
                    em_x_diffs.push_back(expected_vals.at(0) - std::get<0>(mats).get_pos(3, 0));
                    em_y_diffs.push_back(expected_vals.at(1) - std::get<0>(mats).get_pos(4, 0));
                    em_z_diffs.push_back(expected_vals.at(2) - std::get<0>(mats).get_pos(5, 0));
                    opt_x_diffs.push_back(expected_vals.at(3) - std::get<1>(mats).get_pos(3, 0));
                    opt_y_diffs.push_back(expected_vals.at(4) - std::get<1>(mats).get_pos(4, 0));
                    opt_z_diffs.push_back(expected_vals.at(5) - std::get<1>(mats).get_pos(5, 0));
                }
                em_x_m_and_std = calculate_m_and_std(em_x_diffs);
                em_y_m_and_std = calculate_m_and_std(em_y_diffs);
                em_z_m_and_std = calculate_m_and_std(em_z_diffs);
                opt_x_m_and_std = calculate_m_and_std(opt_x_diffs);
                opt_y_m_and_std = calculate_m_and_std(opt_y_diffs);
                opt_z_m_and_std = calculate_m_and_std(opt_z_diffs);
                out_file << "dataset " << debugs[i] << ": \n";
                out_file << "EM pivot x:  calculated mean " << expected_vals.at(0) + std::get<0>(em_x_m_and_std) << "\n";
                out_file << "             error mean " << std::get<0>(em_x_m_and_std) << "\n";
                out_file << "             error standard deviation " << std::get<1>(em_x_m_and_std) << "\n";
                out_file << "EM pivot y:  calculated mean " << expected_vals.at(1) + std::get<0>(em_y_m_and_std) << "\n";
                out_file << "             error mean " << std::get<0>(em_y_m_and_std) << "\n";
                out_file << "             error standard deviation " << std::get<1>(em_y_m_and_std) << "\n";
                out_file << "EM pivot z:  calculated mean " << expected_vals.at(2) + std::get<0>(em_z_m_and_std) << "\n";
                out_file << "             error mean " << std::get<0>(em_z_m_and_std) << "\n";
                out_file << "             error standard deviation " << std::get<1>(em_z_m_and_std) << "\n";
                out_file << "Opt pivot x: calculated mean " << expected_vals.at(3) + std::get<0>(opt_x_m_and_std) << "\n";
                out_file << "             error mean " << std::get<0>(opt_x_m_and_std) << "\n";
                out_file << "             error standard deviation " << std::get<1>(opt_x_m_and_std) << "\n";
                out_file << "Opt pivot y: calculated mean " << expected_vals.at(4) + std::get<0>(opt_y_m_and_std) << "\n";
                out_file << "             error mean " << std::get<0>(opt_y_m_and_std) << "\n";
                out_file << "             error standard deviation " << std::get<1>(opt_y_m_and_std) << "\n";
                out_file << "Opt pivot z: calculated mean " << expected_vals.at(5) + std::get<0>(opt_z_m_and_std) << "\n";
                out_file << "             error mean " << std::get<0>(opt_z_m_and_std) << "\n";
                out_file << "             error standard deviation " << std::get<1>(opt_z_m_and_std) << "\n";
                out_file << "\n";
            }
            out_file.close();
        }
    }
    return 0;
}

std::tuple<float, float> get_m_and_std(std::string expected_output_file, std::string output_file)
{
    std::vector<float> diff;

    std::ifstream in_expected_file;
    in_expected_file.open(expected_output_file);
    std::string in_line;
    for (int i = 0; i < 3; ++i)
    {
        std::getline(in_expected_file, in_line);
    }
    float f1, f2, f3;
    char garbage;
    while (std::getline(in_expected_file, in_line))
    {
        std::stringstream(in_line) >> f1 >> garbage >> f2 >> garbage >> f3;
        diff.push_back(f1);
        diff.push_back(f2);
        diff.push_back(f3);
    }
    in_expected_file.close();
    std::ifstream in_actual_file;
    in_actual_file.open(output_file);
    for (int i = 0; i < 3; ++i)
    {
        std::getline(in_actual_file, in_line);
    }
    int idx = 0;
    while (std::getline(in_actual_file, in_line))
    {
        std::stringstream(in_line) >> f1 >> garbage >> f2 >> garbage >> f3;
        diff[idx++] -= f1;
        diff[idx++] -= f2;
        diff[idx++] -= f3;
    }
    in_actual_file.close();

    return calculate_m_and_std(diff);
}

std::tuple<float, float> calculate_m_and_std(std::vector<float> diff)
{
    float sum = std::accumulate(diff.begin(), diff.end(), 0.0);
    float mean = sum / diff.size();

    std::vector<float> var(diff.size());
    std::transform(diff.begin(), diff.end(), var.begin(),
                   std::bind2nd(std::minus<float>(), mean));
    float sq_sum = std::inner_product(var.begin(), var.end(), var.begin(), 0.0);
    float stdev = std::sqrt(sq_sum / diff.size());
    return std::make_tuple(mean, stdev);
}

std::tuple<Matrix, Matrix> get_pos(std::vector<f_data> &em_frames,
                                   std::vector<f_data> &opt_frames,
                                   std::vector<std::vector<float>> &data_d)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rng = std::default_random_engine{seed};
    std::shuffle(std::begin(em_frames), std::end(em_frames), rng);
    std::shuffle(std::begin(opt_frames), std::end(opt_frames), rng);

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
    return std::make_tuple(em_p_ts, opt_p_ts);
}