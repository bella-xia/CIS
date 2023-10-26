#include <iostream>
#include "registration.h"
#include "interpolation.h"
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

    std::string answer_path_name;

    // two ems
    std::vector<f_data> fiducial_frames;
    std::vector<f_data> nav_frames;
    std::vector<f_data> em_frames;

    // b file
    std::vector<std::vector<float>> data_b;

    // designate file pathway
    if (idx.compare("other") == 0)
    {
        std::string calbody_string;
        std::string calreading_string;
        std::string em_fi_string;
        std::string em_nav_string;
        std::string ct_fi_string;
        std::cout << "Please enter calbody file name:" << std::endl;
        std::cin >> calbody_string;
        std::cout << "Please enter calreading file name:" << std::endl;
        std::cin >> calreading_string;
        std::cout << "Please enter em fiducial file name:" << std::endl;
        std::cin >> em_fi_string;
        std::cout << "Please enter answer file name:" << std::endl;
        std::cin >> answer_path_name;

        answer_path_name = "../OUTPUT/" + answer_path_name;

        read_calbody(calbody_string, data_d, data_a, data_c);
        read_calreadings(calreading_string, calreadings_frames);

        // read two gs
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
        std::string em_fi_str = "-em-fiducialss.txt";
        std::string em_nav_str = "-EM-nav.txt";
        std::string ct_fi_str = "-ct-fiducials.txt";

        // step 1 : read files
        read_calbody(path_name + idx + calbody_str, data_d, data_a, data_c);
        read_calreadings(path_name + idx + calreadings_str, calreadings_frames);

        // use two G values
        read_empivot(path_name + idx + empivot_str, em_frames);
        read_empivot(path_name + idx + em_fi_str, fiducial_frames);
        read_empivot(path_name + idx + em_nav_str, nav_frames);

        read_ct_fi(path_name + idx + ct_fi_str, data_b);
    }
    // randomize the sequence of the readings so that the origin of the pivot
    // reference frame is randomized.
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // auto rng = std::default_random_engine{seed};
    // std::shuffle(std::begin(em_frames), std::end(em_frames), rng);
    // std::shuffle(std::begin(opt_frames), std::end(opt_frames), rng);

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
    std::vector<std::vector<Matrix>> C_mat_expected;
    std::vector<std::vector<Matrix>> C_mat_real;
    std::vector<Matrix> C_mat_expected_flattened;
    std::vector<Matrix> C_mat_real_flattened;
    std::vector<Matrix> C_mat_real_sub;
    // pass in A and D vectors
    for (int frame_num = 0; frame_num < (int)calreadings_frames.size(); ++frame_num)
    {
        // C_mat_real[frame_num] = std::vector<Matrix>();

        for (int i = 0; i < (int)calreadings_frames[frame_num].data_d.size(); ++i)
        {
            fd_reg.add_matrix_b(Matrix(calreadings_frames[frame_num].data_d[i]));
            fa_reg.add_matrix_b(Matrix(calreadings_frames[frame_num].data_a[i]));
        }
        for (int i = 0; i < (int)calreadings_frames[frame_num].data_c.size(); ++i)
        {
            C_mat_real_sub.push_back(Matrix(calreadings_frames[frame_num].data_c[i]));
        }
        Frame fd = fd_reg.point_cloud_registration();
        Frame fa = fa_reg.point_cloud_registration();
        fd_frames.push_back(fd);
        fa_frames.push_back(fa);
        for (int i = 0; i < (int)c_mat.size(); ++i)
        {
            C_frame.push_back(fd.inverse() * (fa * c_mat[i]));
        }
        C_mat_real.push_back(C_mat_real_sub);
        C_mat_expected.push_back(C_frame);
        C_mat_real_sub.clear();
        C_frame.clear();
        fd_reg.clean_matrix_b();
        fa_reg.clean_matrix_b();
    }

    for (std::vector<Matrix> ele_vec : C_mat_real)
    {
        for (Matrix ele : ele_vec)
        {
            C_mat_real_flattened.push_back(ele);
        }
    }
    for (std::vector<Matrix> ele_vec : C_mat_expected)
    {
        for (Matrix ele : ele_vec)
        {
            C_mat_expected_flattened.push_back(ele);
        }
    }
    Interpolation interpolation(C_mat_real_flattened, C_mat_expected_flattened);
    // Interpolation interpolation(C_mat_real.at(100), C_mat_expected.at(100));
    Matrix coefficients = interpolation.interpolate();
    std::vector<std::tuple<int, int>> samples;
    samples.push_back(std::make_tuple<int, int>(100, 18));
    samples.push_back(std::make_tuple<int, int>(50, 1));
    samples.push_back(std::make_tuple<int, int>(25, 10));
    samples.push_back(std::make_tuple<int, int>(75, 7));

    std::cout << "using all frames: " << std::endl;

    for (std::tuple<int, int> ele : samples)
    {
        int frame = std::get<0>(ele);
        int idx = std::get<1>(ele);
        Matrix sample(C_mat_real.at(frame).at(idx));
        Matrix expected(C_mat_expected.at(frame).at(idx));
        Matrix calc_result = interpolation.correction_func(Matrix((C_mat_real.at(frame).at(idx))));
        std::cout << "Frame " << frame << " offset " << idx << ": " << std::endl;
        std::cout << "Real: " << std::endl;
        sample.print_str();
        std::cout << "Expected: " << std::endl;
        expected.print_str();
        std::cout << "Calculated: " << std::endl;
        calc_result.print_str();
    }

    // get Gis from EM pivot
    Registration fg_reg = Registration();
    std::vector<Frame> fg_frames = std::vector<Frame>();
    // pass in G vectors, find f_g frames
    for (int frame_num = (int)em_frames.size() - 1; frame_num >= 0; --frame_num)
    {
        for (int i = 0; i < (int)em_frames[frame_num].data_g.size(); ++i)
        {
            fg_reg.add_matrix_b(interpolation.correction_func(
                                                 Matrix(em_frames[frame_num].data_g[i]))
                                    .transpose());
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

    Matrix b_tip(em_p_ts.get_pos(0, 0), em_p_ts.get_pos(1, 0), em_p_ts.get_pos(2, 0));
    Matrix b_post(em_p_ts.get_pos(3, 0), em_p_ts.get_pos(4, 0), em_p_ts.get_pos(5, 0));

    std::vector<Matrix> B_vec;

    for (int frame_num = 0; frame_num < (int)fiducial_frames.size(); ++frame_num)
    {
        for (int i = 0; i < (int)fiducial_frames[frame_num].data_g.size(); ++i)
        {
            fg_reg.add_matrix_b(interpolation.correction_func(
                                                 Matrix(fiducial_frames[frame_num].data_g[i]))
                                    .transpose());
        }
        Frame fg = fg_reg.point_cloud_registration();
        B_vec.push_back(fg * b_tip);
        fg_reg.clean_matrix_b();
    }

    Registration fb_reg = Registration();
    for (int i = 0; i < (int)B_vec.size(); i++)
    {
        fb_reg.add_matrix_a(B_vec[i]);
        fb_reg.add_matrix_b(Matrix(data_b[i]));
    }
    Frame fb = fb_reg.point_cloud_registration();

    std::vector<Matrix> nav_b;

    for (int frame_num = 0; frame_num < (int)nav_frames.size(); ++frame_num)
    {
        for (int i = 0; i < (int)nav_frames[frame_num].data_g.size(); ++i)
        {
            fg_reg.add_matrix_b(interpolation.correction_func(
                                                 Matrix(nav_frames[frame_num].data_g[i]))
                                    .transpose());
        }
        Frame fg = fg_reg.point_cloud_registration();
        Matrix B(fg * b_tip);
        nav_b.push_back(fb * B);
        fg_reg.clean_matrix_b();
    }

    for (Matrix ele : nav_b)
    {
        ele.print_str();
        std::cout << std::endl;
    }

    return 0;
}
