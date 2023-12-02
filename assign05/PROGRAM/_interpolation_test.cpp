#include <iostream>
#include "registration.h"
#include "interpolation.h"
#include "io_read.h"
#include <algorithm>
#include <random>
#include <chrono>

int main(int argc, char **argv)
{
    srand(time(0));
    std::vector<char> file_idx = {'a', 'b', 'c', 'd', 'e', 'f'};
    char idx = file_idx[rand() % 6];
    std::cout << "Using file " << idx << ": " << std::endl;
    // preparing data storage
    // calbody
    std::vector<std::vector<float>> data_d;
    std::vector<std::vector<float>> data_a;
    std::vector<std::vector<float>> data_c;
    std::vector<Matrix> c_mat;
    // calreading
    std::vector<f_data> calreadings_frames;

    // designate file pathway
    // initialize all file name components
    std::string path_name = "../DATA/pa2-debug-";
    std::string answer_path_name = "../OUTPUT/pa2-debug-";

    std::string calbody_str = "-calbody.txt";
    std::string calreadings_str = "-calreadings.txt";

    // step 1 : read files
    read_calbody(path_name + idx + calbody_str, data_d, data_a, data_c);
    read_calreadings(path_name + idx + calreadings_str, calreadings_frames);

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
    std::vector<Matrix> C_mat_expected_flattened;
    std::vector<Matrix> C_mat_real_flattened;
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
            C_mat_real_flattened.push_back(Matrix(calreadings_frames[frame_num].data_c[i]));
        }

        // perform point cloud registration on a and d respectively, and collect the frame estimates
        Frame fd = fd_reg.point_cloud_registration();
        Frame fa = fa_reg.point_cloud_registration();
        fd_frames.push_back(fd);
        fa_frames.push_back(fa);
        // create the C_expected point based on estimated fa and fd frame transformation
        for (int i = 0; i < (int)c_mat.size(); ++i)
        {
            C_mat_expected_flattened.push_back(fd.inverse() * (fa * c_mat[i]));
        }
        fd_reg.clean_matrix_b();
        fa_reg.clean_matrix_b();
    }
    // create an interpolation function between the real C points and the expected C points
    Interpolation interpolation(C_mat_real_flattened, C_mat_expected_flattened);
    Matrix coefficients = interpolation.interpolate();

    // randomly sample a number of points in the real C sets
    std::vector<std::tuple<int, int>> samples;
    for (int i = 0; i < 5; ++i)
    {
        samples.push_back(std::make_tuple<int, int>(rand() % 100, rand() % 20));
    }

    int frame_size = calreadings_frames[0].data_c.size();

    // compare the result of expected C and real C after interpolation,
    // they should be the same

    for (std::tuple<int, int> ele : samples)
    {
        int frame = std::get<0>(ele);
        int frame_idx = std::get<1>(ele);
        Matrix sample(C_mat_real_flattened.at(frame * frame_size + frame_idx));
        Matrix expected(C_mat_expected_flattened.at(frame * frame_size + frame_idx));
        Matrix calc_result = interpolation.correction_func(Matrix((C_mat_real_flattened.at(frame * frame_size + frame_idx))));
        std::cout << "Frame " << frame << " offset " << frame_idx << ": " << std::endl;
        std::cout << "Real: " << std::endl;
        sample.transpose().print_str();
        std::cout << "Expected: " << std::endl;
        expected.transpose().print_str();
        std::cout << "Calculated: " << std::endl;
        calc_result.print_str();
    }
    return 0;
}