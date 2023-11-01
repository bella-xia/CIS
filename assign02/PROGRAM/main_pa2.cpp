#include <iostream>
#include "registration.h"
#include "interpolation.h"
#include "io_read.h"
#include <algorithm>
#include <random>
#include <chrono>

int main(int argc, char **argv)
{
    // first read in the terminal command - whether a file is specified
    std::string idx = "";
    if (argc > 1)
    {
        idx = argv[1];
    }
    // preparing data storage
    // calbody - stores data for d, a, and cs respectively
    std::vector<std::vector<float>> data_d;
    std::vector<std::vector<float>> data_a;
    std::vector<std::vector<float>> data_c;
    std::vector<Matrix> c_mat;

    // calreading
    std::vector<f_data> calreadings_frames;

    // two ems
    std::vector<f_data> fiducial_frames;
    std::vector<f_data> nav_frames;
    std::vector<f_data> em_frames;

    // b file
    std::vector<std::vector<float>> data_b;

    std::string answer_path_name;

    // if the user specifies "other",
    // then the program asks for the full name of each of the
    // designate file pathway
    if (idx.compare("other") == 0)
    {
        std::string calbody_string;
        std::string calreading_string;
        std::string em_pivot_string;
        std::string em_fi_string;
        std::string em_nav_string;
        std::string ct_fi_string;

        std::cout << "Please enter calbody file name:" << std::endl;
        std::cin >> calbody_string;
        std::cout << "Please enter calreading file name:" << std::endl;
        std::cin >> calreading_string;
        std::cout << "Please enter em pivot file name:" << std::endl;
        std::cin >> em_pivot_string;
        std::cout << "Please enter em fiducial file name:" << std::endl;
        std::cin >> em_fi_string;
        std::cout << "Please enter ct fiducial file name:" << std::endl;
        std::cin >> ct_fi_string;
        std::cout << "Plase endter em navigation file name:" << std::endl;
        std::cin >> em_nav_string;
        std::cout << "Please enter answer file name:" << std::endl;
        std::cin >> answer_path_name;

        answer_path_name = "../OUTPUT/" + answer_path_name;

        read_calbody(calbody_string, data_d, data_a, data_c);
        read_calreadings(calreading_string, calreadings_frames);

        // read three G values
        read_empivot(em_pivot_string, em_frames);
        read_empivot(em_fi_string, fiducial_frames);
        read_empivot(em_nav_string, nav_frames);

        read_ct_fi(ct_fi_string, data_b);
    }
    // otherwise, the input should be one of the letters
    else
    {
        // initialize all file name components
        std::string path_name = "";
        std::string debug_path_name = "../DATA/pa2-debug-";
        std::string debug_answer_path_name = "../OUTPUT/pa2-debug-";
        std::string unknown_answer_path_name = "../OUTPUT/pa2-unknown-";
        std::string unknown_path_name = "../DATA/pa2-unknown-";
        if (idx.compare("g") < 0 && !(idx.compare("a") < 0))
        {
            path_name = debug_path_name;
            answer_path_name = debug_answer_path_name;
        }
        else if (idx.compare("g") >= 0 && idx.compare("k") < 0)
        {
            path_name = unknown_path_name;
            answer_path_name = unknown_answer_path_name;
        }
        // if the input is neither of them, then
        // throw an error
        else
        {
            std::cout << "invalid file name." << std::endl;
            return 1;
        }
        // specify the names for each of the files
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

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rng = std::default_random_engine{seed};
    std::shuffle(std::begin(em_frames), std::end(em_frames), rng);

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

    // initialize all the data structures needed for point-cload registration
    std::vector<Frame> fa_frames;
    std::vector<Frame> fd_frames;

    // create both expected and real C vector arrays
    // since we need to stack all the frames together,
    // so the C_mat would be a 1d array with size (frame_num * points_per_frame,)
    std::vector<Matrix> C_mat_expected_flattened;
    std::vector<Matrix> C_mat_real_flattened;

    // pass in A and D vectors
    for (int frame_num = 0; frame_num < (int)calreadings_frames.size(); ++frame_num)
    {

        for (int i = 0; i < (int)calreadings_frames[frame_num].data_d.size(); ++i)
        {
            // put all the A and D vectors into their corresponding position
            // because the least square equation takes the form F_a * a = A, F_d * b = D,
            // so A and D would be in matrix b
            fd_reg.add_matrix_b(Matrix(calreadings_frames[frame_num].data_d[i]));
            fa_reg.add_matrix_b(Matrix(calreadings_frames[frame_num].data_a[i]));
        }
        for (int i = 0; i < (int)calreadings_frames[frame_num].data_c.size(); ++i)
        {
            C_mat_real_flattened.push_back(Matrix(calreadings_frames[frame_num].data_c[i]));
        }
        Frame fd = fd_reg.point_cloud_registration();
        Frame fa = fa_reg.point_cloud_registration();
        fd_frames.push_back(fd);
        fa_frames.push_back(fa);
        for (int i = 0; i < (int)c_mat.size(); ++i)
        {
            C_mat_expected_flattened.push_back(fd.inverse() * (fa * c_mat[i]));
        }
        fd_reg.clean_matrix_b();
        fa_reg.clean_matrix_b();
    }

    // pass in real and expected values of C into interpolation constructor
    Interpolation interpolation(C_mat_real_flattened, C_mat_expected_flattened);

    // interpolate to get the correction function
    interpolation.interpolate();

    // create a vector to store final results
    std::vector<Matrix> total_mats;

    // pivot calibration with updated vectors
    Registration fg_reg = Registration();
    std::vector<Frame> fg_frames = std::vector<Frame>();
    // pass in G vectors, find f_g frames
    for (int frame_num = 0; frame_num < (int)em_frames.size(); frame_num++)
    {
        for (int i = 0; i < (int)em_frames[frame_num].data_g.size(); ++i)
        {
            // for every frame, pass every vector into correction function,
            // then put the output into matrix b for point cloud registration
            fg_reg.add_matrix_b(interpolation.correction_func(
                                                 Matrix(em_frames[frame_num].data_g[i]))
                                    .transpose());
        }
        // if this is the first frame, calculate the g references
        if (frame_num == 0)
        {
            fg_reg.get_matrix_a_from_b();
        }
        // do point cloud registration for each frame, push the frame transformation
        // into a vector, and then clean all the matrix b
        // since we will use the same local reference axis, so matrix a is never cleaned
        Frame fg = fg_reg.point_cloud_registration();
        fg_frames.push_back(fg);
        fg_reg.clean_matrix_b();
    }

    // pass in all the frame transformation, and do pivot calibration
    Matrix em_p_ts = fg_reg.pivot_calibration(fg_frames);

    // extract from the result
    Matrix b_tip(em_p_ts.get_pos(0, 0), em_p_ts.get_pos(1, 0), em_p_ts.get_pos(2, 0));
    std::cout << "b tip: " << std::endl;
    b_tip.print_str();

    // calculate all the B vectors based on the current frame's point-cloud registration
    // and the calculate tip vector from local reference axis
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

    // pair the calculated B vectors and the bs from CT fiducial
    // then perform point-cloud registration on the EM frame and CT frame
    Registration fb_reg = Registration();
    for (int i = 0; i < (int)B_vec.size(); i++)
    {
        fb_reg.add_matrix_a(B_vec[i]);
        fb_reg.add_matrix_b(Matrix(data_b[i]));
    }
    Frame fb = fb_reg.point_cloud_registration();

    // for each navigation frame G value, use point-cloud registration
    // with respect to the local frame to obtain the vector from EM to probe tip
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
        // then use the previously calculated frame transformation
        // from EM frame to CT frame to obtain b vectors
        total_mats.push_back(fb * B);
        fg_reg.clean_matrix_b();
    }

    // version if we would randomly shuffle the em_frames and
    // iterate for 100 times
    // // create a vector of vector of matrix to store the results
    // std::vector<std::vector<Matrix>> results;
    // for (int iter = 0; iter < 100; ++iter)
    // {
    //     unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //     auto rng = std::default_random_engine{seed};
    //     std::shuffle(std::begin(em_frames), std::end(em_frames), rng);
    //     // get Gis from EM pivot
    //     Registration fg_reg = Registration();
    //     std::vector<Frame> fg_frames = std::vector<Frame>();
    //     // pass in G vectors, find f_g frames
    //     for (int frame_num = (int)em_frames.size() - 1; frame_num >= 0; --frame_num)
    //     {
    //         for (int i = 0; i < (int)em_frames[frame_num].data_g.size(); ++i)
    //         {
    //             fg_reg.add_matrix_b(interpolation.correction_func(
    //                                                  Matrix(em_frames[frame_num].data_g[i]))
    //                                     .transpose());
    //         }
    //         if (frame_num == (int)em_frames.size() - 1)
    //         {
    //             fg_reg.get_matrix_a_from_b();
    //         }
    //         Frame fg = fg_reg.point_cloud_registration();
    //         fg_frames.push_back(fg);
    //         fg_reg.clean_matrix_b();
    //     }
    //     Matrix em_p_ts = fg_reg.pivot_calibration(fg_frames);

    //     Matrix b_tip(em_p_ts.get_pos(0, 0), em_p_ts.get_pos(1, 0), em_p_ts.get_pos(2, 0));
    //     Matrix b_post(em_p_ts.get_pos(3, 0), em_p_ts.get_pos(4, 0), em_p_ts.get_pos(5, 0));

    //     std::vector<Matrix> B_vec;

    //     for (int frame_num = 0; frame_num < (int)fiducial_frames.size(); ++frame_num)
    //     {
    //         for (int i = 0; i < (int)fiducial_frames[frame_num].data_g.size(); ++i)
    //         {
    //             fg_reg.add_matrix_b(interpolation.correction_func(
    //                                                  Matrix(fiducial_frames[frame_num].data_g[i]))
    //                                     .transpose());
    //         }
    //         Frame fg = fg_reg.point_cloud_registration();
    //         B_vec.push_back(fg * b_tip);
    //         fg_reg.clean_matrix_b();
    //     }

    //     Registration fb_reg = Registration();
    //     for (int i = 0; i < (int)B_vec.size(); i++)
    //     {
    //         fb_reg.add_matrix_a(B_vec[i]);
    //         fb_reg.add_matrix_b(Matrix(data_b[i]));
    //     }
    //     Frame fb = fb_reg.point_cloud_registration();

    //     std::vector<Matrix> nav_b;

    //     for (int frame_num = 0; frame_num < (int)nav_frames.size(); ++frame_num)
    //     {
    //         for (int i = 0; i < (int)nav_frames[frame_num].data_g.size(); ++i)
    //         {
    //             fg_reg.add_matrix_b(interpolation.correction_func(
    //                                                  Matrix(nav_frames[frame_num].data_g[i]))
    //                                     .transpose());
    //         }
    //         Frame fg = fg_reg.point_cloud_registration();
    //         Matrix B(fg * b_tip);
    //         nav_b.push_back(fb * B);
    //         fg_reg.clean_matrix_b();
    //     }

    //     results.push_back(nav_b);
    // }
    // std::vector<Matrix> total_mats;
    // int index;
    // int iterate;
    // iterate = 0;
    // for (std::vector<Matrix> elements : results)
    // {
    //     index = 0;
    //     for (Matrix ele : elements)
    //     {
    //         if (iterate == 0)
    //         {
    //             total_mats.push_back(ele);
    //         }
    //         else
    //         {
    //             total_mats.at(index) = total_mats.at(index) + ele;
    //             if (iterate == (int)results.size() - 1)
    //             {
    //                 total_mats.at(index) = Matrix(total_mats.at(index).get_pos(0, 0) / (float)results.size(),
    //                                               total_mats.at(index).get_pos(1, 0) / (float)results.size(),
    //                                               total_mats.at(index).get_pos(2, 0) / (float)results.size());
    //             }
    //         }
    //         index++;
    //     }
    //     iterate++;
    // }

    // print out the final result
    // for (Matrix ele : total_mats)
    // {
    //     ele.print_str();
    // }

    // opens a file with specified answer path name
    std::string output_filename = (idx.compare("other") != 0) ? answer_path_name + idx + "-output2.txt" : answer_path_name;
    std::ofstream out_file(output_filename);
    if (out_file.is_open())
    {
        out_file << (int)total_mats.size() << ", " << output_filename << std::endl;
        for (int i = 0; i < (int)total_mats.size(); ++i)
        {
            out_file << total_mats[i].get_pos(0, 0) << ", " << total_mats[i].get_pos(1, 0) << ", " << total_mats[i].get_pos(2, 0) << "\n";
        }
        out_file.close();
    }

    return 0;
}
