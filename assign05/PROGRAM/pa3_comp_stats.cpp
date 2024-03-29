#include <cmath>
#include <numeric>
#include <algorithm>
#include <random>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

/**
 * helper function to calculate mean and standard deviation in a given set of values
 */
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

/**
 * helper function read the expected values in the specified file
 */
void read_file_diff(std::string filename, std::vector<float> &diff, bool closest_point)
{
    std::ifstream in_file;
    in_file.open(filename);
    std::string in_line;
    int n_diff;
    float d1, d2, d3;
    float garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_diff;
    for (int i = 0; i < n_diff; ++i)
    {
        std::getline(in_file, in_line);
        if (closest_point)
        {
            std::stringstream(in_line) >> garbage >> garbage >> garbage >> d1 >> d2 >> d3;
        }
        else
        {
            std::stringstream(in_line) >> d1 >> d2 >> d3;
        }
        diff.push_back(d1);
        diff.push_back(d2);
        diff.push_back(d3);
    }
    in_file.close();
}

/**
 * helper function to read actual output and calculate its differences with the expected one
 */
void read_get_err(std::string filename, std::vector<float> &err, const std::vector<float> &expected, bool closest_point)
{
    std::ifstream in_file;
    in_file.open(filename);
    std::string in_line;
    int n_diff;
    float d1, d2, d3;
    float garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_diff;
    for (int i = 0; i < n_diff; ++i)
    {
        std::getline(in_file, in_line);
        if (closest_point)
        {
            std::stringstream(in_line) >> garbage >> garbage >> garbage >> d1 >> d2 >> d3;
        }
        else
        {
            std::stringstream(in_line) >> d1 >> d2 >> d3;
        }
        err.push_back(expected.at(i * 3) - d1);
        err.push_back(expected.at(i * 3 + 1) - d2);
        err.push_back(expected.at(i * 3 + 2) - d3);
    }
    in_file.close();
}

int main(int argc, char **argv)
{
    // specify constants for filenames
    std::string expected_str = "../DATA/PA3-";
    std::string output_with_regis_str = "../OUTPUT/PA3-";
    std::string output_without_regis_str = "../TEST/PA3-";
    std::string expected_end_str = "-Debug-Output.txt";
    std::string output_end_str = "-Debug_Output.txt";
    std::string output_filename = "../TEST/pa3_stats.txt";
    std::string input = "";
    if (argc > 1)
    {
        input = argv[1];
        if (input == "4")
        {
            expected_str = "../DATA/PA4-";
            output_with_regis_str = "../OUTPUT/PA4-";
            output_filename = "../TEST/pa4_stats.txt";
        }
        else if (input == "5")
        {
            expected_str = "../DATA/PA5-HighRes-";
            output_with_regis_str = "../OUTPUT/PA5-HighRes-";
            output_filename = "../TEST/pa5_stats.txt";
        }
    }
    std::vector<std::string> idxs({"A", "B", "C", "D", "E", "F"});
    // pa3 check
    if (input == "")
    {
        // intialize data structures to store all the data
        std::vector<float> expected_points;
        std::vector<float> expected_dks;
        std::vector<float> with_regis_err;
        std::vector<float> without_regis_err;
        std::vector<float> regis_err;

        std::vector<float> with_regis_means;
        std::vector<float> with_regis_stds;
        std::vector<float> without_regis_means;
        std::vector<float> without_regis_stds;
        std::vector<float> regis_means;
        std::vector<float> regis_stds;

        for (int i = 0; i < (int)idxs.size(); ++i)
        {
            // read the expected for closest point with registration step;
            // closest point by directly using the given registration information;
            // expected for the registration, the calculate the error for each instance
            read_file_diff(expected_str + idxs.at(i) + expected_end_str, expected_points, true);
            read_file_diff(expected_str + idxs.at(i) + expected_end_str, expected_dks, false);
            read_get_err(output_with_regis_str + idxs.at(i) + output_end_str, with_regis_err, expected_points, true);
            read_get_err(output_without_regis_str + idxs.at(i) + output_end_str, without_regis_err, expected_points, true);
            read_get_err(output_with_regis_str + idxs.at(i) + output_end_str, regis_err, expected_dks, false);

            std::tuple<float, float> with_regis = calculate_m_and_std(with_regis_err);
            std::tuple<float, float> without_regis = calculate_m_and_std(without_regis_err);
            std::tuple<float, float> regis = calculate_m_and_std(regis_err);

            // calcualte the mean and standard deviation

            with_regis_means.push_back(std::get<0>(with_regis));
            without_regis_means.push_back(std::get<0>(without_regis));
            regis_means.push_back(std::get<0>(regis));

            with_regis_stds.push_back(std::get<1>(with_regis));
            without_regis_stds.push_back(std::get<1>(without_regis));
            regis_stds.push_back(std::get<1>(regis));

            expected_points.clear();
            expected_dks.clear();
            with_regis_err.clear();
            without_regis_err.clear();
            regis_err.clear();
        }
        // output the result ih the specified file
        std::ofstream out_file(output_filename);
        if (out_file.is_open())
        {
            for (int i = 0; i < (int)idxs.size(); ++i)
            {
                out_file << "file " << idxs.at(i) << ":" << std::endl;
                out_file << "Error mean for closest point with registration process: " << with_regis_means.at(i) << std::endl;
                out_file << "Error standard deviation for closest point with registration process: " << with_regis_stds.at(i) << std::endl;
                out_file << "Error mean without for closest point registration process: " << without_regis_means.at(i) << std::endl;
                out_file << "Error standard deviation for closest point without registration process: " << without_regis_stds.at(i) << std::endl;
                out_file << "Error mean for registration process: " << regis_means.at(i) << std::endl;
                out_file << "Error standard deviation for registration process: " << regis_stds.at(i) << std::endl;
                out_file << std::endl;
            }

            out_file.close();
        }
    }
    // pa4 check
    else
    {
        // intialize data structures to store all the data
        std::vector<float> expected_points;
        std::vector<float> expected_dks;
        std::vector<float> with_regis_err;

        std::vector<float> with_regis_means;
        std::vector<float> with_regis_stds;

        // read the expected for closest point,
        // compare to the outputed closest point, then calculate the error for each instance
        for (int i = 0; i < (int)idxs.size(); ++i)
        {
            read_file_diff(expected_str + idxs.at(i) + expected_end_str, expected_points, true);
            read_file_diff(expected_str + idxs.at(i) + expected_end_str, expected_dks, false);
            read_get_err(output_with_regis_str + idxs.at(i) + output_end_str, with_regis_err, expected_points, true);

            std::tuple<float, float> with_regis = calculate_m_and_std(with_regis_err);

            // calculate mean and standard deviation

            with_regis_means.push_back(std::get<0>(with_regis));

            with_regis_stds.push_back(std::get<1>(with_regis));

            expected_points.clear();
            expected_dks.clear();
            with_regis_err.clear();
        }
        // output the result ih the specified file
        std::ofstream out_file(output_filename);
        if (out_file.is_open())
        {
            for (int i = 0; i < (int)idxs.size(); ++i)
            {
                out_file << "file " << idxs.at(i) << ":" << std::endl;
                out_file << "Error mean for closest point: " << with_regis_means.at(i) << std::endl;
                out_file << "Error standard deviation for closest point: " << with_regis_stds.at(i) << std::endl;
                out_file << std::endl;
            }

            out_file.close();
        }
    }
}