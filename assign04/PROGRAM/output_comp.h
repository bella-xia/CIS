#ifndef OUTPUT_COMP_H
#define OUTPUT_COMP_H

#include "io_read.h"
#include "registration.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <random>
#include <chrono>
/**
 * calculate the statistics about the results
*/

// get the std and mean of the error
std::tuple<float, float> get_m_and_std(std::string expected_output_file, std::string output_file);

// randomly generate results.
std::tuple<Matrix, Matrix> get_pos(std::vector<f_data> &em_frames,
                                   std::vector<f_data> &opt_frames,
                                   std::vector<std::vector<float>> &data_d);
// calculate the std and mean of the error
std::tuple<float, float> calculate_m_and_std(std::vector<float> diff);

#endif