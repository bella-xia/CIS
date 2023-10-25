#ifndef IO_READ_H
#define IO_READ_H

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

struct f_data
{
    int frame_num;
    // place to store which ones are here
    std::vector<char> list;

    std::vector<std::vector<float>> data_a;
    std::vector<std::vector<float>> data_c;
    std::vector<std::vector<float>> data_d;
    std::vector<std::vector<float>> data_g;
    std::vector<std::vector<float>> data_h;
};

void vector_print(std::vector<std::vector<float>> &vec);

void read_calbody(std::string file_name, std::vector<std::vector<float>> &data_d,
                  std::vector<std::vector<float>> &data_a, std::vector<std::vector<float>> &data_c);

void read_calreadings(std::string file_name, std::vector<f_data> &frames);

void read_empivot(std::string file_name, std::vector<f_data> &frames);

void read_optpivot(std::string file_name, std::vector<f_data> &frames);

void read_ct_fi(std::string file_name, std::vector<std::vector<float>> &data_b);

void read_helper(std::ifstream &in_file, std::vector<std::vector<float>> &data, int num_read);

#endif