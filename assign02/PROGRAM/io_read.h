#ifndef IO_READ_H
#define IO_READ_H

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

/* 
 * f_data: data structure. It contains:
 * 1. data points in different frame stored in vector<vector<float>> form,
 * where the first layer of vector contains all frames, and the second layer
 * consists of the pointers in the frame.
 * 2. frame number: an integer counting the frame number.
 * 3. a list to store which kinds of data are stored.
*/

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

// print out a vector
void vector_print(std::vector<std::vector<float>> &vec);

/**
 * @param file_name the input file name (a calbody file)
 * @param data_d, data_a, data_c where the data will read into
*/
void read_calbody(std::string file_name, std::vector<std::vector<float>> &data_d,
                  std::vector<std::vector<float>> &data_a, std::vector<std::vector<float>> &data_c);

/**
 * @param file_name the input file name (a calreading file)
 * @param frames where the data will read into
*/
void read_calreadings(std::string file_name, std::vector<f_data> &frames);

/**
 * @param file_name the input file name (a empivot file, a em_fiducial file or a EM_nav file)
 * @param frames where the data will read into
*/
void read_empivot(std::string file_name, std::vector<f_data> &frames);

/**
 * @param file_name the input file name (opt_pivot file)
 * @param frames where the data will read into
*/
void read_optpivot(std::string file_name, std::vector<f_data> &frames);

/**
 * @param file_name the input file name (ct_fiducial file)
 * @param frames where the data will read into
*/
void read_ct_fi(std::string file_name, std::vector<std::vector<float>> &data_b);

/**
 * read num_read number of data from in_file string to data.
*/
void read_helper(std::ifstream &in_file, std::vector<std::vector<float>> &data, int num_read);

#endif