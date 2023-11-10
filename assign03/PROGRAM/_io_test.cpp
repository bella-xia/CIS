#include <vector>
#include <iostream>
#include "io_read.h"

int main(int argc, char **argv)
{
    std::string file_name = "";
    std::string file_type = "";
    if (argc > 1)
    {
        file_name = argv[1];
        file_type = argv[2];
    }

    if (file_type == "calbody")
    {
        std::vector<std::vector<float>> data_d;
        std::vector<std::vector<float>> data_a;
        std::vector<std::vector<float>> data_c;
        read_calbody(file_name, data_d, data_a, data_c);
        std::cout << "data d: " << std::endl;
        vector_print(data_d);
        std::cout << "data a: " << std::endl;
        vector_print(data_a);
        std::cout << "data c: " << std::endl;
        vector_print(data_c);
    }
    else if (file_type == "calreadings")
    {
        std::vector<f_data> frames;
        read_calreadings(file_name, frames);
        for (int i = 0; i < (int)frames.size(); ++i)
        {
            std::cout << "Frame # " << i << std::endl;
            std::cout << "d" << std::endl;
            vector_print(frames[i].data_d);
            std::cout << "a" << std::endl;
            vector_print(frames[i].data_a);
            std::cout << "c" << std::endl;
            vector_print(frames[i].data_c);
        }
    }
    else if (file_type == "empivot")
    {
        std::vector<f_data> frames;
        read_empivot(file_name, frames);
        for (int i = 0; i < (int)frames.size(); ++i)
        {
            std::cout << "Frame # " << i << std::endl;
            std::cout << "g" << std::endl;
            vector_print(frames[i].data_g);
        }
    }
    else if (file_type == "optpivot")
    {
        std::vector<f_data> frames;
        read_optpivot(file_name, frames);
        for (int i = 0; i < (int)frames.size(); ++i)
        {
            std::cout << "Frame # " << i << std::endl;
            std::cout << "d" << std::endl;
            vector_print(frames[i].data_d);
            std::cout << "h" << std::endl;
            vector_print(frames[i].data_h);
        }
    }

    return 0;
}