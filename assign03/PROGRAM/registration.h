#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "frame.h"
#include <iostream>

class Registration
{

/**
 * This class aim to provide an interface for point registration 
 * and pivot calibration calculation.
*/
private:
    std::vector<Matrix> eigens_a, eigens_b; // F * a = b
    int size_a, size_b;
    Eigen::MatrixXf mid_a, mid_b; // mean of a and mean of b respectively
    bool a_mid_calculated; // whether mean of a is calculated

public:
    /* construct the interface basing on the given a and b*/
    Registration() : eigens_a(std::vector<Matrix>()), eigens_b(std::vector<Matrix>()),
                     size_a(0), size_b(0), mid_a(Eigen::MatrixXf(3, 1)), mid_b(Eigen::MatrixXf(3, 1)),
                     a_mid_calculated(false){};

    /* add a new vector into a*/
    void add_matrix_a(Matrix eigen)
    {
        eigens_a.push_back(eigen);
        size_a++;
    }

    /* add a new vector into b*/
    void add_matrix_b(Matrix eigen)
    {
        eigens_b.push_back(eigen);
        size_b++;
    }

    /* delete all matrix b stored */
    void clean_matrix_b()
    {
        eigens_b.clear();
        size_b = 0;
    }

    /* delete all matrix a stored */
    void clean_matrix_a()
    {
        eigens_a.clear();
        size_a = 0;
        a_mid_calculated = false;
    }

    void calculate_midpoint_a();
    void calculate_midpoint_b();

    /* calculate all a values such that a = b - mid_b*/
    void get_matrix_a_from_b();

    // return the mid a
    Matrix get_midpoint_a() { return Matrix(mid_a); }
    // return the mid b
    Matrix get_midpoint_b() { return Matrix(mid_b); }

    // perform the point cloud registration basing on SVD
    Frame point_cloud_registration();

    // perform pivot calibration with a vector of frame transformations.
    Matrix pivot_calibration(const std::vector<Frame> &f);

    // helper function to construct the matrixes in pivot calibration
    void pass_matrix(Eigen::MatrixXf &rot_mat, Eigen::MatrixXf &pos_mat, const std::vector<Frame> &f);

private:
};

#endif