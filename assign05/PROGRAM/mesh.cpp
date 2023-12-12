#include "mesh.h"

Mesh::Mesh() : m_vertices(std::vector<Matrix>()), m_triangles(std::vector<TriangleMesh>()),
               m_num_vertices(0), m_num_triangles(0)
{
    initiate_lambdas();
}

Mesh::Mesh(int n_m, std::vector<std::vector<Matrix>> modes) : m_vertices(std::vector<Matrix>()), m_modes(modes), m_triangles(std::vector<TriangleMesh>()),
                                                              m_num_vertices(0), m_num_triangles(0)
{
    initiate_lambdas(n_m);
}

Mesh::~Mesh()
{
}

void Mesh::initiate_lambdas(int n_m)
{
    m_lambdas = std::vector<float>();
    for (int i = 0; i < n_m; ++i)
    {
        m_lambdas.push_back(0);
    }
}

void Mesh::insert_vertex(float m1, float m2, float m3)
{
    m_vertices.push_back(Matrix(m1, m2, m3));
    m_num_vertices++;
}
void Mesh::insert_triangle(Matrix m1, Matrix m2, Matrix m3,
                           int n_idx1, int n_idx2, int n_idx3, int v_idx1, int v_idx2, int v_idx3)
{
    m_triangles.push_back(TriangleMesh(&m_modes, n_idx1, n_idx2, n_idx3, v_idx1, v_idx2, v_idx3, &m_lambdas));
    m_num_triangles++;
}

Matrix Mesh::get_vertex_at(int idx)
{
    assert((int)m_vertices.size() > idx);
    return m_vertices.at(idx);
}

std::tuple<TriangleMesh, Matrix> Mesh::find_closest_point(Matrix mat)
{
    Matrix min_mat;            // closest point
    float min_dist = INFINITY; // bound
    TriangleMesh min_tri;

    // loop through all triangles to find the closest point
    for (int i = 0; i < m_num_triangles; i++)
    {
        TriangleMesh trig = m_triangles.at(i);
        std::tuple<float, Matrix> trig_find = trig.find_closest_point_in_triangle(mat);
        float dist = std::get<0>(trig_find);

        // update the bound and closest point if the current one is closer to the
        // target than the original closest point.
        if (dist < min_dist)
        {
            min_dist = dist;
            min_mat = std::get<1>(trig_find);
            min_tri = trig;
        }
    }

    return std::make_tuple(min_tri, min_mat);
}

std::vector<std::tuple<TriangleMesh, Matrix>> Mesh::find_closest_point_advanced(const std::vector<Matrix> &mat, BoundingBoxTreeNode *node) const
{
    // vectors to store the closest point to each target matrix.
    std::vector<std::tuple<TriangleMesh, Matrix>> closest_sets;

    // initialize the closest point for each target matrix.
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        closest_sets.push_back(std::make_tuple(TriangleMesh(), Matrix(3, 1)));
    }

    // find the closest point in each target matrix using the bounding box search
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        float bound = INFINITY;
        node->findClosestPoint(mat.at(i), bound, closest_sets.at(i));
    }
    return closest_sets;
}

std::tuple<Frame, std::vector<std::tuple<TriangleMesh, Matrix>>> Mesh::deformed_find_optimum_transformation(const std::vector<Matrix> &mat,
                                                                                                            float threshold, bool advanced)
{
    std::vector<Matrix> mat_copy(mat);
    // initiate a frame transformation with identity rotation and zero translation
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = 10000;
    float cur_err = 1000;
    Frame f_reg = Frame();
    std::vector<std::tuple<TriangleMesh, Matrix>> closest_set;

    while (1)
    {
        Matrix q0k(3 * (int)mat_copy.size(), 1);
        Matrix qmk(3 * (int)mat_copy.size(), (int)m_lambdas.size());
        auto iter_output = debug_find_optimum_transformation(f_reg, mat_copy, threshold, advanced);
        f_reg = std::get<0>(iter_output);
        closest_set = std::get<1>(iter_output);
        pre_err = cur_err;
        cur_err = std::get<2>(iter_output);

        if (cur_err / pre_err > threshold)
        {
            std::cout<<"cur_err: " << cur_err << "pre_err: "<< pre_err<<std::endl;
            break;
        }
        float err = 0;
        for (int i = 0; i < (int)mat_copy.size(); ++i)
        {
            err += (f_reg * mat_copy.at(i) - std::get<1>(closest_set.at(i))).magnitude();
        }
        err /= (int)mat_copy.size();
        std::cout << "current error at deformed level: " << cur_err << " " << err << std::endl;
        std::tuple<float, float, float> coefs;
        for (int i = 0; i < (int)closest_set.size(); ++i)
        {
            TriangleMesh tri = std::get<0>(closest_set.at(i));
            Matrix c = std::get<1>(closest_set.at(i));
            coefs = tri.get_barycentric_coefficient(c);
            Matrix q0k_mat = tri.get_original_coord(0) * std::get<0>(coefs) +
                             tri.get_original_coord(1) * std::get<1>(coefs) + 
                             tri.get_original_coord(2) * std::get<2>(coefs);
            q0k.assign(0 + 3 * i, 0, q0k_mat.get_pos(0, 0));
            q0k.assign(1 + 3 * i, 0, q0k_mat.get_pos(1, 0));
            q0k.assign(2 + 3 * i, 0, q0k_mat.get_pos(2, 0));

            for (int j = 0; j < (int)m_lambdas.size(); ++j)
            {
                Matrix qmk_mat = tri.get_mode_coord(0, j) * std::get<0>(coefs) +
                                 tri.get_mode_coord(1, j) * std::get<1>(coefs) + tri.get_mode_coord(2, j) * std::get<2>(coefs);
                qmk.assign(0 + 3 * i, j, qmk_mat.get_pos(0, 0));
                qmk.assign(1 + 3 * i, j, qmk_mat.get_pos(1, 0));
                qmk.assign(2 + 3 * i, j, qmk_mat.get_pos(2, 0));
            }
        }
        Matrix s(3 * (int)mat_copy.size(), 1);
        // recalculate the estimate for each point using the finalized frame
        // transformation estimate
        for (int k = 0; k < (int)mat_copy.size(); ++k)
        {
            Matrix f_mat = f_reg * mat_copy.at(k);
            s.assign(0 + 3 * k, 0, f_mat.get_pos(0, 0));
            s.assign(1 + 3 * k, 0, f_mat.get_pos(1, 0));
            s.assign(2 + 3 * k, 0, f_mat.get_pos(2, 0));
        }

        // Equation:
        // s = q0k +  qmk * lambdas ^T
        // qmk * lambdas ^T = s - q0k
        Matrix s_minus_q0k = s - q0k;
        //std::cout << "before: " << std::endl;

        //std::cout << s_minus_q0k.magnitude() << std::endl;

        // least square
        //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        Matrix lambda = Matrix(qmk.get().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(s_minus_q0k.get()));
        
        Matrix output = q0k + qmk * lambda;
        
        err = 0;
        float after_dis_err = 0;
        for (int i = 0; i < (int)mat_copy.size(); ++i){
            TriangleMesh tri = std::get<0>(closest_set.at(i));
            auto coefs = tri.get_barycentric_coefficient(std::get<1>(closest_set.at(i)));
            Matrix q0k_mat =  tri.get_original_coord(0) * std::get<0>(coefs)
                                +tri.get_original_coord(1) * std::get<1>(coefs) 
                                + tri.get_original_coord(2) * std::get<2>(coefs);
            Matrix err_mat = f_reg * mat_copy.at(i) - q0k_mat;
            Matrix after_dis_err_mat = f_reg * mat_copy.at(i) - q0k_mat;
            for (int j = 0; j < (int)m_lambdas.size(); ++j){
                Matrix qmk_mat = tri.get_mode_coord(0,j) * std::get<0>(coefs)
                                +tri.get_mode_coord(1, j) * std::get<1>(coefs) 
                                + tri.get_mode_coord(2, j) * std::get<2>(coefs);
            after_dis_err_mat = after_dis_err_mat - qmk_mat * lambda.get_pos(j, 0);
            err_mat = err_mat - qmk_mat * tri.get_lambda().at(j);
            }
            err += err_mat.magnitude();
            after_dis_err += after_dis_err_mat.magnitude();
        }
        err /= (int)mat_copy.size();
        after_dis_err/=(int)mat_copy.size();
        std::cout<< "error before distortion"<<err <<std::endl
                << "error after distortion" << after_dis_err<< std::endl;
        Matrix original_lambda = Matrix((int)m_lambdas.size(), 1); 
        for(int i = 0; i < (int)m_lambdas.size(); i++) {
            original_lambda.assign(i,0,std::get<0>(closest_set.at(0)).get_lambda().at(i));
        }

        float mag = (s -( q0k + qmk * original_lambda)).magnitude();
        float after_dis_mag = (s -( q0k + qmk * lambda)).magnitude();
        std::cout<<"total magnitude before distortion" <<mag <<std::endl
                <<"total magnitude after distortion" << after_dis_mag << std::endl;
        for (int z = 0; z < (int)m_lambdas.size(); ++z)
        {
            //m_lambdas.clear();
            m_lambdas.at(z) = lambda.get_pos(z, 0);
            
        }

        //std::vector<Matrix> adjusted_cks;
        //std::cout << "TriangleMesh2: " << std::get<0>(closest_set.at(0)).get_coord(0).as_str()
                  //<< std::get<0>(closest_set.at(0)).get_coord(1).as_str() << std::get<0>(closest_set.at(0)).get_coord(2).as_str() << std::endl;
        //std::cout << "point2: " << output.get_pos(0, 0) << " " << output.get_pos(1, 0) << output.get_pos(2, 0) << std::endl;
        /*
        for (int m = 0; m < (int)mat_copy.size(); ++m)
        {
            adjusted_cks.push_back(Matrix(output.get_pos(3 * m, 0),
                                          output.get_pos(1 + 3 * m, 0), output.get_pos(2 + 3 * m, 0)));
        }
        */
        //std::cout << "target triangle: " << targetTri.get_coord(0).as_str()
                  //<< targetTri.get_coord(1).as_str() << targetTri.get_coord(2).as_str() << std::endl;
        //std::cout<<"point: " << std::endl;
        //adjusted_cks.at(137).print_str();
        //Matrix adj_137 =  targetTri.get_coord(0) * std::get<0>(coef_ss.at(137)) +
                          //targetTri.get_coord(1) * std::get<1>(coef_ss.at(137)) +
                          //targetTri.get_coord(2) * std::get<2>(coef_ss.at(137));
        //std::cout<<"point expected: " << std::endl;
        //adj_137.print_str();
        //std::cout<<"distance" << std::get<0>(targetTri.find_closest_point_in_triangle(adjusted_cks.at(137)))<<std::endl;
        //std::cout<<"linear searching..."<<std::endl;
            //linear_search_triangle(adjusted_cks.at(0));

        
        
        //iter_output = debug_find_optimum_transformation(adjusted_cks, threshold, advanced);
    
    }
    return std::make_tuple(f_reg, closest_set);
}

std::tuple<Frame, std::vector<std::tuple<TriangleMesh, Matrix>>, float> Mesh::find_optimum_transformation(const std::vector<Matrix> &mat, float threshold, bool advanced)
{
    std::vector<Matrix> mat_copy(mat);
    // initiate a frame transformation with identity rotation and zero translation
    Frame estimate = Frame(Rotation(), Position());
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = 10000;
    float cur_err = 1000;

    // initiate the matrix to store the closest points
    std::vector<std::tuple<TriangleMesh, Matrix>> c_ks;

    // iterate while the ratio between current error and previous err has not exceeded the
    // threshold

    // allocate space for an array of BoundingSphere pointers.
    BoundingSphere **spheres = new BoundingSphere *[m_num_triangles];
    int nSphere = 0;

    for (int i = 0; i < m_num_triangles; i++)
    {
        spheres[nSphere++] = new BoundingSphere(m_triangles.at(i));
    }

    // construct BoundingBoxTreeNode
    BoundingBoxTreeNode node(spheres, nSphere, false);
    
    while (cur_err / pre_err <= threshold)
    {
        // find the current error and modify the frame transformation as well as the closest point
        // sets using find_transformation_helper function
        pre_err = cur_err;
        cur_err = find_transformation_helper(mat_copy, c_ks, estimate, &node, advanced,false);
        std::cout << "current error: " << cur_err << std::endl;
    }
    c_ks.clear();
    std::vector<Matrix> s;
    // recalculate the estimate for each point using the finalized frame
    // transformation estimate
    for (Matrix m : mat)
    {
        s.push_back(estimate * m);
    }
    // recalculate the closest points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s, &node);
    
    }
    else
    {
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    // return both the frame transformation estimates and the set of closest points
    for (int i = 0; i < nSphere; ++i)
    {
        delete spheres[i]; // Deletes each BoundingSphere object
    }

    delete[] spheres;
    
    return std::make_tuple(estimate, c_ks, cur_err);
}

std::tuple<Frame, std::vector<std::tuple<TriangleMesh, Matrix>>, float> Mesh::debug_find_optimum_transformation(Frame estimate, const std::vector<Matrix> &mat, float threshold, bool advanced)
{
    std::vector<Matrix> mat_copy(mat);
    // initiate a frame transformation with identity rotation and zero translation
    
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = 10000;
    float cur_err = 1000;

    // initiate the matrix to store the closest points
    std::vector<std::tuple<TriangleMesh, Matrix>> c_ks;

    // iterate while the ratio between current error and previous err has not exceeded the
    // threshold

    // allocate space for an array of BoundingSphere pointers.
    BoundingSphere **spheres = new BoundingSphere *[m_num_triangles];
    int nSphere = 0;

    for (int i = 0; i < m_num_triangles; i++)
    {
        spheres[nSphere++] = new BoundingSphere(m_triangles.at(i));
    }

    // construct BoundingBoxTreeNode
    BoundingBoxTreeNode node(spheres, nSphere, false);
    
    while (cur_err / pre_err <= threshold)
    {
        // find the current error and modify the frame transformation as well as the closest point
        // sets using find_transformation_helper function
        pre_err = cur_err;
        cur_err = debug_find_transformation_helper(mat_copy, c_ks, estimate, &node, advanced,false);
        std::cout << "current error: " << cur_err << std::endl;
    }
    c_ks.clear();
    std::vector<Matrix> s;
    // recalculate the estimate for each point using the finalized frame
    // transformation estimate
    for (Matrix m : mat)
    {
        s.push_back(estimate * m);
    }
    // recalculate the closest points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s, &node);
    
    }
    else
    {
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    // return both the frame transformation estimates and the set of closest points
    for (int i = 0; i < nSphere; ++i)
    {
        delete spheres[i]; // Deletes each BoundingSphere object
    }

    delete[] spheres;
    
    return std::make_tuple(estimate, c_ks, cur_err);
}
float Mesh::find_transformation_helper(std::vector<Matrix> &mat, std::vector<std::tuple<TriangleMesh, Matrix>> &c_ks, Frame &frame, BoundingBoxTreeNode *node,
                                       bool advanced, bool has_outlier)
{
    c_ks.clear();
    // find the current estimates for points
    std::vector<Matrix> s;
    for (Matrix m : mat)
    {
        s.push_back(frame * m);
    }
    // find the closest points for each of the points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s, node);
    }
    else
    {
        c_ks.clear();
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    // perform point-cloud registration. Adding all the closest points to
    // matrix b (before transformation) and all the estimate points to matrix a
    // (after transformation)
    Registration regis = Registration();
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        regis.add_matrix_a(s.at(i));
        regis.add_matrix_b(std::get<1>(c_ks.at(i)));
    }
    // modify the current frame transformation estimate with the new residual frame
    // transformation
    Frame modification = regis.point_cloud_registration();
    // std::cout << "frame: " << std::endl;
    // modification.get_rot().get_rot().print_str();
    // modification.get_pos().get_pos().print_str();
    frame = regis.point_cloud_registration() * frame;

    // initiate error float, and add the error magnitude of each point
    float err = 0;
    std::vector<Matrix> s_s; 
    std::vector<Matrix> c_s; 
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        s_s.push_back(frame * mat.at(i));
        c_s.push_back(std::get<1>(c_ks.at(i)));
    }
    err = get_error(s_s, c_s);

    // if outliers are expected to be excluded, then delete any point whose error is larger than
    // three times the average error
    if (has_outlier)
    {
        for (int i = 0; i < (int)mat.size(); ++i)
        {
            float individual_err = ((frame * mat.at(i)) - std::get<1>(c_ks.at(i))).magnitude();
            if (individual_err > 3 * err)
            {
                mat.erase(mat.begin() + i);
                std::cout << "erased matrix at index " << i << std::endl;
            }
        }
    }
    // return the average error
    return err;
}

float Mesh::debug_find_transformation_helper(std::vector<Matrix> &mat, std::vector<std::tuple<TriangleMesh, Matrix>> &c_ks, Frame &frame, BoundingBoxTreeNode *node,
                                       bool advanced, bool has_outlier)
{
    c_ks.clear();
    // find the current estimates for points
    std::vector<Matrix> s;
    for (Matrix m : mat)
    {
        s.push_back(frame * m);
    }
    // find the closest points for each of the points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s, node);
    }
    else
    {
        c_ks.clear();
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    float err = 0;
    std::vector<Matrix> s_s; 
    std::vector<Matrix> c_s; 
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        c_s.push_back(std::get<1>(c_ks.at(i)));
    }
    err = get_error(s, c_s);
    std::cout<<"original err: "<<err<<std::endl;
    // perform point-cloud registration. Adding all the closest points to
    // matrix b (before transformation) and all the estimate points to matrix a
    // (after transformation)
    Registration regis = Registration();
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        regis.add_matrix_a(s.at(i));
        regis.add_matrix_b(std::get<1>(c_ks.at(i)));
    }
    // modify the current frame transformation estimate with the new residual frame
    // transformation
    Frame modification = regis.point_cloud_registration();
    // std::cout << "frame: " << std::endl;
    // modification.get_rot().get_rot().print_str();
    // modification.get_pos().get_pos().print_str();
    frame = regis.point_cloud_registration() * frame;

    // initiate error float, and add the error magnitude of each point
    err = 0;
    s_s.clear(); 
    c_s.clear(); 
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        s_s.push_back(frame * mat.at(i));
        c_s.push_back(std::get<1>(c_ks.at(i)));
    }
    err = get_error(s_s, c_s);

    // if outliers are expected to be excluded, then delete any point whose error is larger than
    // three times the average error
    if (has_outlier)
    {
        for (int i = 0; i < (int)mat.size(); ++i)
        {
            float individual_err = ((frame * mat.at(i)) - std::get<1>(c_ks.at(i))).magnitude();
            if (individual_err > 3 * err)
            {
                mat.erase(mat.begin() + i);
                std::cout << "erased matrix at index " << i << std::endl;
            }
        }
    }
    // return the average error
    return err;
}
/*
void Mesh::linear_search_triangle(Matrix m) {
    float closest = 10000;
    int closet_idx = 0;
    m = Matrix (7.694596, -7.689957, 23.280283);

    for(int i = 564; i < 565; i++) {
        auto clo = m_triangles.at(i).debug_find_closest_point_in_triangle(m);
        Matrix closest_point = std::get<1>(clo);
        auto clo_2 = m_triangles.at(i).debug_find_closest_point_in_triangle(closest_point);
        Matrix closest_point2 = std::get<1>(clo_2);
        if((closest_point - closest_point2).magnitude() > 0.0001) {
            std::cout<<"--------------------"<<std::endl;
            std::cout<<"not matching:"<<std::endl;
            closest_point.print_str();
            closest_point2.print_str();
            std::cout<<"triangle index: "<<i << std::endl;
            std::cout << "found triangle: " << std::endl;
            std::cout<<m_triangles.at(i).get_coord(0).as_str()
                  << m_triangles.at(i).get_coord(1).as_str() << m_triangles.at(i).get_coord(2).as_str() << std::endl;
            std::cout<<"original mat:" <<std::endl;
            m.print_str(); 
            
            
        }
        float cur_close = std::get<0>(clo);
        if(cur_close < closest) {
            closet_idx = i;
            closest = cur_close;
            //std::cout<<"dist " << closest<<std::endl;
        }
    }
    //std::cout<<"closest distance: "<< closest<<std::endl;
    //std::cout << "found triangle: " << m_triangles.at(closet_idx).get_coord(0).as_str()
                  //<< m_triangles.at(closet_idx).get_coord(1).as_str() << m_triangles.at(closet_idx).get_coord(2).as_str() << std::endl;
    
}



std::tuple<Frame, std::vector<std::tuple<TriangleMesh, Matrix>>, float> Mesh::debug_find_optimum_transformation(const std::vector<Matrix> &mat, float threshold, bool advanced)
{
    std::vector<Matrix> mat_copy(mat);
    // initiate a frame transformation with identity rotation and zero translation
    Frame estimate = Frame(Rotation(), Position());
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = 10000;
    float cur_err = 1000;

    // initiate the matrix to store the closest points
    std::vector<std::tuple<TriangleMesh, Matrix>> c_ks;

    // iterate while the ratio between current error and previous err has not exceeded the
    // threshold

    // allocate space for an array of BoundingSphere pointers.
    BoundingSphere **spheres = new BoundingSphere *[m_num_triangles];
    int nSphere = 0;

    for (int i = 0; i < m_num_triangles; i++)
    {
        spheres[nSphere++] = new BoundingSphere(m_triangles.at(i));
    }

    // construct BoundingBoxTreeNode
    BoundingBoxTreeNode node(spheres, nSphere, false);

    while (cur_err / pre_err <= threshold)
    {
        // find the current error and modify the frame transformation as well as the closest point
        // sets using find_transformation_helper function
        pre_err = cur_err;
        c_ks = find_closest_point_advanced(mat, &node);
        cur_err = 0;
        for (int i = 0; i < (int)mat.size(); ++i)
        {
            std::cout<<"target mat - found mat at "<<i <<std::endl;
            (mat.at(i) - std::get<1>(c_ks.at(i))).print_str();
            cur_err += (mat.at(i) - std::get<1>(c_ks.at(i))).magnitude();
        }
        

        cur_err /= (int)mat.size();

        //cur_err = find_transformation_helper(mat_copy, c_ks, estimate, &node, advanced);
        std::cout << "debug: current error: " << cur_err << std::endl;
    }
    c_ks.clear();
    std::vector<Matrix> s;
    // recalculate the estimate for each point using the finalized frame
    // transformation estimate
    for (Matrix m : mat)
    {
        s.push_back(estimate * m);
    }
    // recalculate the closest points
    if (advanced)
    {
        // std::tuple<TriangleMesh, Matrix> c_0 = find_closest_point(s.at(0));
        c_ks = find_closest_point_advanced(s, &node);
        //std::cout << "TriangleMesh1: " << std::get<0>(c_ks.at(0)).get_coord(0).as_str()
                  //<< std::get<0>(c_ks.at(0)).get_coord(1).as_str() << std::get<0>(c_ks.at(0)).get_coord(2).as_str() << std::endl;
        //std::cout << "point1: " << std::get<1>(c_ks.at(0)).as_str() << std::endl;
    }
    else
    {
        c_ks.clear();
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    // return both the frame transformation estimates and the set of closest points
    return std::make_tuple(estimate, c_ks, cur_err);
}
*/
float Mesh::get_error(std::vector<Matrix> s_s, std::vector<Matrix> c_s) {
    Matrix s_mat = Matrix(s_s.size() * 3, 1);
    Matrix c_mat = Matrix(c_s.size() * 3, 1);
    for(int i = 0; i < c_s.size(); i++) {
        s_mat.assign(0 + i * 3, 0, s_s.at(i).get_pos(0,0));
        s_mat.assign(1 + i * 3, 0, s_s.at(i).get_pos(1,0));
        s_mat.assign(2 + i * 3, 0, s_s.at(i).get_pos(2,0));
        c_mat.assign(0 + i * 3, 0, c_s.at(i).get_pos(0,0));
        c_mat.assign(1 + i * 3, 0, c_s.at(i).get_pos(1,0));
        c_mat.assign(2 + i * 3, 0, c_s.at(i).get_pos(2,0));
    }
    return (s_mat-c_mat).magnitude();
}