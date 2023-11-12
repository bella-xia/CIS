#include "mesh.h"
#include "registration.h"
#include "io_read.h"
#include <iostream>

std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks);
std::string formatLine(Matrix d, Matrix s);
std::string formatNum(float a);

int main(int argc, char **argv)
{

    std::string idx = argv[1];
    if (argc > 1)
    {
        idx = argv[1]; 
    }

    // create structures to store data
    std::vector<Matrix> body_a = std::vector<Matrix>();
    std::vector<Matrix> body_b = std::vector<Matrix>();
    std::vector<f_data> sample_reading = std::vector<f_data>();
    Mesh mesh;

    std::string answer_path_name = "../OUTPUT/PA3-";
    std::string answer_path_name_2;
    std::string sample_str;
    if (idx < "G") {
        answer_path_name_2 = "-Debug_Output";
        sample_str = "-Debug-SampleReadingsTest.txt";
    } else {
        answer_path_name_2 = "-Unknown_Output";
        sample_str = "-Unknown-SampleReadingsTest.txt";
    }
    std::string path_name = "";
    std::string debug_path_name = "../DATA/PA3-";

    // specify the names for each of the files
    std::string body_a_str = "../DATA/Problem3-BodyA.txt";
    std::string body_b_str = "../DATA/Problem3-BodyB.txt";
    std::string mesh_str = "../DATA/Problem3Mesh.sur";

    Matrix ptr_a = p3_read_body(body_a_str, body_a);
    p3_read_body(body_b_str, body_b);
    p3_read_mesh(mesh_str, mesh);
    p3_read_sample(debug_path_name + idx + sample_str, sample_reading, (int)body_a.size(), (int)body_b.size());

    Registration fa_reg = Registration();
    Registration fb_reg = Registration();
    std::vector<Matrix> d_ks = std::vector<Matrix>();
    std::vector<Frame> frames_aks = std::vector<Frame>();
    std::vector<Frame> frames_bks = std::vector<Frame>();

    // pass in a and d and c vectors
    for (int i = 0; i < (int)body_a.size(); ++i)
    {
        fa_reg.add_matrix_a(body_a[i]);
    }
    for (int i = 0; i < (int)body_b.size(); ++i)
    {
        fb_reg.add_matrix_a(body_b[i]);
    }
    for (int frame_num = 0; frame_num < (int)sample_reading.size(); ++frame_num)
    {

        for (int i = 0; i < (int)sample_reading[frame_num].data_a.size(); ++i)
        {
            // put all the A and D vectors into their corresponding position
            // because the least square equation takes the form F_a * a = A, F_d * b = D,
            // so A and D would be in matrix b
            fa_reg.add_matrix_b(Matrix(sample_reading[frame_num].data_a[i]));
            fb_reg.add_matrix_b(Matrix(sample_reading[frame_num].data_b[i]));
        }
        Frame fa = fa_reg.point_cloud_registration();
        Frame fb = fb_reg.point_cloud_registration();
        frames_aks.push_back(fa);
        frames_bks.push_back(fb);
        d_ks.push_back(fb.inverse() * (fa * ptr_a));
        fa_reg.clean_matrix_b();
        fb_reg.clean_matrix_b();
    }

    Frame reg = Frame();

    std::vector<Matrix> s_ks = std::vector<Matrix>();
    for (Matrix ele : d_ks)
    {
        s_ks.push_back(reg * ele);
    }

    std::vector<Matrix> output = matching(mesh, s_ks);

    std::string output_filename = answer_path_name + idx + answer_path_name_2;
    std::ofstream out_file(output_filename);
    if (out_file.is_open())
    {
        out_file << (int)sample_reading.size() << ", " << output_filename << std::endl;
        for (int frame_num; frame_num < (int)sample_reading.size(); ++frame_num)
        {
            
            std::string output_line = formatLine(d_ks[frame_num], output[frame_num]);
            out_file << output_line <<"\n";
        }
        out_file.close();
    }

}

std::vector<Matrix> matching(Mesh &mesh, const std::vector<Matrix> &q_ks)
{
    // uncomment the block below to perform the simple search
   /* std::vector<Matrix> closest_points;
    for(int i = 0; i < q_ks.size(); i++){
        closest_points.push_back(mesh.find_closest_point(q_ks.at(i)));
    }
*/
    std::vector<Matrix> closest_points = mesh.find_closest_point_advanced(q_ks);
    return closest_points;
}

std::string formatLine( Matrix d, Matrix s){
    std::string dx = formatNum(d.get_pos(0,0));
    std::string dy = formatNum(d.get_pos(1,0));
    std::string dz = formatNum(d.get_pos(2,0));
    std::string sx = formatNum(s.get_pos(0,0));
    std::string sy = formatNum(s.get_pos(1,0));
    std::string sz = formatNum(s.get_pos(2,0));
    float mg = (d - s).magnitude();
    std::stringstream stream;
    stream << std::fixed << std::setprecision(3) << mg;
    std::string mg_string = stream.str();
    return dx + " " + dy + " " + dz + "  " + sx + " " + sy + " " + sz + "  " + mg_string;

}

std::string formatNum(float a) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << a;
    std::string fill = "";
    std::string cur = stream.str();
    
    for (int i = 0; i < 7 - cur.length(); i++){
        fill += " ";
    }
    return fill + cur;
}