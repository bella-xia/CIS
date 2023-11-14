#include "io_read.h"

void read_calbody(std::string file_name, std::vector<std::vector<float>> &data_d,
                  std::vector<std::vector<float>> &data_a, std::vector<std::vector<float>> &data_c)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    // this is used to read the first three integers
    int n_d, n_a, n_c;
    char garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_d >> garbage >> n_a >> garbage >> n_c;
    read_helper(in_file, data_d, n_d);
    read_helper(in_file, data_a, n_a);
    read_helper(in_file, data_c, n_c);
    in_file.close();
}

void read_ct_fi(std::string file_name, std::vector<std::vector<float>> &data_b)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    // this is used to read the first three integers
    int n_b;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_b;
    read_helper(in_file, data_b, n_b);
    in_file.close();
}

void read_calreadings(std::string file_name, std::vector<f_data> &frames)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    // this is used to read the first three integers
    int n_d, n_a, n_c, n_f;
    char garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_d >> garbage >> n_a >> garbage >> n_c >> garbage >> n_f;
    for (int i = 0; i < n_f; i++)
    {
        f_data frame;
        frame.list = std::vector<char>();
        frame.list.push_back('d');
        frame.list.push_back('a');
        frame.list.push_back('c');
        frame.data_d = std::vector<std::vector<float>>();
        frame.data_a = std::vector<std::vector<float>>();
        frame.data_c = std::vector<std::vector<float>>();

        read_helper(in_file, frame.data_d, n_d);
        read_helper(in_file, frame.data_a, n_a);
        read_helper(in_file, frame.data_c, n_c);

        frames.push_back(frame);
    }
    in_file.close();
}

void read_empivot(std::string file_name, std::vector<f_data> &frames)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    // this is used to read the first three integers
    int n_g, n_f;
    char garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_g >> garbage >> n_f;
    for (int i = 0; i < n_f; i++)
    {
        f_data frame;
        frame.list = std::vector<char>();
        frame.list.push_back('g');
        frame.data_g = std::vector<std::vector<float>>();

        read_helper(in_file, frame.data_g, n_g);

        frames.push_back(frame);
    }
    in_file.close();
}

void read_optpivot(std::string file_name, std::vector<f_data> &frames)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    // this is used to read the first three integers
    int n_d, n_h, n_f;
    char garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_d >> garbage >> n_h >> garbage >> n_f;
    for (int i = 0; i < n_f; i++)
    {
        f_data frame;
        frame.list = std::vector<char>();
        frame.list.push_back('d');
        frame.list.push_back('h');
        frame.data_d = std::vector<std::vector<float>>();
        frame.data_h = std::vector<std::vector<float>>();

        read_helper(in_file, frame.data_d, n_d);
        read_helper(in_file, frame.data_h, n_h);

        frames.push_back(frame);
    }
    in_file.close();
}

void read_helper(std::ifstream &in_file, std::vector<std::vector<float>> &data, int num_read)
{
    float x, y, z;
    std::string in_line;
    char garbage;
    for (int i = 0; i < num_read; i++)
    {
        std::getline(in_file, in_line);
        std::stringstream(in_line) >> x >> garbage >> y >> garbage >> z;
        std::vector<float> temp = std::vector<float>();
        temp.push_back(x);
        temp.push_back(y);
        temp.push_back(z);
        data.push_back(temp);
    }
}

// read ProblemXMesh.sur
void p3_read_mesh(std::string file_name, Mesh &mesh)
{
    std::ifstream in_file(file_name);
    std::string in_line;
    int n_v; // num vertices
    float v1, v2, v3;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_v;
    for (int i = 0; i < n_v; i++)
    {
        std::getline(in_file, in_line);
        std::stringstream(in_line) >> v1 >> v2 >> v3;
        mesh.insert_vertex(v1, v2, v3);
    }
    int n_tr; // num triangles
    int v_idx1, v_idx2, v_idx3, n_idx1, n_idx2, n_idx3;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_tr;
    for (int i = 0; i < n_tr; ++i)
    {
        std::getline(in_file, in_line);
        std::stringstream(in_line) >> v_idx1 >> v_idx2 >> v_idx3 >> n_idx1 >> n_idx2 >> n_idx3;
        mesh.insert_triangle(mesh.get_vertex_at(v_idx1), mesh.get_vertex_at(v_idx2), mesh.get_vertex_at(v_idx3),
                             n_idx1, n_idx2, n_idx3, v_idx1, v_idx2, v_idx3);
    }
    in_file.close();
}

// read ProblemX-BodyY file
Matrix p3_read_body(std::string file_name, std::vector<Matrix> &marker)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    // this is used to read the first three integers
    int n_marker;
    float v1, v2, v3;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_marker;
    for (int i = 0; i < n_marker; i++)
    {
        std::getline(in_file, in_line);
        std::stringstream(in_line) >> v1 >> v2 >> v3;
        marker.push_back(Matrix(v1, v2, v3));
    }
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> v1 >> v2 >> v3;
    return Matrix(v1, v2, v3);
}

// read the SampleReading files
void p3_read_sample(std::string file_name, std::vector<f_data> &frames, int n_a, int n_b)
{
    std::ifstream in_file;
    in_file.open(file_name);
    std::string in_line;
    int n_s, n_f;
    char garbage;
    std::getline(in_file, in_line);
    std::stringstream(in_line) >> n_s >> garbage >> n_f;
    for (int i = 0; i < n_f; i++)
    {
        f_data frame;
        frame.list = std::vector<char>();
        frame.list.push_back('d');
        frame.list.push_back('a');
        frame.list.push_back('b');
        frame.data_a = std::vector<std::vector<float>>();
        frame.data_b = std::vector<std::vector<float>>();
        frame.data_d = std::vector<std::vector<float>>(); // dummy markers

        read_helper(in_file, frame.data_a, n_a);
        read_helper(in_file, frame.data_b, n_b);
        read_helper(in_file, frame.data_d, n_s - n_b - n_a);

        frames.push_back(frame);
    }
    in_file.close();
}

void vector_print(std::vector<std::vector<float>> &vec)
{
    for (int i = 0; i < (int)vec.size(); ++i)
    {
        std::cout << vec[i][0] << " " << vec[i][1] << " " << vec[i][2] << std::endl;
    }
}
