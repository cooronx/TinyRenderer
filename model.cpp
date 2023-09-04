#include "model.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

Model::Model(const char* filename)
    : verts_()
    , faces_()
{
    std::ifstream in;
    in.clear();
    in.open(filename, std::ifstream::in);
    if (in.fail()) {
        std::cerr << "cant open obj file" << std::endl;
        return;
    }
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line);
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vector3f v;
            for (int i = 0; i < 3; ++i)
                iss >> v[i];
            verts_.push_back(v);
        } else if (!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vector2f uv;
            for (int i = 0; i < 2; i++)
                iss >> uv[i];
            uv_.push_back(uv);
        } else if (!line.compare(0, 3, "vn ")) {
            iss >> trash >> trash;
            Vector3f normal;
            for (int i = 0; i < 3; i++)
                iss >> normal[i];
            norms_.push_back(normal);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<Vector3i> f;
            Vector3i temp {};
            iss >> trash;
            while (iss >> temp[0] >> trash >> temp[1] >> trash >> temp[2]) {
                for (int i = 0; i < 3; ++i) {
                    temp[i]--;
                }
                f.push_back(temp);
            }
            faces_.push_back(f);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << std::endl;
    // 加载纹理
    LoadTexture(filename, "_diffuse.tga", diffuse_map_);
    LoadTexture(filename, "_normal.tga", normal_map_);
    LoadTexture(filename, "_spec.tga", spec_map_);
}

Model::~Model() = default;

int Model::GetVertSize()
{
    return (int)verts_.size();
}

int Model::GetFaceSize()
{
    return (int)faces_.size();
}

std::vector<int> Model::GetVertexIndex(int idx)
{
    std::vector<int> face {};
    std::vector<Vector3i> tmp = faces_[idx];
    face.reserve(tmp.size());
    for (auto& i : tmp)
        face.push_back(i[0]);
    return face;
}

Vector3f Model::GetVertByIndex(int i)
{
    return verts_[i];
}

void Model::LoadTexture(const std::string& obj_filename, const std::string& suffix, TGAImage& image)
{
    size_t dot_pos = obj_filename.find_last_of('.');
    if (dot_pos == std::string::npos)
        return;
    auto diffuse_texture_filename = obj_filename.substr(0, dot_pos) + suffix;
    if (image.read_tga_file(diffuse_texture_filename.c_str()))
        image.flip_vertically();
    else
        std::cerr << "diffuse texture file read failed" << std::endl;
}

TGAColor Model::Diffuse(Vector2f uv)
{
    // Vector2i uv(uvf[0]*diffuse_map_.get_width(), uvf[1]*diffuse_map_.get_height());
    return diffuse_map_.get(static_cast<int>(uv.x()), static_cast<int>(uv.y()));
}

Vector2f Model::GetUVByIndex(int face_index, int vertex_cnt)
{
    int index = faces_[face_index][vertex_cnt][1];
    return { static_cast<int>(uv_[index].x() * diffuse_map_.get_width()),
        static_cast<int>(uv_[index].y() * diffuse_map_.get_height()) };
}

Vector3f Model::GetVertexNorm(size_t face_index, size_t vertex_num)
{
    int idx = faces_[face_index][vertex_num][2];
    norms_[idx].normalize();
    return norms_[idx];
}

Vector3f Model::GetVertPosByIndex(size_t face_index, size_t vertex_num)
{
    int idx = faces_[face_index][vertex_num][0];
    return verts_[idx];
}

Vector3f Model::Normal(Vector2f uv)
{
    // Vector2i uv(uvf[0]*normal_map_.get_width(), uvf[1]*normal_map_.get_height());
    TGAColor c = normal_map_.get(uv[0], uv[1]);
    Vector3f res;
    for (int i = 0; i < 3; i++)
        res[2 - i] = (float)c.raw[i] / 255.f * 2.f - 1.f; // 从(0,255)转换到(-1,1)
    return res;
}

float Model::Spec(Vector2f uvf)
{
    Vector2i uv(uvf[0] * spec_map_.get_width(), uvf[1] * spec_map_.get_height());
    return spec_map_.get(uv[0], uv[1]).raw[0] / 1.0f;
}
